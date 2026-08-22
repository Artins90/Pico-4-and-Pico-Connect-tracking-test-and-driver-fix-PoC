#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <memory>
#include <mutex>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wignored-attributes"
#pragma clang diagnostic ignored "-Wunused-parameter"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <openvr_driver.h>

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

using HmdDriverFactoryFn = void* (*)(const char* pInterfaceName, int* pReturnCode);

static HMODULE g_hModule = nullptr;
static HMODULE g_hOrigDriver = nullptr;
static HmdDriverFactoryFn g_pfnOrigFactory = nullptr;
static std::once_flag g_loadOrigDriverOnce;

struct Vec3 {
    double x = 0, y = 0, z = 0;
};

static Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static Vec3 operator*(Vec3 a, double s) { return {a.x*s, a.y*s, a.z*s}; }

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
};

static Quat QNormalize(Quat q) {
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12) return {1, 0, 0, 0};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}
static Quat QConj(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }
static Quat QMul(Quat a, Quat b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

class HmdVelocitySynthesizer {
public:
    HmdVelocitySynthesizer() {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        perfFreq_ = static_cast<double>(freq.QuadPart);
    }

    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        hasPrev_ = false;
        smoothOmega_ = {0, 0, 0};
        smoothVel_ = {0, 0, 0};
    }

    void ProcessPose(vr::DriverPose_t& pose) {
        if (!pose.poseIsValid) {
            std::lock_guard<std::mutex> lock(mutex_);
            hasPrev_ = false;
            // Tracking just went invalid: whatever velocity we were smoothing
            // toward is no longer trustworthy (the HMD could resume anywhere,
            // moving in any direction, once tracking recovers). Previously
            // this only cleared hasPrev_, so smoothOmega_/smoothVel_ kept
            // their last pre-loss values -- and once a new valid pose arrived,
            // the "hasPrev_ == false" branch below re-syncs the reference pose
            // but does NOT touch smoothOmega_/smoothVel_, so that stale,
            // possibly-large velocity from before the tracking loss would get
            // written into the very first recovered pose. Clearing here means
            // the first pose(s) after recovery correctly report zero velocity
            // until a new real finite-difference measurement is available.
            smoothOmega_ = {0, 0, 0};
            smoothVel_ = {0, 0, 0};
            return;
        }

        LARGE_INTEGER counter;
        QueryPerformanceCounter(&counter);
        const double t = static_cast<double>(counter.QuadPart) / perfFreq_;

        Quat curQ = QNormalize({pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z});
        Vec3 curP = {pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]};

        std::lock_guard<std::mutex> lock(mutex_);

        if (hasPrev_) {
            const double dt = t - prevTime_;

            if (dt > kMaxReliableDt) {
                // A gap this long (compositor/game hitch, stall, or a tracking
                // blackout that didn't clear poseIsValid) makes any finite-
                // difference derivative computed over it unreliable, and
                // leaves whatever was previously smoothed stale relative to
                // however far the device may actually have moved during the
                // gap. Reset instead of continuing to report a (now stale)
                // velocity across the gap indefinitely.
                smoothOmega_ = {0, 0, 0};
                smoothVel_ = {0, 0, 0};
            } else if (dt >= kMinValidDt) {
                const bool poseChanged = (std::abs(curQ.w - prevQ_.w) > 1e-5 ||
                                          std::abs(curQ.x - prevQ_.x) > 1e-5 ||
                                          std::abs(curQ.y - prevQ_.y) > 1e-5 ||
                                          std::abs(curQ.z - prevQ_.z) > 1e-5 ||
                                          std::abs(curP.x - prevP_.x) > 1e-4 ||
                                          std::abs(curP.y - prevP_.y) > 1e-4 ||
                                          std::abs(curP.z - prevP_.z) > 1e-4);

                if (poseChanged) {
                    Quat dqWorld = QNormalize(QMul(curQ, QConj(prevQ_)));
                    if (dqWorld.w < 0.0) {
                        dqWorld.w = -dqWorld.w; dqWorld.x = -dqWorld.x; dqWorld.y = -dqWorld.y; dqWorld.z = -dqWorld.z;
                    }
                    const double halfW = std::clamp(dqWorld.w, 0.0, 1.0);
                    const double angle = 2.0 * std::acos(halfW);
                    Vec3 axis{dqWorld.x, dqWorld.y, dqWorld.z};
                    const double sn = std::sin(angle * 0.5);
                    if (sn > 1e-6) axis = axis * (1.0 / sn);
                    else axis = {0, 0, 0};

                    Vec3 derivedWorldOmega = axis * (angle / dt);
                    Vec3 derivedWorldVel = (curP - prevP_) * (1.0 / dt);

                    // Exponential smoothing normalized by the actual elapsed dt
                    // (alpha = 1 - exp(-dt/tau)) instead of a fixed alpha=0.80
                    // per-sample. A fixed per-sample alpha implicitly assumes a
                    // constant sample rate; since dt is allowed to range from
                    // kMinValidDt to kMaxReliableDt here, a fixed alpha means
                    // the filter's effective cutoff frequency (and therefore
                    // how much it smooths vs. lags) drifts with frame rate.
                    // kSmoothingTimeConstant is chosen so that at a nominal
                    // ~11.1ms sample interval (dt = -tau * ln(1 - alpha)),
                    // this reproduces the original fixed alpha=0.80 behavior,
                    // while degrading gracefully (more smoothing, proportionally
                    // less lag increase) at other frame times.
                    const double alpha = 1.0 - std::exp(-dt / kSmoothingTimeConstant);
                    smoothOmega_ = derivedWorldOmega * alpha + smoothOmega_ * (1.0 - alpha);
                    smoothVel_ = derivedWorldVel * alpha + smoothVel_ * (1.0 - alpha);
                } else if (t - prevTime_ > kStationaryTimeoutSec) {
                    smoothOmega_ = {0, 0, 0};
                    smoothVel_ = {0, 0, 0};
                }
            }
            // else: dt < kMinValidDt -- too small/noisy a sample interval to
            // trust (possible duplicate pose update or timer jitter). Skip
            // computing a new derivative this frame and keep the previously
            // smoothed value.

            // Always resync the reference pose, even on frames that fail the
            // dt-window check above, so prevTime_ can never go stale and
            // produce a huge synthetic velocity spike on the next valid frame.
            prevQ_ = curQ;
            prevP_ = curP;
            prevTime_ = t;
        } else {
            prevQ_ = curQ;
            prevP_ = curP;
            prevTime_ = t;
            hasPrev_ = true;
        }

        const bool omegaFinite = std::isfinite(smoothOmega_.x) && std::isfinite(smoothOmega_.y) && std::isfinite(smoothOmega_.z);
        const bool velFinite = std::isfinite(smoothVel_.x) && std::isfinite(smoothVel_.y) && std::isfinite(smoothVel_.z);

        if (omegaFinite && velFinite) {
            pose.vecAngularVelocity[0] = smoothOmega_.x;
            pose.vecAngularVelocity[1] = smoothOmega_.y;
            pose.vecAngularVelocity[2] = smoothOmega_.z;

            pose.vecVelocity[0] = smoothVel_.x;
            pose.vecVelocity[1] = smoothVel_.y;
            pose.vecVelocity[2] = smoothVel_.z;
        }
        // else: leave pose.vecVelocity / vecAngularVelocity as the original
        // driver set them rather than hand vrserver NaN/Inf.
    }

private:
    // dt bounds for trusting a finite-difference velocity estimate. Values
    // preserved from the original implementation; only how they're used
    // changed (see kMaxReliableDt handling above and the dt-normalized alpha).
    static constexpr double kMinValidDt = 0.003;
    static constexpr double kMaxReliableDt = 0.050;
    static constexpr double kStationaryTimeoutSec = 0.080;
    // tau such that alpha = 1 - exp(-dt/tau) reproduces the original fixed
    // alpha = 0.80 at a nominal dt of 11.1ms: tau = -dt / ln(1 - alpha).
    static constexpr double kSmoothingTimeConstant = 0.0069;

    std::mutex mutex_;
    double perfFreq_ = 1.0;
    Quat prevQ_{1, 0, 0, 0};
    Vec3 prevP_{0, 0, 0};
    Vec3 smoothOmega_{0, 0, 0};
    Vec3 smoothVel_{0, 0, 0};
    double prevTime_ = 0.0;
    bool hasPrev_ = false;
};

static HmdVelocitySynthesizer g_hmdSynthesizer;

// Proxy Server Driver Host to intercept TrackedDevicePoseUpdated
class ProxyServerDriverHost final : public vr::IVRServerDriverHost {
public:
    ProxyServerDriverHost(vr::IVRServerDriverHost* pRealHost) : m_pRealHost(pRealHost) {}
    ~ProxyServerDriverHost() = default;

    bool TrackedDeviceAdded(const char* pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver* pDriver) override {
        // Pass original, untouched pDriver directly to SteamVR so DirectMode & OpenXR work natively
        return m_pRealHost->TrackedDeviceAdded(pchDeviceSerialNumber, eDeviceClass, pDriver);
    }

    void TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t unPoseStructSize) override {
        vr::DriverPose_t fixedPose = newPose;
        if (unWhichDevice == vr::k_unTrackedDeviceIndex_Hmd) {
            g_hmdSynthesizer.ProcessPose(fixedPose);
        }
        m_pRealHost->TrackedDevicePoseUpdated(unWhichDevice, fixedPose, unPoseStructSize);
    }

    void VsyncEvent(double vsyncTimeOffsetSeconds) override {
        m_pRealHost->VsyncEvent(vsyncTimeOffsetSeconds);
    }

    void VendorSpecificEvent(uint32_t unWhichDevice, vr::EVREventType eventType, const vr::VREvent_Data_t& eventData, double eventTimeOffset) override {
        m_pRealHost->VendorSpecificEvent(unWhichDevice, eventType, eventData, eventTimeOffset);
    }

    bool IsExiting() override {
        return m_pRealHost->IsExiting();
    }

    bool PollNextEvent(vr::VREvent_t* pEvent, uint32_t uncbVREvent) override {
        return m_pRealHost->PollNextEvent(pEvent, uncbVREvent);
    }

    void GetRawTrackedDevicePoses(float fPredictedSecondsFromNow, vr::TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) override {
        m_pRealHost->GetRawTrackedDevicePoses(fPredictedSecondsFromNow, pTrackedDevicePoseArray, unTrackedDevicePoseArrayCount);
    }

    void RequestRestart(const char* pchLocalizedReason, const char* pchExecutableToStart, const char* pchArguments, const char* pchWorkingDirectory) override {
        m_pRealHost->RequestRestart(pchLocalizedReason, pchExecutableToStart, pchArguments, pchWorkingDirectory);
    }

    uint32_t GetFrameTimings(vr::Compositor_FrameTiming* pTiming, uint32_t nFrames) override {
        return m_pRealHost->GetFrameTimings(pTiming, nFrames);
    }

    void SetDisplayEyeToHead(uint32_t unWhichDevice, const vr::HmdMatrix34_t& eyeToHeadLeft, const vr::HmdMatrix34_t& eyeToHeadRight) override {
        m_pRealHost->SetDisplayEyeToHead(unWhichDevice, eyeToHeadLeft, eyeToHeadRight);
    }

    void SetDisplayProjectionRaw(uint32_t unWhichDevice, const vr::HmdRect2_t& eyeLeft, const vr::HmdRect2_t& eyeRight) override {
        m_pRealHost->SetDisplayProjectionRaw(unWhichDevice, eyeLeft, eyeRight);
    }

    void SetRecommendedRenderTargetSize(uint32_t unWhichDevice, uint32_t nWidth, uint32_t nHeight) override {
        m_pRealHost->SetRecommendedRenderTargetSize(unWhichDevice, nWidth, nHeight);
    }

private:
    vr::IVRServerDriverHost* m_pRealHost = nullptr;
};

// Proxy Driver Context to intercept IVRServerDriverHost
class ProxyDriverContext final : public vr::IVRDriverContext {
public:
    ProxyDriverContext(vr::IVRDriverContext* pRealCtx) : m_pRealCtx(pRealCtx) {}
    ~ProxyDriverContext() = default;

    void* GetGenericInterface(const char* pchInterfaceVersion, vr::EVRInitError* peError = nullptr) override {
        void* pRealInterface = m_pRealCtx->GetGenericInterface(pchInterfaceVersion, peError);
        if (!pRealInterface) return nullptr;

        if (std::strncmp(pchInterfaceVersion, "IVRServerDriverHost", 19) == 0) {
            if (!m_pProxyHost) {
                m_pProxyHost = std::make_unique<ProxyServerDriverHost>(reinterpret_cast<vr::IVRServerDriverHost*>(pRealInterface));
            }
            return m_pProxyHost.get();
        }
        return pRealInterface;
    }

    vr::DriverHandle_t GetDriverHandle() override {
        return m_pRealCtx->GetDriverHandle();
    }

private:
    vr::IVRDriverContext* m_pRealCtx = nullptr;
    std::unique_ptr<ProxyServerDriverHost> m_pProxyHost;
};

// Proxy Tracked Device Provider
class ProxyTrackedDeviceProvider final : public vr::IServerTrackedDeviceProvider {
public:
    ProxyTrackedDeviceProvider(vr::IServerTrackedDeviceProvider* pRealProvider) : m_pRealProvider(pRealProvider) {}
    ~ProxyTrackedDeviceProvider() = default;

    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        m_pProxyCtx = std::make_unique<ProxyDriverContext>(pDriverContext);
        return m_pRealProvider->Init(m_pProxyCtx.get());
    }

    void Cleanup() override {
        m_pRealProvider->Cleanup();
        m_pProxyCtx.reset();
        g_hmdSynthesizer.Reset();
    }

    const char* const* GetInterfaceVersions() override {
        return m_pRealProvider->GetInterfaceVersions();
    }

    void RunFrame() override {
        m_pRealProvider->RunFrame();
    }

    bool ShouldBlockStandbyMode() override {
        return m_pRealProvider->ShouldBlockStandbyMode();
    }

    void EnterStandby() override {
        m_pRealProvider->EnterStandby();
    }

    void LeaveStandby() override {
        m_pRealProvider->LeaveStandby();
    }

private:
    vr::IServerTrackedDeviceProvider* m_pRealProvider = nullptr;
    std::unique_ptr<ProxyDriverContext> m_pProxyCtx;
};

static std::unique_ptr<ProxyTrackedDeviceProvider> g_pProxyProvider;

static bool LoadOriginalDriver() {
    if (g_pfnOrigFactory) return true;

    wchar_t driverDir[MAX_PATH] = {};
    wchar_t origDllPath[MAX_PATH] = {};
    bool truncated = false;

    if (GetModuleFileNameW(g_hModule, driverDir, MAX_PATH)) {
        wchar_t* lastSlash = wcsrchr(driverDir, L'\\');
        if (lastSlash) {
            *lastSlash = L'\0';

            static constexpr wchar_t kOrigDriverSuffix[] = L"\\driver_pico_orig.dll";
            const size_t dirLen = wcslen(driverDir);
            const size_t suffixLen = wcslen(kOrigDriverSuffix);

            // Verify the full path (dir + suffix + null terminator) actually
            // fits in MAX_PATH before building it. wcsncpy_s/wcsncat_s with
            // _TRUNCATE would otherwise silently produce a shorter path than
            // intended, and LoadLibraryExW could then load an unrelated file
            // that happens to exist at the truncated path.
            if (dirLen + suffixLen < MAX_PATH) {
                wcsncpy_s(origDllPath, MAX_PATH, driverDir, _TRUNCATE);
                wcsncat_s(origDllPath, MAX_PATH, kOrigDriverSuffix, _TRUNCATE);
            } else {
                truncated = true;
            }
        }
    }

    if (truncated) {
        // The real driver directory path is known but doesn't fit alongside
        // the target filename: refuse to guess at a shortened path rather
        // than risk loading the wrong DLL.
        return false;
    }

    if (driverDir[0]) {
        SetDllDirectoryW(driverDir);
    }

    g_hOrigDriver = LoadLibraryExW(origDllPath[0] ? origDllPath : L"driver_pico_orig.dll", NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!g_hOrigDriver) return false;

    g_pfnOrigFactory = reinterpret_cast<HmdDriverFactoryFn>(GetProcAddress(g_hOrigDriver, "HmdDriverFactory"));
    return (g_pfnOrigFactory != nullptr);
}

BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID) {
    if (fdwReason == DLL_PROCESS_ATTACH) {
        g_hModule = hinstDLL;
        DisableThreadLibraryCalls(hinstDLL);
    }
    return TRUE;
}

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    std::call_once(g_loadOrigDriverOnce, []() {
        LoadOriginalDriver();
    });

    if (!g_pfnOrigFactory) {
        if (pReturnCode) *pReturnCode = vr::VRInitError_Init_FileNotFound;
        return nullptr;
    }

    void* pInterface = g_pfnOrigFactory(pInterfaceName, pReturnCode);
    if (!pInterface) return nullptr;

    if (std::strncmp(pInterfaceName, "IServerTrackedDeviceProvider", 28) == 0) {
        if (!g_pProxyProvider) {
            g_pProxyProvider = std::make_unique<ProxyTrackedDeviceProvider>(reinterpret_cast<vr::IServerTrackedDeviceProvider*>(pInterface));
        }
        return g_pProxyProvider.get();
    }

    return pInterface;
}