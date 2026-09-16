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
#include <atomic>
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
static std::mutex g_factoryMutex;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    constexpr Vec3() noexcept = default;
    constexpr Vec3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const noexcept { return {x * s, y * s, z * s}; }
    double Dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
    double LengthSq() const noexcept { return x * x + y * y + z * z; }
    double Length() const noexcept { return std::sqrt(LengthSq()); }
};

struct Quat {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

class SRWLockGuard {
public:
    explicit SRWLockGuard(SRWLOCK& lock) noexcept : lock_(lock) {
        AcquireSRWLockExclusive(&lock_);
    }
    ~SRWLockGuard() noexcept {
        ReleaseSRWLockExclusive(&lock_);
    }
    SRWLockGuard(const SRWLockGuard&) = delete;
    SRWLockGuard& operator=(const SRWLockGuard&) = delete;
private:
    SRWLOCK& lock_;
};

class HmdVelocitySynthesizer {
public:
    HmdVelocitySynthesizer() noexcept {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        perfFreq_ = static_cast<double>(freq.QuadPart);
        invPerfFreq_ = (perfFreq_ > 0.0) ? (1.0 / perfFreq_) : 1.0;
    }

    void Reset() noexcept {
        SRWLockGuard lock(srwLock_);
        hasPrev_ = false;
        smoothOmega_ = {0.0, 0.0, 0.0};
        smoothVel_ = {0.0, 0.0, 0.0};
        cadenceFilteredVel_ = {0.0, 0.0, 0.0};
        prevTime_ = 0.0;
        lastStepTime_ = 0.0;
        stepIntervalEma_ = 0.0166;
    }

    void ProcessPose(vr::DriverPose_t& pose) noexcept {
        pose.vecAcceleration[0] = pose.vecAcceleration[1] = pose.vecAcceleration[2] = 0.0;
        pose.vecAngularAcceleration[0] = pose.vecAngularAcceleration[1] = pose.vecAngularAcceleration[2] = 0.0;

        if (!pose.poseIsValid) {
            Reset();
            pose.vecAngularVelocity[0] = pose.vecAngularVelocity[1] = pose.vecAngularVelocity[2] = 0.0;
            pose.vecVelocity[0] = pose.vecVelocity[1] = pose.vecVelocity[2] = 0.0;
            pose.poseTimeOffset = 0.0;
            return;
        }

        LARGE_INTEGER counter;
        QueryPerformanceCounter(&counter);
        const double t = static_cast<double>(counter.QuadPart) * invPerfFreq_;

        const double qw = pose.qRotation.w;
        const double qx = pose.qRotation.x;
        const double qy = pose.qRotation.y;
        const double qz = pose.qRotation.z;
        const double qNormSq = qw * qw + qx * qx + qy * qy + qz * qz;

        Quat curQ;
        if (std::abs(qNormSq - 1.0) > 1e-12 && qNormSq > 1e-12) {
            const double invNorm = 1.0 / std::sqrt(qNormSq);
            curQ = {qw * invNorm, qx * invNorm, qy * invNorm, qz * invNorm};
        } else if (qNormSq <= 1e-12) {
            curQ = {1.0, 0.0, 0.0, 0.0};
        } else {
            curQ = {qw, qx, qy, qz};
        }

        const Vec3 curRawP{pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]};

        SRWLockGuard lock(srwLock_);

        if (hasPrev_) {
            const double dt = t - prevTime_;

            if (dt > kMaxReliableDt || dt < -kMaxBackwardStep) {
                smoothOmega_ = {0.0, 0.0, 0.0};
                smoothVel_ = {0.0, 0.0, 0.0};
                cadenceFilteredVel_ = {0.0, 0.0, 0.0};
                prevQ_ = curQ;
                prevP_ = curRawP;
                prevTime_ = t;
                lastStepTime_ = t;
            } else if (dt > kMinValidDt) {
                const double invDt = 1.0 / dt;

                // 1. Angular Velocity (Driver World Space)
                double dqw_rot = curQ.w * prevQ_.w + curQ.x * prevQ_.x + curQ.y * prevQ_.y + curQ.z * prevQ_.z;
                double dqx_rot = curQ.x * prevQ_.w - curQ.w * prevQ_.x - curQ.y * prevQ_.z + curQ.z * prevQ_.y;
                double dqy_rot = curQ.y * prevQ_.w - curQ.w * prevQ_.y - curQ.z * prevQ_.x + curQ.x * prevQ_.z;
                double dqz_rot = curQ.z * prevQ_.w - curQ.w * prevQ_.z - curQ.x * prevQ_.y + curQ.y * prevQ_.x;

                const double dqNormSq = dqw_rot * dqw_rot + dqx_rot * dqx_rot + dqy_rot * dqy_rot + dqz_rot * dqz_rot;
                if (dqNormSq > 1e-12) {
                    const double invDqNorm = 1.0 / std::sqrt(dqNormSq);
                    dqw_rot *= invDqNorm; dqx_rot *= invDqNorm; dqy_rot *= invDqNorm; dqz_rot *= invDqNorm;
                }
                if (dqw_rot < 0.0) {
                    dqw_rot = -dqw_rot; dqx_rot = -dqx_rot; dqy_rot = -dqy_rot; dqz_rot = -dqz_rot;
                }

                const double halfW = std::clamp(dqw_rot, 0.0, 1.0);
                const double angle = 2.0 * std::acos(halfW);
                const double sn = std::sqrt(std::max(0.0, 1.0 - halfW * halfW));

                Vec3 rawOmega{0.0, 0.0, 0.0};
                if (sn > 1e-7) {
                    const double scale = (angle * invDt) / sn;
                    rawOmega = {dqx_rot * scale, dqy_rot * scale, dqz_rot * scale};
                }

                double rawOmegaSpeed = rawOmega.Length();
                if (rawOmegaSpeed > kMaxPhysicalOmegaRadS) {
                    rawOmega = rawOmega * (kMaxPhysicalOmegaRadS / rawOmegaSpeed);
                    rawOmegaSpeed = kMaxPhysicalOmegaRadS;
                }

                const double fcRot = kFcMinRot + kBetaRot * rawOmegaSpeed;
                const double alphaRot = 1.0 - std::exp(-2.0 * kPi * fcRot * dt);
                smoothOmega_ = rawOmega * alphaRot + smoothOmega_ * (1.0 - alphaRot);

                // 2. Linear Velocity (Jitter-Decoupled Step Cadence Estimator)
                const Vec3 deltaP = curRawP - prevP_;
                const double stepDist = deltaP.Length();

                if (stepDist > kStepThreshold) {
                    const double stepDt = t - lastStepTime_;
                    if (stepDt > kMinValidDt && stepDt < kMaxReliableDt) {
                        stepIntervalEma_ = stepIntervalEma_ * 0.95 + stepDt * 0.05;
                        stepIntervalEma_ = std::clamp(stepIntervalEma_, 0.010, 0.025);

                        Vec3 rawStepV = deltaP * (1.0 / stepDt);
                        double rawSpeed = rawStepV.Length();
                        if (rawSpeed > kMaxPhysicalVelMs) {
                            rawStepV = rawStepV * (kMaxPhysicalVelMs / rawSpeed);
                        }

                        // Reversal detection: update immediately without lag
                        if (rawStepV.Dot(cadenceFilteredVel_) <= 0.0 && rawSpeed > 0.03) {
                            cadenceFilteredVel_ = rawStepV;
                        } else {
                            const double alphaCadence = 1.0 - std::exp(-stepDt / 0.020);
                            cadenceFilteredVel_ = rawStepV * alphaCadence + cadenceFilteredVel_ * (1.0 - alphaCadence);
                        }
                    }
                    lastStepTime_ = t;
                }

                const double fcLin = kFcMinLin + kBetaLin * cadenceFilteredVel_.Length();
                const double alphaLin = 1.0 - std::exp(-2.0 * kPi * fcLin * dt);
                smoothVel_ = cadenceFilteredVel_ * alphaLin + smoothVel_ * (1.0 - alphaLin);

                // Decay gracefully if movement pauses (> 45ms without step)
                const double dtSinceStep = t - lastStepTime_;
                if (dtSinceStep > 0.045) {
                    const double decay = std::exp(-(dtSinceStep - 0.045) / 0.015);
                    smoothVel_ = smoothVel_ * decay;
                    cadenceFilteredVel_ = cadenceFilteredVel_ * decay;
                }

                prevQ_ = curQ;
                prevP_ = curRawP;
                prevTime_ = t;
            }
        } else {
            prevQ_ = curQ;
            prevP_ = curRawP;
            prevTime_ = t;
            lastStepTime_ = t;
            smoothOmega_ = {0.0, 0.0, 0.0};
            smoothVel_ = {0.0, 0.0, 0.0};
            cadenceFilteredVel_ = {0.0, 0.0, 0.0};
            hasPrev_ = true;
        }

        // Dynamically offset SteamVR by half the estimated hardware frame interval
        // to neutralize Pico's pre-baked exposure lead across any framerate.
        pose.poseTimeOffset = 0.5 * stepIntervalEma_;

        if (std::isfinite(smoothOmega_.x) && std::isfinite(smoothOmega_.y) && std::isfinite(smoothOmega_.z)) {
            pose.vecAngularVelocity[0] = smoothOmega_.x;
            pose.vecAngularVelocity[1] = smoothOmega_.y;
            pose.vecAngularVelocity[2] = smoothOmega_.z;
        }

        if (std::isfinite(smoothVel_.x) && std::isfinite(smoothVel_.y) && std::isfinite(smoothVel_.z)) {
            pose.vecVelocity[0] = smoothVel_.x;
            pose.vecVelocity[1] = smoothVel_.y;
            pose.vecVelocity[2] = smoothVel_.z;
        }
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kMinValidDt = 0.0002;
    static constexpr double kMaxReliableDt = 0.350;
    static constexpr double kMaxBackwardStep = 0.020;
    static constexpr double kStepThreshold = 0.00008; // 0.08mm motion threshold

    static constexpr double kFcMinLin = 5.0;
    static constexpr double kBetaLin = 10.0;
    static constexpr double kFcMinRot = 8.0;
    static constexpr double kBetaRot = 0.8;

    static constexpr double kMaxPhysicalVelMs = 10.0;
    static constexpr double kMaxPhysicalOmegaRadS = 25.0;

    SRWLOCK srwLock_ = SRWLOCK_INIT;
    double perfFreq_ = 1.0;
    double invPerfFreq_ = 1.0;

    Quat prevQ_{1.0, 0.0, 0.0, 0.0};
    Vec3 prevP_{0.0, 0.0, 0.0};
    Vec3 smoothOmega_{0.0, 0.0, 0.0};
    Vec3 smoothVel_{0.0, 0.0, 0.0};
    Vec3 cadenceFilteredVel_{0.0, 0.0, 0.0};

    double prevTime_ = 0.0;
    double lastStepTime_ = 0.0;
    double stepIntervalEma_ = 0.0166;
    bool hasPrev_ = false;
};

static HmdVelocitySynthesizer g_hmdSynthesizer;

class ProxyServerDriverHost final : public vr::IVRServerDriverHost {
public:
    explicit ProxyServerDriverHost(vr::IVRServerDriverHost* pRealHost) noexcept : m_pRealHost(pRealHost) {}
    ~ProxyServerDriverHost() = default;

    bool TrackedDeviceAdded(const char* pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver* pDriver) override {
        return m_pRealHost->TrackedDeviceAdded(pchDeviceSerialNumber, eDeviceClass, pDriver);
    }

    void TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t unPoseStructSize) override {
        if (unWhichDevice == vr::k_unTrackedDeviceIndex_Hmd) {
            vr::DriverPose_t fixedPose = newPose;
            g_hmdSynthesizer.ProcessPose(fixedPose);
            m_pRealHost->TrackedDevicePoseUpdated(unWhichDevice, fixedPose, unPoseStructSize);
        } else {
            m_pRealHost->TrackedDevicePoseUpdated(unWhichDevice, newPose, unPoseStructSize);
        }
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

class ProxyDriverContext final : public vr::IVRDriverContext {
public:
    explicit ProxyDriverContext(vr::IVRDriverContext* pRealCtx) noexcept : m_pRealCtx(pRealCtx) {}
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

class ProxyTrackedDeviceProvider final : public vr::IServerTrackedDeviceProvider {
public:
    explicit ProxyTrackedDeviceProvider(vr::IServerTrackedDeviceProvider* pRealProvider) noexcept : m_pRealProvider(pRealProvider) {}
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
            const size_t suffixLen = sizeof(kOrigDriverSuffix) / sizeof(wchar_t) - 1;

            if (dirLen + suffixLen < static_cast<size_t>(MAX_PATH)) {
                wcsncpy_s(origDllPath, MAX_PATH, driverDir, _TRUNCATE);
                wcsncat_s(origDllPath, MAX_PATH, kOrigDriverSuffix, _TRUNCATE);
            } else {
                truncated = true;
            }
        }
    }

    if (truncated) {
        return false;
    }

    g_hOrigDriver = LoadLibraryExW(origDllPath[0] ? origDllPath : L"driver_pico_orig.dll", NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!g_hOrigDriver) return false;

    g_pfnOrigFactory = reinterpret_cast<HmdDriverFactoryFn>(GetProcAddress(g_hOrigDriver, "HmdDriverFactory"));
    if (!g_pfnOrigFactory) {
        FreeLibrary(g_hOrigDriver);
        g_hOrigDriver = nullptr;
        return false;
    }

    return true;
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
        std::lock_guard<std::mutex> lock(g_factoryMutex);
        if (!g_pProxyProvider) {
            g_pProxyProvider = std::make_unique<ProxyTrackedDeviceProvider>(reinterpret_cast<vr::IServerTrackedDeviceProvider*>(pInterface));
        }
        return g_pProxyProvider.get();
    }

    return pInterface;
}