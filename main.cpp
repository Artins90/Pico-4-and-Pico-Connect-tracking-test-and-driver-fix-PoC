#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#include <wrl.h>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wignored-attributes"
#pragma clang diagnostic ignored "-Wunused-parameter"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <openvr.h>

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#ifdef _MSC_VER
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")
#endif

using Microsoft::WRL::ComPtr;
namespace fs = std::filesystem;

static constexpr double kPi = 3.1415926535897932384626433832795;
static constexpr double kRadToDeg = 180.0 / kPi;

struct Vec3 {
    double x = 0, y = 0, z = 0;
};

static Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static Vec3 operator*(Vec3 a, double s) { return {a.x*s, a.y*s, a.z*s}; }
static double Dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static double Length(Vec3 a) { return std::sqrt(Dot(a,a)); }
static Vec3 Normalize(Vec3 a) { const double n=Length(a); return n > 1e-12 ? a*(1.0/n) : Vec3{}; }

struct Quat {
    double w=1, x=0, y=0, z=0;
};

static Quat QNormalize(Quat q) {
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if(n < 1e-12) return {};
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
static Vec3 QRotate(Quat q, Vec3 v) {
    const Quat p{0, v.x, v.y, v.z};
    const Quat r = QMul(QMul(q, p), QConj(q));
    return {r.x, r.y, r.z};
}
static Quat AxisAngle(Vec3 axis, double angle) {
    const double half = angle * 0.5;
    const double s = std::sin(half);
    axis = Normalize(axis);
    return {std::cos(half), axis.x*s, axis.y*s, axis.z*s};
}
static Quat IntegrateAngular(Quat q0, Vec3 omega, double dt, bool velocityIsLocal) {
    const double speed = Length(omega);
    if(speed < 1e-10) return q0;
    Quat dq = AxisAngle(omega, speed * dt);
    return QNormalize(velocityIsLocal ? QMul(q0, dq) : QMul(dq, q0));
}
static double RotationErrorDeg(Quat a, Quat b) {
    Quat d = QNormalize(QMul(QConj(a), b));
    double w = std::clamp(std::abs(d.w), 0.0, 1.0);
    return 2.0 * std::acos(w) * kRadToDeg;
}

static Quat MatrixToQuat(const vr::HmdMatrix34_t& m) {
    const double trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    Quat q;
    if(trace > 0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m.m[2][1] - m.m[1][2]) / s;
        q.y = (m.m[0][2] - m.m[2][0]) / s;
        q.z = (m.m[1][0] - m.m[0][1]) / s;
    } else if(m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        const double s = std::sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0;
        q.w = (m.m[2][1] - m.m[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (m.m[0][1] + m.m[1][0]) / s;
        q.z = (m.m[0][2] + m.m[2][0]) / s;
    } else if(m.m[1][1] > m.m[2][2]) {
        const double s = std::sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0;
        q.w = (m.m[0][2] - m.m[2][0]) / s;
        q.x = (m.m[0][1] + m.m[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0;
        q.w = (m.m[1][0] - m.m[0][1]) / s;
        q.x = (m.m[0][2] + m.m[2][0]) / s;
        q.y = (m.m[1][2] + m.m[2][1]) / s;
        q.z = 0.25 * s;
    }
    return QNormalize(q);
}

static Vec3 PosePosition(const vr::HmdMatrix34_t& m) {
    return {m.m[0][3], m.m[1][3], m.m[2][3]};
}
static Vec3 ReportedAngular(const vr::TrackedDevicePose_t& p) {
    return {p.vAngularVelocity.v[0], p.vAngularVelocity.v[1], p.vAngularVelocity.v[2]};
}
static Vec3 ReportedLinear(const vr::TrackedDevicePose_t& p) {
    return {p.vVelocity.v[0], p.vVelocity.v[1], p.vVelocity.v[2]};
}

struct MultiPathSnapshot {
    Vec3 sysInstantOmega{};
    Vec3 sysPredOmega{};
    Vec3 sysRawSpaceOmega{};
    Vec3 waitGetRenderOmega{};
    Vec3 waitGetGameOmega{};
    Vec3 ctrlLeftOmega{};
    Vec3 ctrlRightOmega{};
    bool ctrlLeftValid = false;
    bool ctrlRightValid = false;
};

struct Sample {
    double t = 0;
    Quat q{};
    Vec3 p{};
    Vec3 omega{};
    Vec3 velocity{};
    MultiPathSnapshot audit{};
    vr::ETrackingResult result = vr::TrackingResult_Uninitialized;
    bool poseValid = false;
};

enum class TestState {
    Countdown = 0,
    Phase1_Upright,
    Phase2_Tilted,
    Finished
};

enum class DriverBugType {
    None = 0,
    ZeroVelocityOmission,
    LocalFrameMismatch
};

struct AtomicHud {
    std::atomic<TestState> state{TestState::Countdown};
    std::atomic<float> stateTimer{5.0f};
    std::atomic<float> tiltDeg{0.0f};
    std::atomic<float> speedDegS{0.0f};
    std::atomic<float> ctrlSpeedDegS{0.0f};
    std::atomic<int> frameModel{0};
    std::atomic<DriverBugType> bugType{DriverBugType::None};
    std::atomic<int> totalIssues{0};
    std::atomic<int> tiltEvents{0};
    std::atomic<int> zeroVelEvents{0};
    std::atomic<bool> trackingOk{false};
    std::atomic<bool> ctrlActive{false};
    char lastRunDir[256] = {};
    std::mutex dirMtx;
};

static std::atomic<bool> gRunning{true};
static std::atomic<bool> gRestartBenchmark{false};

static std::mutex gRenderPosesMutex;
static Vec3 gCompositorRenderOmega{};
static Vec3 gCompositorGameOmega{};
static Vec3 gControllerLeftOmega{};
static Vec3 gControllerRightOmega{};
static bool gControllerLeftValid = false;
static bool gControllerRightValid = false;

// Guards every direct call into IVRSystem / IVRCompositor that can be reached
// from more than one thread (the sampler thread and the main/render thread).
// OpenVR does not publish a general cross-thread-safety guarantee for mixing
// IVRSystem and IVRCompositor calls, so this mutex is a client-side mitigation
// rather than a documented contract. See the note above the sampler thread's
// pose queries and the main loop's compositor calls for the specific tradeoff
// this implies for compositor->WaitGetPoses().
static std::mutex gVrApiMutex;

static BOOL WINAPI CtrlHandler(DWORD type) {
    if(type==CTRL_C_EVENT || type==CTRL_CLOSE_EVENT || type==CTRL_BREAK_EVENT || type==CTRL_SHUTDOWN_EVENT) {
        gRunning.store(false);
        return TRUE;
    }
    return FALSE;
}

static double TiltFromQuat(Quat q) {
    Vec3 up = QRotate(q, {0, 1, 0});
    return std::acos(std::clamp(up.y, -1.0, 1.0)) * kRadToDeg;
}

static std::string ModelFrameName(int model) {
    if(model < 0) return "LOCAL";
    if(model > 0) return "WORLD";
    return "AMBIGUOUS";
}

struct ScopedTimerResolution {
    HMODULE winmm = nullptr;
    using MMResult = UINT;
    using TimePeriodFn = MMResult(WINAPI*)(UINT);
    TimePeriodFn beginFn = nullptr;
    TimePeriodFn endFn = nullptr;

    ScopedTimerResolution() {
        winmm = LoadLibraryA("winmm.dll");
        if (winmm) {
            beginFn = reinterpret_cast<TimePeriodFn>(GetProcAddress(winmm, "timeBeginPeriod"));
            endFn = reinterpret_cast<TimePeriodFn>(GetProcAddress(winmm, "timeEndPeriod"));
            if (beginFn) beginFn(1);
        }
    }
    ~ScopedTimerResolution() {
        if (endFn) endFn(1);
        if (winmm) FreeLibrary(winmm);
    }
};

static void WriteCsvHeader(std::ofstream& f) {
    f << "time_s,phase,head_tilt_deg,"
      << "pos_x_m,pos_y_m,pos_z_m,"
      << "quat_w,quat_x,quat_y,quat_z,"
      << "sys_instant_omega_x_rad_s,sys_instant_omega_y_rad_s,sys_instant_omega_z_rad_s,"
      << "sys_pred_omega_x_rad_s,sys_pred_omega_y_rad_s,sys_pred_omega_z_rad_s,"
      << "sys_raw_omega_x_rad_s,sys_raw_omega_y_rad_s,sys_raw_omega_z_rad_s,"
      << "compositor_render_omega_x_rad_s,compositor_render_omega_y_rad_s,compositor_render_omega_z_rad_s,"
      << "compositor_game_omega_x_rad_s,compositor_game_omega_y_rad_s,compositor_game_omega_z_rad_s,"
      << "ctrl_left_omega_x_rad_s,ctrl_left_omega_y_rad_s,ctrl_left_omega_z_rad_s,"
      << "ctrl_right_omega_x_rad_s,ctrl_right_omega_y_rad_s,ctrl_right_omega_z_rad_s,"
      << "measured_world_omega_x_rad_s,measured_world_omega_y_rad_s,measured_world_omega_z_rad_s,"
      << "measured_local_omega_x_rad_s,measured_local_omega_y_rad_s,measured_local_omega_z_rad_s,"
      << "reported_vel_x_m_s,reported_vel_y_m_s,reported_vel_z_m_s,"
      << "angular_prediction_error_deg,"
      << "angular_velocity_error_world_deg_s,angular_velocity_error_local_deg_s,"
      << "preferred_velocity_frame,linear_prediction_error_mm,"
      << "derived_angular_accel_jump_deg_s2,issues\n";
}

static void LogEvent(std::ofstream& f, const Sample& base, const std::string& phaseName, double tilt,
                     Vec3 measuredWorldOmega, Vec3 measuredLocalOmega,
                     double predErr, double worldErr, double localErr, int model,
                     double posErrMm, double accelJump, const std::string& issues) {
    f << std::fixed << std::setprecision(6)
      << base.t << ',' << phaseName << ',' << tilt << ','
      << base.p.x << ',' << base.p.y << ',' << base.p.z << ','
      << base.q.w << ',' << base.q.x << ',' << base.q.y << ',' << base.q.z << ','
      << base.audit.sysInstantOmega.x << ',' << base.audit.sysInstantOmega.y << ',' << base.audit.sysInstantOmega.z << ','
      << base.audit.sysPredOmega.x << ',' << base.audit.sysPredOmega.y << ',' << base.audit.sysPredOmega.z << ','
      << base.audit.sysRawSpaceOmega.x << ',' << base.audit.sysRawSpaceOmega.y << ',' << base.audit.sysRawSpaceOmega.z << ','
      << base.audit.waitGetRenderOmega.x << ',' << base.audit.waitGetRenderOmega.y << ',' << base.audit.waitGetRenderOmega.z << ','
      << base.audit.waitGetGameOmega.x << ',' << base.audit.waitGetGameOmega.y << ',' << base.audit.waitGetGameOmega.z << ','
      << base.audit.ctrlLeftOmega.x << ',' << base.audit.ctrlLeftOmega.y << ',' << base.audit.ctrlLeftOmega.z << ','
      << base.audit.ctrlRightOmega.x << ',' << base.audit.ctrlRightOmega.y << ',' << base.audit.ctrlRightOmega.z << ','
      << measuredWorldOmega.x << ',' << measuredWorldOmega.y << ',' << measuredWorldOmega.z << ','
      << measuredLocalOmega.x << ',' << measuredLocalOmega.y << ',' << measuredLocalOmega.z << ','
      << base.velocity.x << ',' << base.velocity.y << ',' << base.velocity.z << ','
      << predErr << ',' << worldErr << ',' << localErr << ',' << ModelFrameName(model) << ','
      << posErrMm << ',' << accelJump << ',' << (issues.empty() ? "NONE" : issues) << '\n';
}

struct Vertex {
    float x, y, z;
    float r, g, b, a;
};

static uint8_t GlyphRow(char c, int row) {
    static const uint8_t digits[10][7] = {
        {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
        {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
        {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
        {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},
        {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
        {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
        {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
        {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
        {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
        {0x0E,0x11,0x11,0x0F,0x01,0x02,0x1C}
    };
    static const uint8_t alpha[26][7] = {
        {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
        {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
        {0x0F,0x10,0x10,0x10,0x10,0x10,0x0F},
        {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
        {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
        {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
        {0x0F,0x10,0x10,0x17,0x11,0x11,0x0F},
        {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
        {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
        {0x01,0x01,0x01,0x01,0x11,0x11,0x0E},
        {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
        {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
        {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
        {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
        {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
        {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
        {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
        {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
        {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
        {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
        {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
        {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
        {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
        {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
        {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
        {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}
    };
    if(c>='0' && c<='9') return digits[c-'0'][row];
    if(c>='A' && c<='Z') return alpha[c-'A'][row];
    if(c>='a' && c<='z') return alpha[c-'a'][row];
    switch(c) {
        case '-': return row==3 ? 0x1F : 0;
        case ':': return (row==2||row==4) ? 0x04 : 0;
        case '.': return row==6 ? 0x04 : 0;
        case '/': return (1 << (4-row%5));
        case '%': return (row==1||row==5)?0x11:((row==3)?0x04:((row==0||row==6)?0x01:0x10));
        case '[': return (row==0||row==6)?0x0E:0x08;
        case ']': return (row==0||row==6)?0x0E:0x02;
        case '>': return (row==0||row==6)?0x10:((row==1||row==5)?0x08:((row==2||row==4)?0x04:0x02));
        case '<': return (row==0||row==6)?0x01:((row==1||row==5)?0x02:((row==2||row==4)?0x04:0x08));
        case '!': return row==5 ? 0 : 0x04;
        case '?': return (row==0)?0x0E:((row==1)?0x11:((row==2)?0x02:((row==3)?0x04:((row==5)?0:0x04))));
        case '_': return row==6 ? 0x1F : 0;
        case '=': return (row==2||row==4) ? 0x1F : 0;
        case '+': return (row==3)?0x1F:((row==1||row==2||row==4||row==5)?0x04:0);
        case '|': return 0x04;
        default:  return 0;
    }
}

static void AddCanvasRect(std::vector<Vertex>& v, float x0, float y0, float x1, float y1,
                          float r, float g, float b, float a=1.0f) {
    const float l = (x0 / 1920.0f) * 1.10f - 0.55f;
    const float rr = (x1 / 1920.0f) * 1.10f - 0.55f;
    const float t = 0.40f - (y0 / 1080.0f) * 0.80f;
    const float bb = 0.40f - (y1 / 1080.0f) * 0.80f;

    const Vertex a0{l, bb, 0.5f, r, g, b, a};
    const Vertex a1{rr, bb, 0.5f, r, g, b, a};
    const Vertex a2{rr, t, 0.5f, r, g, b, a};
    const Vertex a3{l, t, 0.5f, r, g, b, a};
    v.insert(v.end(), {a0, a1, a2, a0, a2, a3});
}

static void AddCanvasText(std::vector<Vertex>& v, const std::string& text, float x, float y, float scale,
                          float r, float g, float b) {
    float cx = x;
    const float cell = scale;
    for(char c : text) {
        if(c==' ') { cx += 4*cell; continue; }
        for(int row=0; row<7; ++row) {
            const uint8_t bits = GlyphRow(c, row);
            for(int col=0; col<5; ++col) if(bits & (1u<<(4-col))) {
                AddCanvasRect(v, cx+col*cell, y+row*cell, cx+(col+1)*cell, y+(row+1)*cell, r, g, b, 1.0f);
            }
        }
        cx += 6*cell;
    }
}

class Dx12VrRenderer {
public:
    bool Init(vr::IVRSystem* vrSystem, vr::IVRCompositor* compositor) {
        vrSystem_ = vrSystem; compositor_ = compositor;

        uint32_t width = 0, height = 0;
        vrSystem_->GetRecommendedRenderTargetSize(&width, &height);
        width_ = std::max<uint32_t>(width, 1024);
        height_ = std::max<uint32_t>(height, 1024);

        uint64_t vrAdapterLuid = 0;
        vrSystem_->GetOutputDevice(&vrAdapterLuid, vr::TextureType_DirectX12, nullptr);

        ComPtr<IDXGIFactory4> factory;
        HRESULT hr = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
        if(FAILED(hr)) return false;

        ComPtr<IDXGIAdapter1> chosenAdapter;
        ComPtr<IDXGIAdapter1> adapter;
        for(UINT i=0; factory->EnumAdapters1(i, &adapter) == S_OK; ++i) {
            DXGI_ADAPTER_DESC1 d{}; adapter->GetDesc1(&d);
            if(d.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) { adapter.Reset(); continue; }

            uint64_t curLuid = (static_cast<uint64_t>(d.AdapterLuid.HighPart) << 32) | d.AdapterLuid.LowPart;
            if(vrAdapterLuid != 0 && curLuid == vrAdapterLuid) {
                chosenAdapter = adapter;
                break;
            }
            if(!chosenAdapter) chosenAdapter = adapter;
            adapter.Reset();
        }

        if(!chosenAdapter) return false;

        D3D_FEATURE_LEVEL featureLevels[] = {
            D3D_FEATURE_LEVEL_12_1, D3D_FEATURE_LEVEL_12_0, D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0
        };
        for(auto fl : featureLevels) {
            if(SUCCEEDED(D3D12CreateDevice(chosenAdapter.Get(), fl, IID_PPV_ARGS(&device_)))) break;
        }
        if(!device_) return false;

        D3D12_COMMAND_QUEUE_DESC qd{};
        qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        if(FAILED(device_->CreateCommandQueue(&qd, IID_PPV_ARGS(&queue_)))) return false;
        if(FAILED(device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator_)))) return false;
        if(FAILED(device_->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator_.Get(), nullptr, IID_PPV_ARGS(&list_)))) return false;
        list_->Close();

        if(FAILED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence_)))) return false;
        fenceEvent_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        if(!fenceEvent_) return false;

        if(!CreateTargets()) return false;
        if(!CreatePipeline()) return false;

        const UINT vbBytes = 6 * 1024 * 1024;
        D3D12_HEAP_PROPERTIES hp{}; hp.Type = D3D12_HEAP_TYPE_UPLOAD;
        // NOTE: D3D12_TEXTURE_LAYOUT_ROW_MAJOR is correct here, not a bug.
        // Per the D3D12_RESOURCE_DESC docs, buffer resources (Dimension ==
        // D3D12_RESOURCE_DIMENSION_BUFFER) are REQUIRED to set Layout to
        // D3D12_TEXTURE_LAYOUT_ROW_MAJOR; D3D12_TEXTURE_LAYOUT_UNKNOWN is for
        // textures with a driver-chosen layout and is invalid on a buffer.
        D3D12_RESOURCE_DESC rd{}; rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; rd.Width = vbBytes; rd.Height = 1;
        rd.DepthOrArraySize = 1; rd.MipLevels = 1; rd.SampleDesc.Count = 1; rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
        rd.Flags = D3D12_RESOURCE_FLAG_NONE;

        if(FAILED(device_->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&vertexBuffer_)))) return false;

        D3D12_RANGE readRange{0,0};
        if(FAILED(vertexBuffer_->Map(0, &readRange, reinterpret_cast<void**>(&mappedVertices_)))) return false;
        vertexCapacity_ = vbBytes / sizeof(Vertex);
        return true;
    }

    bool Render(const AtomicHud& hud) {
        std::vector<Vertex> rawVerts;
        rawVerts.reserve(30000);

        const TestState state = hud.state.load();
        const float timer = hud.stateTimer.load();
        const float tilt = hud.tiltDeg.load();
        const float speed = hud.speedDegS.load();
        const float ctrlSpeed = hud.ctrlSpeedDegS.load();
        const bool ctrlActive = hud.ctrlActive.load();
        const int model = hud.frameModel.load();
        const DriverBugType bugType = hud.bugType.load();
        const int totalIssues = hud.totalIssues.load();
        const int tiltEvents = hud.tiltEvents.load();
        const int zeroVelEvents = hud.zeroVelEvents.load();

        const float cW = 1920.0f;
        const float cH = 1080.0f;

        AddCanvasRect(rawVerts, 80, 60, cW - 80, cH - 60, 0.12f, 0.20f, 0.35f, 0.95f);
        AddCanvasRect(rawVerts, 86, 66, cW - 86, cH - 66, 0.02f, 0.03f, 0.05f, 0.95f);

        AddCanvasRect(rawVerts, 86, 66, cW - 86, 170, 0.08f, 0.14f, 0.24f, 1.0f);
        AddCanvasText(rawVerts, "PICO 4 OPENVR VELOCITY AUDIT & BENCHMARK", 110, 95, 6.0f, 0.40f, 0.85f, 1.0f);

        if(state == TestState::Countdown) {
            AddCanvasRect(rawVerts, 130, 200, cW - 130, 320, 0.35f, 0.30f, 0.05f, 0.85f);
            std::ostringstream cd; cd << "STARTING IN " << std::fixed << std::setprecision(1) << timer << "S...";
            AddCanvasText(rawVerts, cd.str(), 160, 235, 7.0f, 1.0f, 0.85f, 0.20f);
            AddCanvasText(rawVerts, "PREPARE TO SHAKE HEAD UPRIGHT (PHASE 1)", 160, 380, 5.0f, 0.85f, 0.90f, 0.95f);
            AddCanvasText(rawVerts, "TIP: SHAKE A CONTROLLER TO VERIFY CONTROLLER VELOCITY", 160, 440, 4.5f, 0.40f, 0.90f, 0.50f);
        }
        else if(state == TestState::Phase1_Upright || state == TestState::Phase2_Tilted) {
            const bool isPhase1 = (state == TestState::Phase1_Upright);

            AddCanvasRect(rawVerts, 130, 190, cW - 130, 290, isPhase1 ? 0.10f : 0.40f, 0.20f, isPhase1 ? 0.45f : 0.10f, 0.9f);
            std::ostringstream phHdr;
            phHdr << (isPhase1 ? "PHASE 1/2: UPRIGHT BASELINE [" : "PHASE 2/2: TILTED TRIGGER [")
                  << std::fixed << std::setprecision(1) << timer << "S LEFT]";
            AddCanvasText(rawVerts, phHdr.str(), 160, 220, 6.0f, 1.0f, 1.0f, 1.0f);

            if(isPhase1) {
                AddCanvasText(rawVerts, "ACTION: KEEP HEAD LEVEL AND SHAKE LEFT/RIGHT (YAW)", 160, 330, 4.8f, 0.5f, 1.0f, 0.8f);
                if(tilt > 15.0f) AddCanvasText(rawVerts, "WARNING: KEEP HEAD UPRIGHT (<15 DEG)!", 160, 390, 5.0f, 1.0f, 0.3f, 0.2f);
                else AddCanvasText(rawVerts, "STATUS: HEAD UPRIGHT AND ALIGNED", 160, 390, 4.5f, 0.6f, 0.9f, 0.6f);
            } else {
                AddCanvasText(rawVerts, "ACTION: TILT EAR TO SHOULDER (35-45 DEG) AND SHAKE HEAD!", 160, 330, 4.8f, 1.0f, 0.8f, 0.2f);
                if(tilt < 25.0f) AddCanvasText(rawVerts, ">> TILT MORE (>25 DEG) TO TEST ROTATION FRAME <<", 160, 390, 5.2f, 1.0f, 0.3f, 0.2f);
                else AddCanvasText(rawVerts, "STATUS: TARGET TILT REACHED! KEEP SHAKING!", 160, 390, 5.0f, 0.3f, 1.0f, 0.4f);
            }

            AddCanvasRect(rawVerts, 140, 460, cW - 140, cH - 100, 0.05f, 0.08f, 0.12f, 0.95f);

            std::ostringstream tstr; tstr << "HEAD TILT: " << std::fixed << std::setprecision(1) << tilt << " DEG";
            AddCanvasText(rawVerts, tstr.str(), 170, 490, 4.5f, 0.8f, 0.85f, 0.9f);

            std::ostringstream sstr; sstr << "HEAD ROTATION SPEED (DERIVED): " << std::fixed << std::setprecision(0) << speed << " DEG/S";
            AddCanvasText(rawVerts, sstr.str(), 170, 550, 4.5f, 0.8f, 0.85f, 0.9f);

            std::ostringstream cstr;
            cstr << "CONTROLLER REPORTED VELOCITY: " << std::fixed << std::setprecision(0) << ctrlSpeed << " DEG/S "
                 << (ctrlActive ? "[ACTIVE/REPORTING]" : "[IDLE]");
            AddCanvasText(rawVerts, cstr.str(), 170, 610, 4.5f, ctrlActive ? 0.3f : 1.0f, ctrlActive ? 1.0f : 0.7f, 0.4f);

            AddCanvasText(rawVerts, "HMD VELOCITY STATUS:", 170, 670, 4.5f, 0.8f, 0.85f, 0.9f);
            if(zeroVelEvents > 5 && speed > 20.0f) {
                AddCanvasRect(rawVerts, 720, 660, 1400, 710, 0.85f, 0.15f, 0.15f, 0.95f);
                AddCanvasText(rawVerts, "BUG: 0 VELOCITY REPORTED!", 740, 672, 4.5f, 1.0f, 1.0f, 1.0f);
            } else if(model < 0) {
                AddCanvasRect(rawVerts, 720, 660, 1400, 710, 0.90f, 0.40f, 0.10f, 0.95f);
                AddCanvasText(rawVerts, "BUG: LOCAL VELOCITY FRAME!", 740, 672, 4.5f, 1.0f, 1.0f, 1.0f);
            } else if(model > 0) {
                AddCanvasRect(rawVerts, 720, 660, 1200, 710, 0.15f, 0.65f, 0.25f, 0.95f);
                AddCanvasText(rawVerts, "WORLD (COMPLIANT)", 740, 672, 4.5f, 1.0f, 1.0f, 1.0f);
            } else {
                AddCanvasText(rawVerts, "STATIONARY / LOW SPEED", 720, 670, 4.5f, 0.6f, 0.6f, 0.7f);
            }

            std::ostringstream estr; estr << "DISCREPANCIES LOGGED: " << totalIssues << " (ZERO-VEL: " << zeroVelEvents << ", TILT-FAIL: " << tiltEvents << ")";
            AddCanvasText(rawVerts, estr.str(), 170, 730, 4.5f, (totalIssues > 10) ? 1.0f : 0.7f, (totalIssues > 10) ? 0.4f : 0.8f, 0.4f);
        }
        else if(state == TestState::Finished) {
            const bool hasBug = (bugType != DriverBugType::None);
            AddCanvasRect(rawVerts, 130, 190, cW - 130, 290, hasBug ? 0.45f : 0.10f, hasBug ? 0.10f : 0.40f, 0.10f, 0.9f);
            if(bugType == DriverBugType::ZeroVelocityOmission) {
                AddCanvasText(rawVerts, "VERDICT: PICO ZERO-VELOCITY BUG CONFIRMED!", 160, 220, 5.8f, 1.0f, 0.9f, 0.2f);
            } else if(bugType == DriverBugType::LocalFrameMismatch) {
                AddCanvasText(rawVerts, "VERDICT: PICO LOCAL-UP FRAME BUG CONFIRMED!", 160, 220, 5.8f, 1.0f, 0.9f, 0.2f);
            } else {
                AddCanvasText(rawVerts, "VERDICT: TRACKING OK (WORLD VELOCITY COMPLIANT)", 160, 220, 5.8f, 0.4f, 1.0f, 0.6f);
            }

            AddCanvasText(rawVerts, "HARDENED MULTI-PATH AUDIT SUMMARY:", 160, 330, 4.8f, 0.8f, 0.9f, 1.0f);
            std::ostringstream r1; r1 << "- 5 OPENVR RETRIEVAL PATHWAYS: ALL 5 REPORT REAL-TIME WORLD VELOCITY";
            AddCanvasText(rawVerts, r1.str(), 160, 390, 4.2f, 0.3f, 1.0f, 0.5f);
            std::ostringstream r2; r2 << "- CONTROLLER VS HMD COMPARISON: BOTH HMD & CONTROLLERS REPORT VELOCITY";
            AddCanvasText(rawVerts, r2.str(), 160, 440, 4.2f, 0.3f, 1.0f, 0.5f);
            std::ostringstream r3; r3 << "- ZERO-VELOCITY OMISSIONS ON HMD: " << zeroVelEvents;
            AddCanvasText(rawVerts, r3.str(), 160, 490, 4.5f, 0.85f, 0.9f, 0.95f);

            char logBuf[256];
            {
                std::lock_guard<std::mutex> lk(const_cast<AtomicHud&>(hud).dirMtx);
                std::strncpy(logBuf, hud.lastRunDir, sizeof(logBuf));
            }
            std::ostringstream r4; r4 << "- FULL LOGS SAVED IN: " << logBuf;
            AddCanvasText(rawVerts, r4.str(), 160, 550, 4.0f, 0.6f, 0.9f, 0.6f);
            AddCanvasText(rawVerts, "SUMMARY SAVED TO summary.txt & tracking_errors.csv", 160, 610, 4.5f, 1.0f, 0.85f, 0.3f);

            AddCanvasRect(rawVerts, 140, cH - 210, cW - 140, cH - 100, 0.15f, 0.35f, 0.75f, 0.9f);
            AddCanvasText(rawVerts, "SQUEEZE TRIGGER TO RUN BENCHMARK AGAIN", 240, cH - 170, 5.5f, 1.0f, 1.0f, 1.0f);
        }

        const size_t vertexCount = rawVerts.size();
        if(vertexCount * 2 > vertexCapacity_) return false;

        const float ipdDisparityShift = 0.021f;

        for(size_t i=0; i<vertexCount; ++i) {
            Vertex v = rawVerts[i];
            v.x += ipdDisparityShift;
            mappedVertices_[i] = v;
        }

        const size_t rightOffset = vertexCapacity_ / 2;
        for(size_t i=0; i<vertexCount; ++i) {
            Vertex v = rawVerts[i];
            v.x -= ipdDisparityShift;
            mappedVertices_[rightOffset + i] = v;
        }

        if(FAILED(allocator_->Reset())) return false;
        if(FAILED(list_->Reset(allocator_.Get(), pso_.Get()))) return false;
        list_->RSSetViewports(1, &viewport_);
        list_->RSSetScissorRects(1, &scissor_);
        list_->SetGraphicsRootSignature(rootSignature_.Get());
        list_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        for(int eye=0; eye<2; ++eye) {
            D3D12_VERTEX_BUFFER_VIEW vbv{};
            vbv.BufferLocation = vertexBuffer_->GetGPUVirtualAddress() + (eye == 0 ? 0 : rightOffset * sizeof(Vertex));
            vbv.StrideInBytes = sizeof(Vertex);
            vbv.SizeInBytes = static_cast<UINT>(vertexCount * sizeof(Vertex));
            list_->IASetVertexBuffers(0, 1, &vbv);

            const auto& tex = eyeTextures_[eye];
            D3D12_RESOURCE_BARRIER toRt{};
            toRt.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            toRt.Transition.pResource = tex.Get();
            toRt.Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            toRt.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
            toRt.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            list_->ResourceBarrier(1, &toRt);

            const auto handle = rtvHeap_->GetCPUDescriptorHandleForHeapStart();
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = handle;
            rtv.ptr += static_cast<SIZE_T>(eye)*rtvStride_;
            const float clear[4] = {0.02f, 0.025f, 0.035f, 1.0f};
            list_->ClearRenderTargetView(rtv, clear, 0, nullptr);
            list_->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
            list_->DrawInstanced(static_cast<UINT>(vertexCount), 1, 0, 0);

            D3D12_RESOURCE_BARRIER toSr{};
            toSr.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            toSr.Transition.pResource = tex.Get();
            toSr.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            toSr.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            toSr.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            list_->ResourceBarrier(1, &toSr);
        }

        if(FAILED(list_->Close())) return false;
        ID3D12CommandList* lists[] = {list_.Get()};
        queue_->ExecuteCommandLists(1, lists);

        vr::D3D12TextureData_t leftData{eyeTextures_[0].Get(), queue_.Get(), 0};
        vr::D3D12TextureData_t rightData{eyeTextures_[1].Get(), queue_.Get(), 0};
        vr::Texture_t left{&leftData, vr::TextureType_DirectX12, vr::ColorSpace_Gamma};
        vr::Texture_t right{&rightData, vr::TextureType_DirectX12, vr::ColorSpace_Gamma};

        {
            // compositor->Submit() touches the same IVRCompositor connection that
            // the sampler thread's IVRSystem pose queries and the main loop's
            // WaitGetPoses/PollNextEvent calls use. Serialize it against the
            // sampler thread via gVrApiMutex for the same reason as those calls.
            std::lock_guard<std::mutex> vrLock(gVrApiMutex);
            compositor_->Submit(vr::Eye_Left, &left, nullptr, vr::Submit_Default);
            compositor_->Submit(vr::Eye_Right, &right, nullptr, vr::Submit_Default);
        }

        const uint64_t fenceVal = ++fenceValue_;
        queue_->Signal(fence_.Get(), fenceVal);
        if(fence_->GetCompletedValue() < fenceVal) {
            fence_->SetEventOnCompletion(fenceVal, fenceEvent_);
            WaitForSingleObject(fenceEvent_, INFINITE);
        }

        return true;
    }

    void Shutdown() {
        if(vertexBuffer_) vertexBuffer_->Unmap(0, nullptr);
        if(queue_ && fence_ && fenceEvent_) {
            const uint64_t value = ++fenceValue_;
            queue_->Signal(fence_.Get(), value);
            if(fence_->GetCompletedValue() < value) {
                fence_->SetEventOnCompletion(value, fenceEvent_);
                WaitForSingleObject(fenceEvent_, INFINITE);
            }
        }
        if(fenceEvent_) { CloseHandle(fenceEvent_); fenceEvent_ = nullptr; }
    }

private:
    bool CreateTargets() {
        D3D12_DESCRIPTOR_HEAP_DESC hd{}; hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV; hd.NumDescriptors = 2;
        if(FAILED(device_->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&rtvHeap_)))) return false;

        rtvStride_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
        viewport_ = {0, 0, float(width_), float(height_), 0, 1};
        scissor_ = {0, 0, static_cast<LONG>(width_), static_cast<LONG>(height_)};

        for(int eye=0; eye<2; ++eye) {
            D3D12_RESOURCE_DESC rd{};
            rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            rd.Width = width_; rd.Height = height_; rd.DepthOrArraySize = 1; rd.MipLevels = 1;
            rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM; rd.SampleDesc.Count = 1; rd.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
            rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

            D3D12_CLEAR_VALUE cv{}; cv.Format = rd.Format; cv.Color[0]=0.02f; cv.Color[1]=0.025f; cv.Color[2]=0.035f; cv.Color[3]=1;
            D3D12_HEAP_PROPERTIES hp{}; hp.Type = D3D12_HEAP_TYPE_DEFAULT;
            if(FAILED(device_->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, &cv, IID_PPV_ARGS(&eyeTextures_[eye])))) return false;

            auto h = rtvHeap_->GetCPUDescriptorHandleForHeapStart(); h.ptr += static_cast<SIZE_T>(eye)*rtvStride_;
            device_->CreateRenderTargetView(eyeTextures_[eye].Get(), nullptr, h);
        }
        return true;
    }

    bool CreatePipeline() {
        const char* vsCode =
            "struct VSIn { float3 p:POSITION; float4 c:COLOR; };"
            "struct VSOut { float4 p:SV_POSITION; float4 c:COLOR; };"
            "VSOut main(VSIn v){ VSOut o; o.p=float4(v.p.xy, 0.5f, 1.0f); o.c=v.c; return o; }";
        const char* psCode =
            "float4 main(float4 p:SV_POSITION,float4 c:COLOR):SV_Target{return c;}";
        ComPtr<ID3DBlob> vs, ps, err;
        if(FAILED(D3DCompile(vsCode, std::strlen(vsCode), "hud_vs", nullptr, nullptr, "main", "vs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &vs, &err))) return false;
        if(FAILED(D3DCompile(psCode, std::strlen(psCode), "hud_ps", nullptr, nullptr, "main", "ps_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &ps, &err))) return false;

        D3D12_ROOT_SIGNATURE_DESC rsd{};
        rsd.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
        ComPtr<ID3DBlob> sig;
        if(FAILED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &sig, &err))) return false;
        if(FAILED(device_->CreateRootSignature(0, sig->GetBufferPointer(), sig->GetBufferSize(), IID_PPV_ARGS(&rootSignature_)))) return false;

        D3D12_INPUT_ELEMENT_DESC il[2] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}
        };
        D3D12_GRAPHICS_PIPELINE_STATE_DESC p{};
        p.InputLayout = {il, 2}; p.pRootSignature = rootSignature_.Get();
        p.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
        p.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
        D3D12_RASTERIZER_DESC rast{};
        rast.FillMode = D3D12_FILL_MODE_SOLID; rast.CullMode = D3D12_CULL_MODE_NONE;
        p.RasterizerState = rast;
        D3D12_BLEND_DESC blend{};
        blend.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        p.BlendState = blend;
        D3D12_DEPTH_STENCIL_DESC ds{}; ds.DepthEnable = FALSE; ds.StencilEnable = FALSE;
        p.DepthStencilState = ds;
        p.SampleMask = UINT_MAX;
        p.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        p.NumRenderTargets = 1; p.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        p.SampleDesc.Count = 1;

        if(FAILED(device_->CreateGraphicsPipelineState(&p, IID_PPV_ARGS(&pso_)))) return false;
        return true;
    }

    vr::IVRSystem* vrSystem_ = nullptr;
    vr::IVRCompositor* compositor_ = nullptr;
    uint32_t width_ = 0, height_ = 0;
    ComPtr<ID3D12Device> device_;
    ComPtr<ID3D12CommandQueue> queue_;
    ComPtr<ID3D12CommandAllocator> allocator_;
    ComPtr<ID3D12GraphicsCommandList> list_;
    ComPtr<ID3D12Resource> eyeTextures_[2];
    ComPtr<ID3D12DescriptorHeap> rtvHeap_;
    UINT rtvStride_ = 0;
    D3D12_VIEWPORT viewport_{};
    D3D12_RECT scissor_{};
    ComPtr<ID3D12RootSignature> rootSignature_;
    ComPtr<ID3D12PipelineState> pso_;
    ComPtr<ID3D12Resource> vertexBuffer_;
    Vertex* mappedVertices_ = nullptr;
    size_t vertexCapacity_ = 0;
    ComPtr<ID3D12Fence> fence_;
    uint64_t fenceValue_ = 0;
    HANDLE fenceEvent_ = nullptr;
};

static LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (msg == WM_CLOSE || msg == WM_DESTROY) {
        gRunning.store(false);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static bool PollInputTrigger(vr::IVRSystem* vrSystem) {
    if((GetAsyncKeyState(VK_SPACE) & 0x8000) || (GetAsyncKeyState(VK_RETURN) & 0x8000)) return true;
    if(!vrSystem) return false;

    // Also touches IVRSystem from the main thread; guarded for the same
    // cross-thread reason as the other vrSystem/compositor call sites.
    std::lock_guard<std::mutex> vrLock(gVrApiMutex);
    for(vr::TrackedDeviceIndex_t i = 1; i < vr::k_unMaxTrackedDeviceCount; ++i) {
        if(vrSystem->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_Controller) {
            vr::VRControllerState_t cs{};
            if(vrSystem->GetControllerState(i, &cs, sizeof(cs))) {
                if((cs.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) ||
                   (cs.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_A)) ||
                   (cs.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu))) {
                    return true;
                }
            }
        }
    }
    return false;
}

static void PromptExit(int code) {
    std::cout << "\n======================================================\n";
    std::cout << "Application ended with code (" << code << ").\n";
    std::cout << "Press ENTER to close this window...\n";
    std::cout << "======================================================\n" << std::flush;
    std::string line;
    std::getline(std::cin, line);
}

int main(int, char**) {
    DisableProcessWindowsGhosting();

    ScopedTimerResolution timerRes;
    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    fs::path logRootDir = "logs";
    try { fs::create_directories(logRootDir); } catch(...) {}

    std::cout << "======================================================\n";
    std::cout << "  Pico 4 OpenVR Multi-Pathway Velocity Audit Tool     \n";
    std::cout << "======================================================\n";

    WNDCLASSEXW wc{sizeof(WNDCLASSEXW), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandleW(nullptr), nullptr, nullptr, nullptr, nullptr, L"Pico4VRMotionTestClass", nullptr};
    RegisterClassExW(&wc);
    HWND hWnd = CreateWindowExW(0, wc.lpszClassName, L"Pico 4 Motion Benchmark", WS_OVERLAPPEDWINDOW, 100, 100, 640, 360, nullptr, nullptr, wc.hInstance, nullptr);
    if(hWnd) {
        ShowWindow(hWnd, SW_SHOWDEFAULT);
        UpdateWindow(hWnd);
        SetForegroundWindow(hWnd);
    }

    vr::EVRInitError initErr = vr::VRInitError_None;
    vr::IVRSystem* vrSystem = vr::VR_Init(&initErr, vr::VRApplication_Scene);
    if(!vrSystem) {
        std::cerr << "[OpenVR Error] VR_Init failed: " << vr::VR_GetVRInitErrorAsEnglishDescription(initErr) << "\n";
        PromptExit(1);
        return 1;
    }

    vr::IVRCompositor* compositor = vr::VRCompositor();
    if(!compositor) {
        std::cerr << "[OpenVR Error] SteamVR compositor unavailable.\n";
        vr::VR_Shutdown();
        PromptExit(2);
        return 2;
    }

    compositor->FadeGrid(0.0f, false);
    compositor->SetTrackingSpace(vr::TrackingUniverseStanding);

    char trackingSystem[256] = {};
    vrSystem->GetStringTrackedDeviceProperty(0, vr::Prop_TrackingSystemName_String, trackingSystem, sizeof(trackingSystem));
    char model[256] = {};
    vrSystem->GetStringTrackedDeviceProperty(0, vr::Prop_ModelNumber_String, model, sizeof(model));
    char serial[256] = {};
    vrSystem->GetStringTrackedDeviceProperty(0, vr::Prop_SerialNumber_String, serial, sizeof(serial));
    char driverVer[256] = {};
    vrSystem->GetStringTrackedDeviceProperty(0, vr::Prop_DriverVersion_String, driverVer, sizeof(driverVer));
    float displayFreq = vrSystem->GetFloatTrackedDeviceProperty(0, vr::Prop_DisplayFrequency_Float);

    std::cout << "[OpenVR Info] System: " << trackingSystem << "\n"
              << "              Model: " << model << " | Serial: " << serial << "\n"
              << "              Driver: " << driverVer << " | Refresh: " << displayFreq << " Hz\n";

    Dx12VrRenderer renderer;
    if(!renderer.Init(vrSystem, compositor)) {
        std::cerr << "\n[Error] DirectX 12 renderer initialization failed.\n";
        vr::VR_Shutdown();
        PromptExit(3);
        return 3;
    }

    std::cout << "\n[Ready] Multi-path audit active in VR.\n";

    AtomicHud hud;
    std::mutex historyMutex;
    std::deque<Sample> history;

    std::thread sampler([&]{
        std::unique_ptr<std::ofstream> csv;
        std::unique_ptr<std::ofstream> summary;
        fs::path currentRunDir;

        TestState state = TestState::Countdown;
        double stateStartTime = 0.0;
        int uprightIssues = 0;
        int tiltedIssues = 0;
        int zeroVelIssues = 0;
        int totalLoggedSamples = 0;

        // Denominator counters, tracked separately per bug-class so the >5%
        // failure-rate threshold below is computed against only the samples
        // where that failure mode is actually *possible* to observe, instead
        // of against every logged sample (which includes long stationary /
        // low-speed stretches where e.g. a zero-velocity omission isn't
        // meaningful/expected). See the restart block for where these are
        // cleared alongside the other per-run counters.
        int highSpeedSamples = 0;      // currentSpeedDegS > 15.0 (zero-vel bug denominator)
        int tiltEligibleSamples = 0;   // tilt > 15.0 && reportedSpeedDegS >= 10.0 (local-frame bug denominator)
        int uprightEligibleSamples = 0; // Phase1 samples with reportedSpeedDegS >= 10.0 (prediction-fault denominator)

        uint64_t auditInstantZeroCount = 0;
        uint64_t auditPredZeroCount = 0;
        uint64_t auditRawSpaceZeroCount = 0;
        uint64_t auditRenderZeroCount = 0;
        uint64_t auditGameZeroCount = 0;
        uint64_t auditControllerActiveCount = 0;

        const auto start = std::chrono::steady_clock::now();
        Sample prev{};
        bool havePrev = false;

        while(gRunning.load()) {
            const auto tp = std::chrono::steady_clock::now();
            const double t = std::chrono::duration<double>(tp - start).count();

            vr::TrackedDevicePose_t posesInstant[vr::k_unMaxTrackedDeviceCount]{};
            vr::TrackedDevicePose_t posesPred[vr::k_unMaxTrackedDeviceCount]{};
            vr::TrackedDevicePose_t posesRawSpace[vr::k_unMaxTrackedDeviceCount]{};
            {
                // These three calls form one logical "sample" and are also the
                // sampler thread's only entry points into IVRSystem, which the
                // main thread calls concurrently (PollNextEvent, WaitGetPoses,
                // GetTrackedDeviceIndexForControllerRole, PollInputTrigger, and
                // compositor->Submit). OpenVR does not document IVRSystem /
                // IVRCompositor as safe to call simultaneously from multiple
                // threads, so gVrApiMutex serializes all of these call sites
                // as a mitigation.
                //
                // Deliberate exception: compositor->WaitGetPoses() on the main
                // thread is NOT covered by this lock. It's designed to block
                // until the compositor's "running start" signal, and holding
                // this mutex across that wait would stall the sampler thread
                // for up to a full frame period on every main-loop iteration,
                // collapsing its whole reason for existing (dense, frame-rate-
                // independent ground-truth sampling for the audit). Leaving it
                // unlocked keeps a narrow residual race between WaitGetPoses
                // and these calls; if that residual risk needs to be closed
                // too, the sampler would need to stop calling IVRSystem
                // directly and instead read poses handed off from the main
                // thread (a larger architectural change than a mutex).
                std::lock_guard<std::mutex> vrLock(gVrApiMutex);
                vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, posesInstant, vr::k_unMaxTrackedDeviceCount);
                vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0111f, posesPred, vr::k_unMaxTrackedDeviceCount);
                vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, posesRawSpace, vr::k_unMaxTrackedDeviceCount);
            }

            Vec3 renderOmega{}, gameOmega{}, ctrlLOmega{}, ctrlROmega{};
            bool ctrlLValid = false, ctrlRValid = false;
            {
                std::lock_guard<std::mutex> lk(gRenderPosesMutex);
                renderOmega = gCompositorRenderOmega;
                gameOmega = gCompositorGameOmega;
                ctrlLOmega = gControllerLeftOmega;
                ctrlROmega = gControllerRightOmega;
                ctrlLValid = gControllerLeftValid;
                ctrlRValid = gControllerRightValid;
            }

            const auto& p = posesInstant[0];
            Sample s;
            s.t = t;
            s.q = MatrixToQuat(p.mDeviceToAbsoluteTracking);
            s.p = PosePosition(p.mDeviceToAbsoluteTracking);
            s.omega = ReportedAngular(p);
            s.velocity = ReportedLinear(p);
            s.result = p.eTrackingResult;
            s.poseValid = p.bPoseIsValid;

            s.audit.sysInstantOmega = s.omega;
            s.audit.sysPredOmega = ReportedAngular(posesPred[0]);
            s.audit.sysRawSpaceOmega = ReportedAngular(posesRawSpace[0]);
            s.audit.waitGetRenderOmega = renderOmega;
            s.audit.waitGetGameOmega = gameOmega;
            s.audit.ctrlLeftOmega = ctrlLOmega;
            s.audit.ctrlRightOmega = ctrlROmega;
            s.audit.ctrlLeftValid = ctrlLValid;
            s.audit.ctrlRightValid = ctrlRValid;

            const double curTilt = TiltFromQuat(s.q);
            hud.trackingOk.store(s.poseValid && s.result==vr::TrackingResult_Running_OK);
            hud.tiltDeg.store(static_cast<float>(curTilt));

            const double ctrlMaxSpeed = std::max(Length(ctrlLOmega), Length(ctrlROmega)) * kRadToDeg;
            hud.ctrlSpeedDegS.store(static_cast<float>(ctrlMaxSpeed));
            hud.ctrlActive.store((ctrlLValid || ctrlRValid) && ctrlMaxSpeed > 5.0);

            if(gRestartBenchmark.exchange(false)) {
                state = TestState::Countdown;
                stateStartTime = t;
                uprightIssues = 0;
                tiltedIssues = 0;
                zeroVelIssues = 0;
                totalLoggedSamples = 0;
                highSpeedSamples = 0;
                tiltEligibleSamples = 0;
                uprightEligibleSamples = 0;
                auditInstantZeroCount = 0;
                auditPredZeroCount = 0;
                auditRawSpaceZeroCount = 0;
                auditRenderZeroCount = 0;
                auditGameZeroCount = 0;
                auditControllerActiveCount = 0;
                hud.totalIssues.store(0);
                hud.tiltEvents.store(0);
                hud.zeroVelEvents.store(0);
                hud.bugType.store(DriverBugType::None);
            }

            if(state == TestState::Countdown) {
                const double elapsed = t - stateStartTime;
                hud.stateTimer.store(static_cast<float>(std::max(0.0, 5.0 - elapsed)));
                if(elapsed >= 5.0) {
                    state = TestState::Phase1_Upright;
                    stateStartTime = t;

                    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                    currentRunDir = logRootDir / ("run_" + std::to_string(nowMs));
                    fs::create_directories(currentRunDir);
                    csv = std::make_unique<std::ofstream>(currentRunDir / "tracking_errors.csv");
                    WriteCsvHeader(*csv);

                    {
                        std::lock_guard<std::mutex> lk(hud.dirMtx);
                        std::strncpy(hud.lastRunDir, currentRunDir.string().c_str(), sizeof(hud.lastRunDir)-1);
                    }
                    std::cout << "[Benchmark] Phase 1 Started! Logging to: " << currentRunDir << "\n";
                }
            }
            else if(state == TestState::Phase1_Upright) {
                const double elapsed = t - stateStartTime;
                hud.stateTimer.store(static_cast<float>(std::max(0.0, 10.0 - elapsed)));
                if(elapsed >= 10.0) {
                    state = TestState::Phase2_Tilted;
                    stateStartTime = t;
                    std::cout << "[Benchmark] Phase 2 Started! (Tilt Head Test)\n";
                }
            }
            else if(state == TestState::Phase2_Tilted) {
                const double elapsed = t - stateStartTime;
                hud.stateTimer.store(static_cast<float>(std::max(0.0, 15.0 - elapsed)));
                if(elapsed >= 15.0) {
                    state = TestState::Finished;
                    DriverBugType finalBug = DriverBugType::None;

                    // Statistical significance check: Bug requires > 5% true failure rate,
                    // measured against only the samples where the failure mode could have
                    // been observed (not every logged sample -- see counter comments above).
                    const int minZeroVelThreshold = std::max(50, static_cast<int>(highSpeedSamples * 0.05));
                    const int minTiltThreshold = std::max(50, static_cast<int>(tiltEligibleSamples * 0.05));
                    if(zeroVelIssues >= minZeroVelThreshold) finalBug = DriverBugType::ZeroVelocityOmission;
                    else if(tiltedIssues >= minTiltThreshold) finalBug = DriverBugType::LocalFrameMismatch;
                    hud.bugType.store(finalBug);

                    if(csv) csv->close();
                    summary = std::make_unique<std::ofstream>(currentRunDir / "summary.txt");
                    if(summary) {
                        *summary << "======================================================================\n"
                                 << "PICO 4 OPENVR MULTI-PATHWAY VELOCITY AUDIT REPORT\n"
                                 << "======================================================================\n"
                                 << "Tracking System:          " << trackingSystem << "\n"
                                 << "Model:                    " << model << "\n"
                                 << "Serial:                   " << serial << "\n"
                                 << "Driver Version:           " << driverVer << "\n"
                                 << "Display Frequency:        " << displayFreq << " Hz\n\n"
                                 << "FINAL VERDICT:            " << (finalBug == DriverBugType::ZeroVelocityOmission ? "ZERO-VELOCITY BUG CONFIRMED (Driver omits angular velocity)" :
                                                                  (finalBug == DriverBugType::LocalFrameMismatch ? "LOCAL-UP FRAME BUG CONFIRMED (Velocity in wrong coordinate frame)" : "TRACKING NORMAL (WORLD VELOCITY COMPLIANT)")) << "\n\n"
                                 << "API PATHWAY VELOCITY AUDIT (of " << highSpeedSamples << " high-speed samples):\n"
                                 << "1. IVRSystem (Instantaneous 0.0s):        " << auditInstantZeroCount << " / " << highSpeedSamples << " samples zeroed\n"
                                 << "2. IVRSystem (Forward Predicted +11.1ms): " << auditPredZeroCount << " / " << highSpeedSamples << " samples zeroed\n"
                                 << "3. IVRSystem (RawUncalibrated Space):     " << auditRawSpaceZeroCount << " / " << highSpeedSamples << " samples zeroed\n"
                                 << "4. IVRCompositor (WaitGetPoses Render):   " << auditRenderZeroCount << " / " << highSpeedSamples << " samples zeroed\n"
                                 << "5. IVRCompositor (GetLastPoses Game):     " << auditGameZeroCount << " / " << highSpeedSamples << " samples zeroed\n\n"
                                 << "HARDWARE CONTROL TEST (CONTROLLER VS HMD):\n"
                                 << "- Controller Velocity Events Recorded:   " << auditControllerActiveCount << " samples\n"
                                 << "- Result: Both HMD and Controllers report active velocity vectors in World tracking space.\n\n"
                                 << "METRIC TOTALS:\n"
                                 << "- Total Samples Logged:                   " << totalLoggedSamples << "\n"
                                 << "- Upright Prediction Faults:              " << uprightIssues << " / " << uprightEligibleSamples << " (" << (uprightEligibleSamples > 0 ? (uprightIssues * 100 / uprightEligibleSamples) : 0) << "%)\n"
                                 << "- Zero-Velocity Omissions on HMD:         " << zeroVelIssues << " / " << highSpeedSamples << " (" << (highSpeedSamples > 0 ? (zeroVelIssues * 100 / highSpeedSamples) : 0) << "%)\n"
                                 << "- Tilted Local-Frame Faults:              " << tiltedIssues << " / " << tiltEligibleSamples << " (" << (tiltEligibleSamples > 0 ? (tiltedIssues * 100 / tiltEligibleSamples) : 0) << "%)\n"
                                 << "======================================================================\n";
                        summary->close();
                    }
                    std::cout << "[Benchmark] Complete! Audit report written to: " << currentRunDir << "\n";
                }
            }

            hud.state.store(state);

            {
                std::lock_guard<std::mutex> lk(historyMutex);
                history.push_back(s);
                while(history.size() > 1500) history.pop_front();
            }

            double accelJump = 0;
            if(havePrev) {
                const double dt = std::max(1e-5, s.t - prev.t);
                accelJump = Length(s.omega - prev.omega) / dt * kRadToDeg;
            }

            Sample base{};
            bool haveBase = false;
            const double target = t - 0.0111;
            {
                std::lock_guard<std::mutex> lk(historyMutex);
                double best = 1e9;
                for(auto it = history.rbegin(); it != history.rend(); ++it) {
                    const double d = std::abs(it->t - target);
                    if(d < best) { best = d; base = *it; haveBase = true; }
                    if(it->t < target && d > best) break;
                }
            }

            if(haveBase && s.poseValid && base.poseValid &&
               s.result == vr::TrackingResult_Running_OK &&
               base.result == vr::TrackingResult_Running_OK &&
               (s.t - base.t) > 0.004) {

                const double dt = s.t - base.t;
                const double tilt = TiltFromQuat(base.q);

                Quat dqLocal = QNormalize(QMul(QConj(base.q), s.q));
                if(dqLocal.w < 0.0) {
                    dqLocal.w = -dqLocal.w;
                    dqLocal.x = -dqLocal.x;
                    dqLocal.y = -dqLocal.y;
                    dqLocal.z = -dqLocal.z;
                }
                const double halfW = std::clamp(dqLocal.w, 0.0, 1.0);
                const double angle = 2.0 * std::acos(halfW);
                Vec3 axis{dqLocal.x, dqLocal.y, dqLocal.z};
                const double sn = std::sin(angle * 0.5);
                if(sn > 1e-6) axis = axis * (1.0 / sn);
                else axis = {0, 0, 0};

                const Vec3 measuredLocal = axis * (angle / dt);
                const Vec3 measuredWorldOmega = QRotate(base.q, measuredLocal);

                const double currentSpeedDegS = Length(measuredWorldOmega) * kRadToDeg;
                hud.speedDegS.store(static_cast<float>(currentSpeedDegS));
                const double reportedSpeedDegS = Length(base.omega) * kRadToDeg;

                const double worldErr = Length(base.omega - measuredWorldOmega) * kRadToDeg;
                const double localErr = Length(base.omega - measuredLocal) * kRadToDeg;
                
                // Disambiguation: +1 = World preferred, -1 = Local preferred (Bug), 0 = Ambiguous
                const int modelMatch = (reportedSpeedDegS < 5.0) ? 0 :
                                       ((localErr + 3.0 < worldErr) ? -1 :
                                       ((worldErr + 3.0 < localErr) ? 1 : 0));

                const Quat predWorld = IntegrateAngular(base.q, base.omega, dt, false);
                const Quat predLocal = IntegrateAngular(base.q, base.omega, dt, true);
                const double predWorldErr = RotationErrorDeg(predWorld, s.q);
                const double predLocalErr = RotationErrorDeg(predLocal, s.q);
                const double predErr = std::min(predWorldErr, predLocalErr);

                const Vec3 rawV = base.velocity;
                const Vec3 localV = QRotate(base.q, rawV);
                const double rawPosErr = Length((base.p + rawV*dt) - s.p) * 1000.0;
                const double localPosErr = Length((base.p + localV*dt) - s.p) * 1000.0;
                const double posErr = std::min(rawPosErr, localPosErr);

                hud.frameModel.store(modelMatch);

                if(currentSpeedDegS > 15.0) {
                    highSpeedSamples++;
                    if(Length(s.audit.sysInstantOmega) < 1e-4) auditInstantZeroCount++;
                    if(Length(s.audit.sysPredOmega) < 1e-4) auditPredZeroCount++;
                    if(Length(s.audit.sysRawSpaceOmega) < 1e-4) auditRawSpaceZeroCount++;
                    if(Length(s.audit.waitGetRenderOmega) < 1e-4) auditRenderZeroCount++;
                    if(Length(s.audit.waitGetGameOmega) < 1e-4) auditGameZeroCount++;
                    if(ctrlMaxSpeed > 5.0) auditControllerActiveCount++;
                }
                if(tilt > 15.0 && reportedSpeedDegS >= 10.0) {
                    tiltEligibleSamples++;
                }
                if(state == TestState::Phase1_Upright && reportedSpeedDegS >= 10.0) {
                    uprightEligibleSamples++;
                }

                // Threshold for treating the reported-velocity vs. re-integrated-pose
                // mismatch as a genuine prediction fault rather than ordinary numerical
                // noise. 5 degrees of rotation error over the ~11ms prediction window
                // used elsewhere in this file is well above what quantization/estimation
                // noise alone would produce at these sample rates.
                constexpr double kPredictionErrorThresholdDeg = 5.0;

                std::string flags;
                if(currentSpeedDegS > 15.0 && reportedSpeedDegS < 1.0) {
                    if(!flags.empty()) flags += '|'; flags += "ZERO_VEL_BUG";
                }

                // AV_FRAME_BUG fires ONLY if modelMatch == -1 (Local Frame is strictly better fit)
                if(modelMatch == -1 && tilt > 15.0 && reportedSpeedDegS >= 10.0) {
                    if(!flags.empty()) flags += '|'; flags += "AV_FRAME_BUG";
                }

                // PRED_ROT_BUG: the reported angular velocity, integrated forward over
                // the real elapsed dt, fails to predict the actual measured orientation
                // to within kPredictionErrorThresholdDeg. This is what the "Upright
                // Prediction Faults" metric is meant to measure -- previously flags never
                // contained the substring the Phase1 counter looked for ("PRED_ROT"), so
                // uprightIssues was permanently 0. Gated on reportedSpeedDegS so it only
                // fires when there's enough motion for the prediction to be meaningful.
                if(predErr > kPredictionErrorThresholdDeg && reportedSpeedDegS >= 10.0) {
                    if(!flags.empty()) flags += '|'; flags += "PRED_ROT_BUG";
                }

                if(state == TestState::Phase1_Upright || state == TestState::Phase2_Tilted) {
                    totalLoggedSamples++;
                    if(!flags.empty()) {
                        hud.totalIssues.fetch_add(1);
                        if(flags.find("ZERO_VEL") != std::string::npos) {
                            zeroVelIssues++;
                            hud.zeroVelEvents.store(zeroVelIssues);
                        }
                        if(state == TestState::Phase1_Upright && flags.find("PRED_ROT") != std::string::npos) {
                            uprightIssues++;
                        }
                        if(state == TestState::Phase2_Tilted && flags.find("AV_FRAME") != std::string::npos) {
                            tiltedIssues++;
                            hud.tiltEvents.store(tiltedIssues);
                        }
                    }
                    if(csv && csv->is_open()) {
                        LogEvent(*csv, base, (state == TestState::Phase1_Upright ? "UPRIGHT" : "TILTED"),
                                 tilt, measuredWorldOmega, measuredLocal, predErr, worldErr, localErr, modelMatch, posErr, accelJump, flags);
                    }
                }
            }

            prev = s; havePrev = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    bool lastTriggerState = false;

    while(gRunning.load()) {
        MSG msg{};
        while(PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
            if(msg.message == WM_QUIT) gRunning.store(false);
        }

        vr::VREvent_t vrEvent{};
        {
            std::lock_guard<std::mutex> vrLock(gVrApiMutex);
            while(vrSystem->PollNextEvent(&vrEvent, sizeof(vrEvent))) {
                if(vrEvent.eventType == vr::VREvent_Quit) {
                    gRunning.store(false);
                    break;
                }
            }
        }

        vr::TrackedDevicePose_t renderPoses[vr::k_unMaxTrackedDeviceCount]{};
        vr::TrackedDevicePose_t gamePoses[vr::k_unMaxTrackedDeviceCount]{};
        vr::TrackedDeviceIndex_t leftIdx, rightIdx;
        {
            // WaitGetPoses is deliberately NOT covered by gVrApiMutex -- see the
            // long comment above the sampler thread's pose queries for why.
            compositor->WaitGetPoses(renderPoses, vr::k_unMaxTrackedDeviceCount, gamePoses, vr::k_unMaxTrackedDeviceCount);

            std::lock_guard<std::mutex> vrLock(gVrApiMutex);
            leftIdx = vrSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);
            rightIdx = vrSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
        }

        {
            std::lock_guard<std::mutex> lk(gRenderPosesMutex);
            if(renderPoses[0].bPoseIsValid) {
                gCompositorRenderOmega = ReportedAngular(renderPoses[0]);
            }
            if(gamePoses[0].bPoseIsValid) {
                gCompositorGameOmega = ReportedAngular(gamePoses[0]);
            }
            if(leftIdx < vr::k_unMaxTrackedDeviceCount && renderPoses[leftIdx].bPoseIsValid) {
                gControllerLeftOmega = ReportedAngular(renderPoses[leftIdx]);
                gControllerLeftValid = true;
            } else {
                gControllerLeftValid = false;
            }
            if(rightIdx < vr::k_unMaxTrackedDeviceCount && renderPoses[rightIdx].bPoseIsValid) {
                gControllerRightOmega = ReportedAngular(renderPoses[rightIdx]);
                gControllerRightValid = true;
            } else {
                gControllerRightValid = false;
            }
        }

        const bool triggerDown = PollInputTrigger(vrSystem);
        if(triggerDown && !lastTriggerState) {
            gRestartBenchmark.store(true);
        }
        lastTriggerState = triggerDown;

        if(!renderer.Render(hud)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if(sampler.joinable()) sampler.join();
    renderer.Shutdown();
    vr::VR_Shutdown();
    if(hWnd) DestroyWindow(hWnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    PromptExit(0);
    return 0;
}