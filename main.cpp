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

    Vec3 sysInstantVel{};
    Vec3 sysPredVel{};
    Vec3 sysRawSpaceVel{};
    Vec3 waitGetRenderVel{};
    Vec3 waitGetGameVel{};

    Vec3 ctrlLeftOmega{};
    Vec3 ctrlRightOmega{};
    Vec3 ctrlLeftVel{};
    Vec3 ctrlRightVel{};
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

    double sampleDt = 0;
    Vec3 stepDelta{};
    double stepDistMm = 0;
    double stepSpeedMs = 0;
    bool isSnapback = false;
    double snapbackMagMm = 0;
    bool isDuplicateCall = false;
};

struct LogEntry {
    Sample base;
    std::string phaseName;
    double tilt = 0;
    Vec3 derivedWorldOmega{};
    Vec3 derivedWorldVel{};
    Vec3 derivedLocalVel{};
    double rotPredErr = 0;
    double linPredErrWithVel = 0;
    double linErrWithZero = 0;
    int angModel = 0;
    int linModel = 0;
    double linAccelJump = 0;
    std::string issues;

    double sampleDtMs = 0;
    double baseDtMs = 0;
    Vec3 stepDeltaMm{};
    double stepDistMm = 0;
    double stepSpeedMs = 0;
    bool isSnapback = false;
    double snapbackMagMm = 0;
    bool isDuplicateCall = false;
    double angWorldErrDegS = 0;
    double angLocalErrDegS = 0;
    double angErrMarginDegS = 0;
};

enum class TestState {
    Countdown = 0,
    Phase1_Upright,
    Phase2_Tilted,
    Phase3_Leaning,
    Finished
};

struct AtomicHud {
    std::atomic<TestState> state{TestState::Countdown};
    std::atomic<float> stateTimer{5.0f};
    std::atomic<float> tiltDeg{0.0f};
    std::atomic<float> speedDegS{0.0f};
    std::atomic<float> linSpeedMs{0.0f};
    std::atomic<float> linPredErrMm{0.0f};
    std::atomic<float> linZeroErrMm{0.0f};
    std::atomic<int> frameModel{0};
    std::atomic<int> linFrameModel{0};
    std::atomic<int> totalIssues{0};
    std::atomic<int> zeroVelEvents{0};
    std::atomic<int> avFrameEvents{0};
    std::atomic<int> zeroLinVelEvents{0};
    std::atomic<int> linOvershootEvents{0};
    std::atomic<bool> trackingOk{false};

    std::atomic<float> appFps{90.0f};
    std::atomic<float> warpAngleDeg{0.0f};
    std::atomic<float> reprojectRatio{0.0f};
    std::atomic<bool> blackEdgeRisk{false};

    char lastRunDir[256] = {};
    std::mutex dirMtx;
};

static std::atomic<bool> gRunning{true};
static std::atomic<bool> gRestartBenchmark{false};
static std::atomic<float> gTargetSimFps{90.0f};
static std::atomic<bool> gDynamicSweepActive{false};

static std::mutex gRenderPosesMutex;
static Vec3 gCompositorRenderOmega{};
static Vec3 gCompositorGameOmega{};
static Vec3 gCompositorRenderVel{};
static Vec3 gCompositorGameVel{};
static Vec3 gControllerLeftOmega{};
static Vec3 gControllerRightOmega{};
static Vec3 gControllerLeftVel{};
static Vec3 gControllerRightVel{};
static bool gControllerLeftValid = false;
static bool gControllerRightValid = false;

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

static void WriteCsvHeader(std::ofstream& f, bool isIncidentLog = false) {
    if (isIncidentLog) {
        f << "incident_id,context_tag,";
    }
    f << "time_s,phase,head_tilt_deg,"
      << "pos_x_m,pos_y_m,pos_z_m,"
      << "quat_w,quat_x,quat_y,quat_z,"
      << "reported_omega_x_rad_s,reported_omega_y_rad_s,reported_omega_z_rad_s,"
      << "reported_vel_x_m_s,reported_vel_y_m_s,reported_vel_z_m_s,"
      << "derived_world_omega_x_rad_s,derived_world_omega_y_rad_s,derived_world_omega_z_rad_s,"
      << "derived_world_vel_x_m_s,derived_world_vel_y_m_s,derived_world_vel_z_m_s,"
      << "derived_local_vel_x_m_s,derived_local_vel_y_m_s,derived_local_vel_z_m_s,"
      << "angular_prediction_error_deg,"
      << "linear_prediction_error_with_vel_mm,linear_error_with_zero_vel_mm,"
      << "preferred_angular_frame,preferred_linear_frame,"
      << "derived_linear_accel_jump_m_s2,issues,"
      << "sample_dt_ms,base_dt_ms,"
      << "step_dx_mm,step_dy_mm,step_dz_mm,step_dist_mm,step_speed_m_s,"
      << "is_snapback,snapback_mag_mm,is_duplicate_call,"
      << "ang_world_err_deg_s,ang_local_err_deg_s,ang_err_margin_deg_s\n";
}

static void LogEvent(std::ofstream& f, const LogEntry& e, int incidentId = -1, const std::string& contextTag = "") {
    if (incidentId >= 0) {
        f << incidentId << ',' << contextTag << ',';
    }
    f << std::fixed << std::setprecision(6)
      << e.base.t << ',' << e.phaseName << ',' << e.tilt << ','
      << e.base.p.x << ',' << e.base.p.y << ',' << e.base.p.z << ','
      << e.base.q.w << ',' << e.base.q.x << ',' << e.base.q.y << ',' << e.base.q.z << ','
      << e.base.omega.x << ',' << e.base.omega.y << ',' << e.base.omega.z << ','
      << e.base.velocity.x << ',' << e.base.velocity.y << ',' << e.base.velocity.z << ','
      << e.derivedWorldOmega.x << ',' << e.derivedWorldOmega.y << ',' << e.derivedWorldOmega.z << ','
      << e.derivedWorldVel.x << ',' << e.derivedWorldVel.y << ',' << e.derivedWorldVel.z << ','
      << e.derivedLocalVel.x << ',' << e.derivedLocalVel.y << ',' << e.derivedLocalVel.z << ','
      << e.rotPredErr << ',' << e.linPredErrWithVel << ',' << e.linErrWithZero << ','
      << ModelFrameName(e.angModel) << ',' << ModelFrameName(e.linModel) << ','
      << e.linAccelJump << ',' << (e.issues.empty() ? "NONE" : e.issues) << ','
      << e.sampleDtMs << ',' << e.baseDtMs << ','
      << e.stepDeltaMm.x << ',' << e.stepDeltaMm.y << ',' << e.stepDeltaMm.z << ','
      << e.stepDistMm << ',' << e.stepSpeedMs << ','
      << (e.isSnapback ? 1 : 0) << ',' << e.snapbackMagMm << ',' << (e.isDuplicateCall ? 1 : 0) << ','
      << e.angWorldErrDegS << ',' << e.angLocalErrDegS << ',' << e.angErrMarginDegS << '\n';
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
        if(FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) return false;

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
        const float rotSpeed = hud.speedDegS.load();
        const float linSpeed = hud.linSpeedMs.load();
        const float linPredErr = hud.linPredErrMm.load();
        const float linZeroErr = hud.linZeroErrMm.load();
        const int angModel = hud.frameModel.load();
        const int linModel = hud.linFrameModel.load();
        const int zeroVelEvents = hud.zeroVelEvents.load();
        const int avFrameEvents = hud.avFrameEvents.load();
        const int zeroLinVelEvents = hud.zeroLinVelEvents.load();
        const int linOvershootEvents = hud.linOvershootEvents.load();

        const float fps = hud.appFps.load();
        const float warpDeg = hud.warpAngleDeg.load();
        const float reprojectRatio = hud.reprojectRatio.load();
        const bool blackEdge = hud.blackEdgeRisk.load();

        const float cW = 1920.0f;
        const float cH = 1080.0f;

        AddCanvasRect(rawVerts, 80, 50, cW - 80, cH - 50, 0.10f, 0.15f, 0.25f, 0.95f);
        AddCanvasRect(rawVerts, 86, 56, cW - 86, cH - 56, 0.02f, 0.03f, 0.05f, 0.95f);

        AddCanvasRect(rawVerts, 86, 56, cW - 86, 150, 0.06f, 0.12f, 0.20f, 1.0f);
        AddCanvasText(rawVerts, "PICO 4 OPENVR TIMEWARP & PREDICTION BENCHMARK", 110, 80, 5.2f, 0.40f, 0.85f, 1.0f);

        std::ostringstream wstr;
        wstr << (gDynamicSweepActive.load() ? "SWEEP FPS: " : "SIMULATED FPS: ")
             << std::fixed << std::setprecision(0) << fps
             << " | REPROJECTED: " << std::setprecision(0) << (reprojectRatio * 100.0f) << "%"
             << " | WARP ANGLE: " << std::setprecision(2) << warpDeg << " DEG";
        AddCanvasText(rawVerts, wstr.str(), 110, 115, 3.8f, (fps < 85.0f) ? 1.0f : 0.4f, (fps < 85.0f) ? 0.6f : 1.0f, 0.4f);

        if(blackEdge) {
            AddCanvasRect(rawVerts, cW - 480, 75, cW - 110, 135, 0.80f, 0.15f, 0.15f, 0.95f);
            AddCanvasText(rawVerts, "BLACK EDGE OVERFLOW!", cW - 460, 95, 4.0f, 1.0f, 1.0f, 1.0f);
        }

        if(state == TestState::Countdown) {
            AddCanvasRect(rawVerts, 130, 180, cW - 130, 300, 0.35f, 0.30f, 0.05f, 0.85f);
            std::ostringstream cd; cd << "STARTING IN " << std::fixed << std::setprecision(1) << timer << "S...";
            AddCanvasText(rawVerts, cd.str(), 160, 215, 7.0f, 1.0f, 0.85f, 0.20f);
            AddCanvasText(rawVerts, "PREPARE FOR BENCHMARK [KEYS 1-5 IN CONSOLE TO TEST FRAMERATES]", 160, 360, 4.2f, 0.85f, 0.90f, 0.95f);
        }
        else if(state == TestState::Phase1_Upright || state == TestState::Phase2_Tilted || state == TestState::Phase3_Leaning) {
            std::ostringstream phHdr;
            if(state == TestState::Phase1_Upright) {
                AddCanvasRect(rawVerts, 130, 170, cW - 130, 260, 0.10f, 0.20f, 0.45f, 0.9f);
                phHdr << "PHASE 1/3: UPRIGHT ROTATION [" << std::fixed << std::setprecision(1) << timer << "S LEFT]";
                AddCanvasText(rawVerts, phHdr.str(), 160, 195, 5.5f, 1.0f, 1.0f, 1.0f);
                AddCanvasText(rawVerts, "ACTION: KEEP HEAD LEVEL AND SHAKE LEFT/RIGHT (YAW)", 160, 290, 4.5f, 0.5f, 1.0f, 0.8f);
                if(tilt > 15.0f) AddCanvasText(rawVerts, "WARNING: KEEP HEAD UPRIGHT (<15 DEG)!", 160, 330, 4.2f, 1.0f, 0.3f, 0.2f);
            } else if(state == TestState::Phase2_Tilted) {
                AddCanvasRect(rawVerts, 130, 170, cW - 130, 260, 0.40f, 0.20f, 0.10f, 0.9f);
                phHdr << "PHASE 2/3: TILTED ROTATION [" << std::fixed << std::setprecision(1) << timer << "S LEFT]";
                AddCanvasText(rawVerts, phHdr.str(), 160, 195, 5.5f, 1.0f, 1.0f, 1.0f);
                AddCanvasText(rawVerts, "ACTION: TILT EAR TO SHOULDER (35-45 DEG) AND SHAKE HEAD", 160, 290, 4.5f, 1.0f, 0.8f, 0.2f);
                if(tilt < 25.0f) AddCanvasText(rawVerts, ">> TILT MORE (>25 DEG) TO TEST ROTATION FRAME <<", 160, 330, 4.2f, 1.0f, 0.3f, 0.2f);
                else AddCanvasText(rawVerts, "STATUS: TARGET TILT REACHED! KEEP SHAKING!", 160, 330, 4.2f, 0.3f, 1.0f, 0.4f);
            } else {
                AddCanvasRect(rawVerts, 130, 170, cW - 130, 260, 0.15f, 0.40f, 0.20f, 0.9f);
                phHdr << "PHASE 3/3: TRANSLATIONAL LEAN & SURGE [" << std::fixed << std::setprecision(1) << timer << "S LEFT]";
                AddCanvasText(rawVerts, phHdr.str(), 160, 195, 5.5f, 1.0f, 1.0f, 1.0f);
                AddCanvasText(rawVerts, "ACTION: KEEP HEAD POINTING FORWARD, LEAN FORWARD/BACK (Z-AXIS)!", 160, 290, 4.2f, 0.3f, 1.0f, 0.4f);
            }

            AddCanvasRect(rawVerts, 140, 370, cW - 140, cH - 80, 0.05f, 0.08f, 0.12f, 0.95f);

            std::ostringstream s1; s1 << "ROTATION SPEED: " << std::fixed << std::setprecision(0) << rotSpeed << " DEG/S  |  FRAME: " << ModelFrameName(angModel);
            AddCanvasText(rawVerts, s1.str(), 170, 400, 4.2f, 0.8f, 0.85f, 0.9f);

            std::ostringstream s2; s2 << "TRANSLATION SPEED: " << std::fixed << std::setprecision(2) << linSpeed << " M/S  |  FRAME: " << ModelFrameName(linModel);
            AddCanvasText(rawVerts, s2.str(), 170, 460, 4.2f, 0.8f, 0.85f, 0.9f);

            std::ostringstream s3;
            s3 << "PREDICTION ERROR: WITH VEL=" << std::fixed << std::setprecision(1) << linPredErr << "mm  vs  ZERO VEL=" << linZeroErr << "mm";
            AddCanvasText(rawVerts, s3.str(), 170, 520, 4.2f, (linPredErr > linZeroErr * 1.3f) ? 1.0f : 0.4f, (linPredErr > linZeroErr * 1.3f) ? 0.3f : 1.0f, 0.4f);

            std::ostringstream estr;
            estr << "ISSUES: ZERO-ANG: " << zeroVelEvents << " | AV-FRAME: " << avFrameEvents
                 << " | ZERO-LIN: " << zeroLinVelEvents << " | LIN-OVERSHOOT: " << linOvershootEvents;
            AddCanvasText(rawVerts, estr.str(), 170, 600, 4.0f, (avFrameEvents > 0) ? 1.0f : 0.7f, (avFrameEvents > 0) ? 0.3f : 0.8f, 0.4f);

            AddCanvasText(rawVerts, "[KEYS: 1=90fps, 2=45fps, 3=60fps, 4=drops, 5=90<->60 sweep]", 170, 660, 3.8f, 0.5f, 0.7f, 0.9f);
        }
        else if(state == TestState::Finished) {
            AddCanvasRect(rawVerts, 130, 170, cW - 130, 260, 0.10f, 0.30f, 0.50f, 0.9f);
            AddCanvasText(rawVerts, "AUDIT COMPLETE — 6DOF PREDICTION REPORT", 160, 195, 5.5f, 1.0f, 1.0f, 1.0f);

            AddCanvasText(rawVerts, "FINDINGS & DIAGNOSTIC SUMMARY:", 160, 300, 4.5f, 0.8f, 0.9f, 1.0f);
            std::ostringstream r0; r0 << "- TILTED HEADSET-UP FAULTS (AV_FRAME_BUG): " << avFrameEvents;
            AddCanvasText(rawVerts, r0.str(), 160, 350, 4.2f, (avFrameEvents > 0) ? 1.0f : 0.3f, (avFrameEvents > 0) ? 0.3f : 1.0f, 0.4f);

            std::ostringstream r1; r1 << "- ZERO LINEAR VELOCITY EVENTS: " << zeroLinVelEvents;
            AddCanvasText(rawVerts, r1.str(), 160, 400, 4.0f, 0.85f, 0.9f, 0.95f);

            std::ostringstream r2; r2 << "- LINEAR EXTRAPOLATION OVERSHOOT EVENTS: " << linOvershootEvents;
            AddCanvasText(rawVerts, r2.str(), 160, 450, 4.0f, (linOvershootEvents > 50) ? 1.0f : 0.4f, (linOvershootEvents > 50) ? 0.4f : 1.0f, 0.4f);

            char logBuf[256];
            {
                std::lock_guard<std::mutex> lk(const_cast<AtomicHud&>(hud).dirMtx);
                std::strncpy(logBuf, hud.lastRunDir, sizeof(logBuf));
            }
            std::ostringstream r4; r4 << "- FULL LOGS IN: " << logBuf;
            AddCanvasText(rawVerts, r4.str(), 160, 510, 3.8f, 0.6f, 0.9f, 0.6f);
            AddCanvasText(rawVerts, "PRESS TRIGGER TO RUN BENCHMARK AGAIN", 240, cH - 150, 5.0f, 1.0f, 1.0f, 1.0f);
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
    if (msg == WM_KEYDOWN) {
        if (wParam == '1') { gDynamicSweepActive.store(false); gTargetSimFps.store(90.0f); std::cout << "\n[FPS Mode] 90 FPS (Clean)\n"; }
        if (wParam == '2') { gDynamicSweepActive.store(false); gTargetSimFps.store(45.0f); std::cout << "\n[FPS Mode] 45 FPS (Reprojecting 2x)\n"; }
        if (wParam == '3') { gDynamicSweepActive.store(false); gTargetSimFps.store(60.0f); std::cout << "\n[FPS Mode] 60 FPS (Sub-90 Jitter)\n"; }
        if (wParam == '4') { gDynamicSweepActive.store(false); gTargetSimFps.store(12.0f); std::cout << "\n[FPS Mode] 12 FPS (Simulating 84ms Stalls)\n"; }
        if (wParam == '5') { gDynamicSweepActive.store(true); std::cout << "\n[FPS Mode] Dynamic Sweep (90 <-> 60 FPS ramp, 1 FPS/sec)\n"; }
    }
    if (msg == WM_CLOSE || msg == WM_DESTROY) {
        gRunning.store(false);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static bool PollInputTrigger(vr::IVRSystem* vrSystem) {
    if((GetAsyncKeyState(VK_SPACE) & 0x8000) || (GetAsyncKeyState(VK_RETURN) & 0x8000)) return true;
    if(!vrSystem) return false;

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

int main(int argc, char** argv) {
    DisableProcessWindowsGhosting();

    ScopedTimerResolution timerRes;
    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    fs::path logRootDir = "logs";
    try { fs::create_directories(logRootDir); } catch(...) {}

    std::cout << "======================================================\n";
    std::cout << "  Pico 4 OpenVR 6DOF Prediction & Warping Benchmark   \n";
    std::cout << "======================================================\n";

    float initialFps = 90.0f;
    bool initialSweep = false;

    if(argc > 1) {
        for(int i = 1; i < argc; ++i) {
            if(std::strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
                std::string argVal = argv[++i];
                if(argVal == "sweep") {
                    initialSweep = true;
                } else {
                    initialFps = std::stof(argVal);
                }
            }
        }
    } else {
        std::cout << "\nChoose simulated rendering rate for the test:\n"
                  << "  [1] 90 FPS        (Standard clean rendering)\n"
                  << "  [2] 45 FPS        (Constant 2x Reprojection — War Thunder drop)\n"
                  << "  [3] 60 FPS        (Irregular dropped vsyncs)\n"
                  << "  [4] 12 FPS        (Simulate ~84ms stall episodes)\n"
                  << "  [5] Dynamic Sweep (90 <-> 60 FPS gradual wave, 1 FPS/sec)\n"
                  << "Select [1-5] (default 1): " << std::flush;

        std::string choice;
        std::getline(std::cin, choice);
        if(!choice.empty()) {
            if(choice[0] == '2') initialFps = 45.0f;
            else if(choice[0] == '3') initialFps = 60.0f;
            else if(choice[0] == '4') initialFps = 12.0f;
            else if(choice[0] == '5') initialSweep = true;
        }
    }

    gTargetSimFps.store(initialFps);
    gDynamicSweepActive.store(initialSweep);

    std::cout << "[Config] Mode: " << (initialSweep ? "Dynamic Sweep (90 <-> 60 FPS)" : (std::to_string((int)initialFps) + " FPS")) << "\n"
              << "[Tip] Switch modes live anytime using keys 1-5 on your PC keyboard!\n\n";

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

    std::cout << "\n[Ready] Full 6DOF audit active in VR.\n";

    AtomicHud hud;
    std::mutex historyMutex;
    std::deque<Sample> history;

    std::thread sampler([&]{
        std::unique_ptr<std::ofstream> csv;
        std::unique_ptr<std::ofstream> incidentCsv;
        std::unique_ptr<std::ofstream> summary;
        fs::path currentRunDir;

        TestState state = TestState::Countdown;
        double stateStartTime = 0.0;
        int zeroVelIssues = 0;
        int avFrameIssues = 0;
        int zeroLinVelIssues = 0;
        int linOvershootIssues = 0;
        int totalLoggedSamples = 0;
        int totalIncidentsDetected = 0;

        int highRotSpeedSamples = 0;
        int tiltEligibleSamples = 0;
        int highLinSpeedSamples = 0;

        uint64_t auditInstantLinZeroCount = 0;
        uint64_t auditPredLinZeroCount = 0;
        uint64_t auditRenderLinZeroCount = 0;

        // Phase Breakdown Telemetry
        struct PhaseStats {
            int totalSamples = 0;
            int highRotSamples = 0;
            int tiltEligibleSamples = 0;
            int highLinSamples = 0;
            int avFrameIssues = 0;
            int zeroLinVelIssues = 0;
            int linOvershootIssues = 0;
            int snapbackCount = 0;
            double totalSnapbackDistMm = 0.0;
            int duplicateCount = 0;

            // Granular Snapback Bins (< 1mm, 1-2mm, > 2mm)
            int snapSub1mm = 0;
            int snap1to2mm = 0;
            int snapOver2mm = 0;
        } p1Stats, p2Stats, p3Stats;

        std::deque<LogEntry> preIncidentBuffer;
        bool inIncident = false;
        int currentIncidentId = 0;
        int postRecoveryRemaining = 0;

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
                std::lock_guard<std::mutex> vrLock(gVrApiMutex);
                vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, posesInstant, vr::k_unMaxTrackedDeviceCount);
                vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0111f, posesPred, vr::k_unMaxTrackedDeviceCount);
                vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, posesRawSpace, vr::k_unMaxTrackedDeviceCount);
            }

            Vec3 renderOmega{}, gameOmega{}, renderVel{}, gameVel{};
            Vec3 ctrlLOmega{}, ctrlROmega{}, ctrlLVel{}, ctrlRVel{};
            bool ctrlLValid = false, ctrlRValid = false;
            {
                std::lock_guard<std::mutex> lk(gRenderPosesMutex);
                renderOmega = gCompositorRenderOmega;
                gameOmega = gCompositorGameOmega;
                renderVel = gCompositorRenderVel;
                gameVel = gCompositorGameVel;
                ctrlLOmega = gControllerLeftOmega;
                ctrlROmega = gControllerRightOmega;
                ctrlLVel = gControllerLeftVel;
                ctrlRVel = gControllerRightVel;
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

            if (havePrev) {
                s.sampleDt = s.t - prev.t;
                s.stepDelta = s.p - prev.p;
                s.stepDistMm = Length(s.stepDelta) * 1000.0;
                s.stepSpeedMs = (s.sampleDt > 1e-6) ? (s.stepDistMm / (s.sampleDt * 1000.0)) : 0.0;
                s.isDuplicateCall = (s.sampleDt < 0.0005 && s.stepDistMm < 0.01);

                // Snap-back detection: step displacement opposes reported motion velocity by > 0.5mm
                if (Length(s.velocity) > 0.08) {
                    const double velProj = Dot(s.stepDelta, Normalize(s.velocity));
                    if (velProj < -0.0005) {
                        s.isSnapback = true;
                        s.snapbackMagMm = std::abs(velProj) * 1000.0;
                    }
                }
            }

            s.audit.sysInstantOmega = s.omega;
            s.audit.sysPredOmega = ReportedAngular(posesPred[0]);
            s.audit.sysRawSpaceOmega = ReportedAngular(posesRawSpace[0]);
            s.audit.waitGetRenderOmega = renderOmega;
            s.audit.waitGetGameOmega = gameOmega;

            s.audit.sysInstantVel = s.velocity;
            s.audit.sysPredVel = ReportedLinear(posesPred[0]);
            s.audit.sysRawSpaceVel = ReportedLinear(posesRawSpace[0]);
            s.audit.waitGetRenderVel = renderVel;
            s.audit.waitGetGameVel = gameVel;

            s.audit.ctrlLeftOmega = ctrlLOmega;
            s.audit.ctrlRightOmega = ctrlROmega;
            s.audit.ctrlLeftVel = ctrlLVel;
            s.audit.ctrlRightVel = ctrlRVel;
            s.audit.ctrlLeftValid = ctrlLValid;
            s.audit.ctrlRightValid = ctrlRValid;

            const double curTilt = TiltFromQuat(s.q);
            hud.trackingOk.store(s.poseValid && s.result==vr::TrackingResult_Running_OK);
            hud.tiltDeg.store(static_cast<float>(curTilt));

            if(gRestartBenchmark.exchange(false)) {
                state = TestState::Countdown;
                stateStartTime = t;
                zeroVelIssues = 0;
                avFrameIssues = 0;
                zeroLinVelIssues = 0;
                linOvershootIssues = 0;
                totalLoggedSamples = 0;
                totalIncidentsDetected = 0;
                highRotSpeedSamples = 0;
                tiltEligibleSamples = 0;
                highLinSpeedSamples = 0;
                auditInstantLinZeroCount = 0;
                auditPredLinZeroCount = 0;
                auditRenderLinZeroCount = 0;
                p1Stats = {}; p2Stats = {}; p3Stats = {};
                inIncident = false;
                currentIncidentId = 0;
                postRecoveryRemaining = 0;
                preIncidentBuffer.clear();
                hud.totalIssues.store(0);
                hud.zeroVelEvents.store(0);
                hud.avFrameEvents.store(0);
                hud.zeroLinVelEvents.store(0);
                hud.linOvershootEvents.store(0);
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
                    WriteCsvHeader(*csv, false);

                    incidentCsv = std::make_unique<std::ofstream>(currentRunDir / "incident_transitions.csv");
                    WriteCsvHeader(*incidentCsv, true);

                    {
                        std::lock_guard<std::mutex> lk(hud.dirMtx);
                        std::strncpy(hud.lastRunDir, currentRunDir.string().c_str(), sizeof(hud.lastRunDir)-1);
                    }
                    std::cout << "[Benchmark] Phase 1: Upright Rotation Test started\n";
                }
            }
            else if(state == TestState::Phase1_Upright) {
                const double elapsed = t - stateStartTime;
                hud.stateTimer.store(static_cast<float>(std::max(0.0, 10.0 - elapsed)));
                if(elapsed >= 10.0) {
                    state = TestState::Phase2_Tilted;
                    stateStartTime = t;
                    std::cout << "[Benchmark] Phase 2: Tilted Rotation Test started\n";
                }
            }
            else if(state == TestState::Phase2_Tilted) {
                const double elapsed = t - stateStartTime;
                hud.stateTimer.store(static_cast<float>(std::max(0.0, 15.0 - elapsed)));
                if(elapsed >= 15.0) {
                    state = TestState::Phase3_Leaning;
                    stateStartTime = t;
                    std::cout << "[Benchmark] Phase 3: Translational Lean & Surge (Z-axis) started\n";
                }
            }
            else if(state == TestState::Phase3_Leaning) {
                const double elapsed = t - stateStartTime;
                hud.stateTimer.store(static_cast<float>(std::max(0.0, 15.0 - elapsed)));
                if(elapsed >= 15.0) {
                    state = TestState::Finished;
                    if(csv) csv->close();
                    if(incidentCsv) incidentCsv->close();
                    summary = std::make_unique<std::ofstream>(currentRunDir / "summary.txt");
                    if(summary) {
                        *summary << "======================================================================\n"
                                 << "PICO 4 OPENVR 6DOF VELOCITY, WARPING & PREDICTION REPORT\n"
                                 << "======================================================================\n"
                                 << "Tracking System:          " << trackingSystem << "\n"
                                 << "Model:                    " << model << " | Driver: " << driverVer << "\n"
                                 << "Simulated FPS Mode:       " << (gDynamicSweepActive.load() ? "Dynamic Sweep (90 <-> 60 FPS)" : (std::to_string((int)gTargetSimFps.load()) + " FPS")) << "\n\n"
                                 << "TOTAL SAMPLES LOGGED:     " << totalLoggedSamples << "\n"
                                 << "TOTAL INCIDENT EPISODES:  " << totalIncidentsDetected << " (Captured in incident_transitions.csv)\n\n"
                                 << "OVERALL ROTATIONAL AUDIT (of " << highRotSpeedSamples << " high-rot-speed samples):\n"
                                 << "- Zero Angular Velocity Events:           " << zeroVelIssues << "\n"
                                 << "- Tilted Headset-Up Faults (AV_FRAME_BUG):" << avFrameIssues << " / " << tiltEligibleSamples << " (" << (tiltEligibleSamples > 0 ? (avFrameIssues * 100 / tiltEligibleSamples) : 0) << "%)\n\n"
                                 << "OVERALL TRANSLATIONAL AUDIT (of " << highLinSpeedSamples << " high-linear-speed samples):\n"
                                 << "- Zero Linear Velocity Events:            " << zeroLinVelIssues << " / " << highLinSpeedSamples << " (" << (highLinSpeedSamples > 0 ? (zeroLinVelIssues * 100 / highLinSpeedSamples) : 0) << "%)\n"
                                 << "- Extrapolation Overshoot Events:         " << linOvershootIssues << " / " << highLinSpeedSamples << " (" << (highLinSpeedSamples > 0 ? (linOvershootIssues * 100 / highLinSpeedSamples) : 0) << "%)\n\n"
                                 << "API PATHWAY LINEAR VELOCITY AUDIT:\n"
                                 << "1. IVRSystem Instantaneous:               " << auditInstantLinZeroCount << " / " << highLinSpeedSamples << " zeroed\n"
                                 << "2. IVRSystem Predicted:                   " << auditPredLinZeroCount << " / " << highLinSpeedSamples << " zeroed\n"
                                 << "3. IVRCompositor WaitGetPoses:            " << auditRenderLinZeroCount << " / " << highLinSpeedSamples << " zeroed\n\n"
                                 << "======================================================================\n"
                                 << "PER-PHASE DETAILED BREAKDOWN\n"
                                 << "======================================================================\n"
                                 << "PHASE 1: UPRIGHT ROTATION\n"
                                 << "- Total Samples:       " << p1Stats.totalSamples << "\n"
                                 << "- High-Rot Samples:    " << p1Stats.highRotSamples << "\n"
                                 << "- Duplicate Pose Hits: " << p1Stats.duplicateCount << "\n\n"
                                 << "PHASE 2: TILTED ROTATION (>25 DEG TILT)\n"
                                 << "- Total Samples:       " << p2Stats.totalSamples << "\n"
                                 << "- Tilt Eligible:       " << p2Stats.tiltEligibleSamples << "\n"
                                 << "- AV_FRAME_BUG Count:  " << p2Stats.avFrameIssues << " (" << (p2Stats.tiltEligibleSamples > 0 ? (p2Stats.avFrameIssues * 100 / p2Stats.tiltEligibleSamples) : 0) << "%)\n\n"
                                 << "PHASE 3: TRANSLATIONAL LEANING & SURGE\n"
                                 << "- Total Samples:       " << p3Stats.totalSamples << "\n"
                                 << "- High-Linear Samples: " << p3Stats.highLinSamples << "\n"
                                 << "- Overshoot Errors:    " << p3Stats.linOvershootIssues << " (" << (p3Stats.highLinSamples > 0 ? (p3Stats.linOvershootIssues * 100 / p3Stats.highLinSamples) : 0) << "%)\n"
                                 << "- Snap-Back Events:    " << p3Stats.snapbackCount << " (Total Snapback: " << std::fixed << std::setprecision(2) << p3Stats.totalSnapbackDistMm << " mm)\n"
                                 << "  * < 1.0 mm (Sub-frame jitter):  " << p3Stats.snapSub1mm << "\n"
                                 << "  * 1.0 - 2.0 mm (Horizon error): " << p3Stats.snap1to2mm << "\n"
                                 << "  * > 2.0 mm (Severe snapback):   " << p3Stats.snapOver2mm << "\n"
                                 << "- Duplicate Pose Hits: " << p3Stats.duplicateCount << "\n"
                                 << "======================================================================\n";
                        summary->close();
                    }
                    std::cout << "[Benchmark] Complete! Report & incident transitions saved to: " << currentRunDir << "\n";
                }
            }

            hud.state.store(state);

            {
                std::lock_guard<std::mutex> lk(historyMutex);
                history.push_back(s);
                while(history.size() > 1500) history.pop_front();
            }

            Sample base{};
            bool haveBase = false;

            // Dynamic evaluation horizon: scales with simulated FPS instead of fixed 90Hz
            const float curTargetFps = gTargetSimFps.load();
            const double evalHorizon = (curTargetFps >= 10.0f && curTargetFps <= 150.0f)
                                       ? (1.0 / static_cast<double>(curTargetFps))
                                       : 0.0111;
            const double target = t - evalHorizon;

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

                // 1. Angular Ground Truth
                Quat dqLocal = QNormalize(QMul(QConj(base.q), s.q));
                if(dqLocal.w < 0.0) {
                    dqLocal.w = -dqLocal.w; dqLocal.x = -dqLocal.x; dqLocal.y = -dqLocal.y; dqLocal.z = -dqLocal.z;
                }
                const double halfW = std::clamp(dqLocal.w, 0.0, 1.0);
                const double angle = 2.0 * std::acos(halfW);
                Vec3 axis{dqLocal.x, dqLocal.y, dqLocal.z};
                const double sn = std::sin(angle * 0.5);
                if(sn > 1e-6) axis = axis * (1.0 / sn);
                else axis = {0, 0, 0};

                const Vec3 derivedLocalOmega = axis * (angle / dt);
                const Vec3 derivedWorldOmega = QRotate(base.q, derivedLocalOmega);

                const double rotSpeedDegS = Length(derivedWorldOmega) * kRadToDeg;
                hud.speedDegS.store(static_cast<float>(rotSpeedDegS));

                const double angWorldErr = Length(base.omega - derivedWorldOmega) * kRadToDeg;
                const double angLocalErr = Length(base.omega - derivedLocalOmega) * kRadToDeg;
                const int angModelMatch = (Length(base.omega)*kRadToDeg < 5.0) ? 0 :
                                          ((angLocalErr + 3.0 < angWorldErr) ? -1 :
                                          ((angWorldErr + 3.0 < angLocalErr) ? 1 : 0));
                hud.frameModel.store(angModelMatch);

                // 2. Linear Ground Truth & Frame
                const Vec3 derivedWorldVel = (s.p - base.p) * (1.0 / dt);
                const Vec3 derivedLocalVel = QRotate(QConj(base.q), derivedWorldVel);

                const double linSpeedMs = Length(derivedWorldVel);
                hud.linSpeedMs.store(static_cast<float>(linSpeedMs));

                const double linWorldErr = Length(base.velocity - derivedWorldVel) * 1000.0;
                const double linLocalErr = Length(base.velocity - derivedLocalVel) * 1000.0;
                const int linModelMatch = (linSpeedMs < 0.05) ? 0 :
                                          ((linLocalErr + 15.0 < linWorldErr) ? -1 :
                                          ((linWorldErr + 15.0 < linLocalErr) ? 1 : 0));
                hud.linFrameModel.store(linModelMatch);

                // 3. Linear Prediction Error vs Zero Extrapolation
                const Vec3 predPosWithVel = base.p + base.velocity * dt;
                const Vec3 predPosWithZero = base.p;
                const double linPredErrWithVel = Length(predPosWithVel - s.p) * 1000.0;
                const double linErrWithZero = Length(predPosWithZero - s.p) * 1000.0;

                hud.linPredErrMm.store(static_cast<float>(linPredErrWithVel));
                hud.linZeroErrMm.store(static_cast<float>(linErrWithZero));

                double linAccelJump = 0;
                if(havePrev) {
                    const double ddt = std::max(1e-5, s.t - prev.t);
                    linAccelJump = Length(s.velocity - prev.velocity) / ddt;
                }

                const Quat predWorld = IntegrateAngular(base.q, base.omega, dt, false);
                const double rotPredErr = RotationErrorDeg(predWorld, s.q);

                if(rotSpeedDegS > 15.0) highRotSpeedSamples++;
                if(tilt > 15.0 && rotSpeedDegS >= 10.0) tiltEligibleSamples++;

                if(linSpeedMs > 0.08) {
                    highLinSpeedSamples++;
                    if(Length(s.audit.sysInstantVel) < 0.01) auditInstantLinZeroCount++;
                    if(Length(s.audit.sysPredVel) < 0.01) auditPredLinZeroCount++;
                    if(Length(s.audit.waitGetRenderVel) < 0.01) auditRenderLinZeroCount++;
                }

                std::string flags;
                if(rotSpeedDegS > 15.0 && Length(base.omega)*kRadToDeg < 1.0) {
                    if(!flags.empty()) flags += '|'; flags += "ZERO_ANG_VEL";
                }
                if(angModelMatch == -1 && tilt > 15.0 && rotSpeedDegS >= 10.0) {
                    if(!flags.empty()) flags += '|'; flags += "AV_FRAME_BUG";
                }
                if(linSpeedMs > 0.08 && Length(base.velocity) < 0.01) {
                    if(!flags.empty()) flags += '|'; flags += "ZERO_LIN_VEL";
                }
                if(linModelMatch == -1 && linSpeedMs > 0.08) {
                    if(!flags.empty()) flags += '|'; flags += "LIN_FRAME_LOCAL_BUG";
                }
                if(linPredErrWithVel > linErrWithZero * 1.5 && linSpeedMs > 0.08) {
                    if(!flags.empty()) flags += '|'; flags += "LIN_OVERSHOOT_ERROR";
                }

                if(state == TestState::Phase1_Upright || state == TestState::Phase2_Tilted || state == TestState::Phase3_Leaning) {
                    totalLoggedSamples++;
                    bool isErrorSample = !flags.empty();

                    PhaseStats* curStats = (state == TestState::Phase1_Upright ? &p1Stats :
                                           (state == TestState::Phase2_Tilted ? &p2Stats : &p3Stats));
                    curStats->totalSamples++;
                    if(rotSpeedDegS > 15.0) curStats->highRotSamples++;
                    if(tilt > 15.0 && rotSpeedDegS >= 10.0) curStats->tiltEligibleSamples++;
                    if(linSpeedMs > 0.08) curStats->highLinSamples++;
                    if(s.isDuplicateCall) curStats->duplicateCount++;
                    if(s.isSnapback) {
                        curStats->snapbackCount++;
                        curStats->totalSnapbackDistMm += s.snapbackMagMm;
                        if(s.snapbackMagMm < 1.0) curStats->snapSub1mm++;
                        else if(s.snapbackMagMm < 2.0) curStats->snap1to2mm++;
                        else curStats->snapOver2mm++;
                    }

                    if(isErrorSample) {
                        hud.totalIssues.fetch_add(1);
                        if(flags.find("ZERO_ANG_VEL") != std::string::npos) {
                            zeroVelIssues++;
                            hud.zeroVelEvents.store(zeroVelIssues);
                        }
                        if(flags.find("AV_FRAME_BUG") != std::string::npos) {
                            avFrameIssues++;
                            curStats->avFrameIssues++;
                            hud.avFrameEvents.store(avFrameIssues);
                        }
                        if(flags.find("ZERO_LIN_VEL") != std::string::npos) {
                            zeroLinVelIssues++;
                            curStats->zeroLinVelIssues++;
                            hud.zeroLinVelEvents.store(zeroLinVelIssues);
                        }
                        if(flags.find("LIN_OVERSHOOT") != std::string::npos) {
                            linOvershootIssues++;
                            curStats->linOvershootIssues++;
                            hud.linOvershootEvents.store(linOvershootIssues);
                        }
                    }

                    std::string pName = (state == TestState::Phase1_Upright ? "UPRIGHT" :
                                        (state == TestState::Phase2_Tilted ? "TILTED" : "LEANING"));

                    LogEntry entry{base, pName, tilt, derivedWorldOmega, derivedWorldVel, derivedLocalVel,
                                   rotPredErr, linPredErrWithVel, linErrWithZero, angModelMatch, linModelMatch, linAccelJump, flags,
                                   s.sampleDt * 1000.0, dt * 1000.0, s.stepDelta * 1000.0, s.stepDistMm, s.stepSpeedMs,
                                   s.isSnapback, s.snapbackMagMm, s.isDuplicateCall,
                                   angWorldErr, angLocalErr, (angWorldErr - angLocalErr)};

                    if(csv && csv->is_open()) {
                        LogEvent(*csv, entry);
                    }

                    if(incidentCsv && incidentCsv->is_open()) {
                        if(isErrorSample) {
                            if(!inIncident) {
                                inIncident = true;
                                currentIncidentId++;
                                totalIncidentsDetected++;
                                for(const auto& pre : preIncidentBuffer) {
                                    LogEvent(*incidentCsv, pre, currentIncidentId, "PRE_GOOD");
                                }
                            }
                            LogEvent(*incidentCsv, entry, currentIncidentId, "INCIDENT");
                            postRecoveryRemaining = 2;
                        } else {
                            if(inIncident && postRecoveryRemaining > 0) {
                                LogEvent(*incidentCsv, entry, currentIncidentId, "POST_RECOVERY");
                                postRecoveryRemaining--;
                                if(postRecoveryRemaining == 0) {
                                    inIncident = false;
                                }
                            }
                            preIncidentBuffer.push_back(entry);
                            while(preIncidentBuffer.size() > 2) {
                                preIncidentBuffer.pop_front();
                            }
                        }
                    }
                }
            }

            prev = s; havePrev = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    bool lastTriggerState = false;
    auto lastRenderTime = std::chrono::steady_clock::now();
    const auto sweepStartTime = std::chrono::steady_clock::now();

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

        if(gDynamicSweepActive.load()) {
            const double sweepElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - sweepStartTime).count();
            const float cycle = std::fmod(static_cast<float>(sweepElapsed), 60.0f);
            const float sweepFps = (cycle < 30.0f) ? (90.0f - cycle) : (60.0f + (cycle - 30.0f));
            gTargetSimFps.store(sweepFps);
        }

        // Synchronous frame pacing before WaitGetPoses
        const float targetFps = gTargetSimFps.load();
        if(targetFps < 89.0f) {
            const double frameBudgetSec = 1.0 / static_cast<double>(targetFps);
            while(true) {
                const auto now = std::chrono::steady_clock::now();
                const double elapsed = std::chrono::duration<double>(now - lastRenderTime).count();
                if(elapsed >= frameBudgetSec) break;
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        }
        lastRenderTime = std::chrono::steady_clock::now();

        vr::Compositor_FrameTiming timing{};
        timing.m_nSize = sizeof(vr::Compositor_FrameTiming);
        if(compositor->GetFrameTiming(&timing, 0)) {
            hud.appFps.store(timing.m_flClientFrameIntervalMs > 0.001f ? (1000.0f / timing.m_flClientFrameIntervalMs) : targetFps);
            const float reprojected = (timing.m_nNumFramePresents > 1 || timing.m_nNumDroppedFrames > 0) ? 1.0f : 0.0f;
            hud.reprojectRatio.store(reprojected);

            const double currentSpeedRadS = Length(gCompositorRenderOmega);
            const double warpTime = std::max(0.0, (timing.m_nNumFramePresents > 1) ? 0.0222 : 0.0111);
            const float warpAngle = static_cast<float>(currentSpeedRadS * warpTime * kRadToDeg);
            hud.warpAngleDeg.store(warpAngle);

            hud.blackEdgeRisk.store(warpAngle > 3.5f);
        }

        vr::TrackedDevicePose_t renderPoses[vr::k_unMaxTrackedDeviceCount]{};
        vr::TrackedDevicePose_t gamePoses[vr::k_unMaxTrackedDeviceCount]{};
        vr::TrackedDeviceIndex_t leftIdx, rightIdx;
        {
            compositor->WaitGetPoses(renderPoses, vr::k_unMaxTrackedDeviceCount, gamePoses, vr::k_unMaxTrackedDeviceCount);

            std::lock_guard<std::mutex> vrLock(gVrApiMutex);
            leftIdx = vrSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);
            rightIdx = vrSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
        }

        {
            std::lock_guard<std::mutex> lk(gRenderPosesMutex);
            if(renderPoses[0].bPoseIsValid) {
                gCompositorRenderOmega = ReportedAngular(renderPoses[0]);
                gCompositorRenderVel = ReportedLinear(renderPoses[0]);
            }
            if(gamePoses[0].bPoseIsValid) {
                gCompositorGameOmega = ReportedAngular(gamePoses[0]);
                gCompositorGameVel = ReportedLinear(gamePoses[0]);
            }
            if(leftIdx < vr::k_unMaxTrackedDeviceCount && renderPoses[leftIdx].bPoseIsValid) {
                gControllerLeftOmega = ReportedAngular(renderPoses[leftIdx]);
                gControllerLeftVel = ReportedLinear(renderPoses[leftIdx]);
                gControllerLeftValid = true;
            } else {
                gControllerLeftValid = false;
            }
            if(rightIdx < vr::k_unMaxTrackedDeviceCount && renderPoses[rightIdx].bPoseIsValid) {
                gControllerRightOmega = ReportedAngular(renderPoses[rightIdx]);
                gControllerRightVel = ReportedLinear(renderPoses[rightIdx]);
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

        renderer.Render(hud);
    }

    if(sampler.joinable()) sampler.join();
    renderer.Shutdown();
    vr::VR_Shutdown();
    if(hWnd) DestroyWindow(hWnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    PromptExit(0);
    return 0;
}