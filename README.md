# Pico 4 SteamVR Tracking Diagnostics & Velocity Fix (PoC)

This repository contains two components:
1. **Pico4VRMotionTest**: A standalone DirectX 12 / OpenVR diagnostic tool that audits OpenVR velocity reporting and measures head-tilt coordinate frame errors.
2. **driver_pico**: An OpenVR proxy wrapper .DLL for Pico Connect that adds missing world-space angular and linear velocity vectors in real time.

---

## Background & Problem Description

The OpenVR specification requires TrackedDevicePose_t.vAngularVelocity and vVelocity to be provided in the **World tracking space** (radians/second and meters/second).

On Pico 4 (not Pico 4 Ultra) the drivers handle velocity data inconsistently:
* **Pico Connect:** Omits velocity reporting entirely, passing vAngularVelocity = (0, 0, 0) and vVelocity = (0, 0, 0) across all OpenVR pathways.

### Consequences of These Implementations
1. **SteamVR Motion Prediction:** When the head is tilted (20°–60°) and rotated, missing or local-frame velocity causes SteamVR's forward extrapolation to fail.
2. **Full-Body Tracking (FBT) & Space Calibration:** Users on the Pico Discord reported bugs with systems that reference the HMD pose (e.g., Fluxpose, can't verify, I don't own extra trackers) 

---

## 1. How the Diagnostic Test Works (Pico4VRMotionTest)

The diagnostic tool runs directly inside SteamVR via DirectX 12 and renders an in-headset HUD placed at 1.5 meters depth.

### Test Procedure
* **Phase 1 (Upright Baseline, 10s):** Measures rotational velocity while the head remains upright (Tilt < 15°).
* **Phase 2 (Tilted Bug Trigger, 15s):** Measures rotational velocity while the head is tilted (> 25°).

### Analysis Performed
* **Ground-Truth Derivation:** Computes physical world angular velocity and local angular velocity directly from quaternion orientation differentials (dq / dt).
* **Model Fitting:** Compares the driver's reported vector against both frames to detect frame mismatches.
* **Multi-Pathway OpenVR Audit:** Queries all 5 OpenVR pose retrieval pathways simultaneously:
  1. IVRSystem::GetDeviceToAbsoluteTrackingPose (Instantaneous 0.0s)
  2. IVRSystem::GetDeviceToAbsoluteTrackingPose (Forward-predicted +11.1ms)
  3. IVRSystem::GetDeviceToAbsoluteTrackingPose (RawUncalibrated space)
  4. IVRCompositor::WaitGetPoses (Render pose)
  5. IVRCompositor::GetLastPoses (Game pose)
* **Hardware Control Test:** Polls active VR controllers as a baseline to verify whether OpenVR velocity reporting is functional in the client application.

### Output Files
Results are written to logs/run_<timestamp>/:
* **summary.txt**: Summary report with pathway audit and final verdict.
* **tracking_errors.csv**: Sample-by-sample telemetry with timestamps, quaternions, raw reported vectors, derived vectors, and error metrics.

---

## 2. How the Proxy Driver Fix Works (driver_pico.dll)

The proxy driver is a drop-in wrapper placed in Pico Connect's OpenVR driver directory.
[ SteamVR / OpenVR ]
▼ 
(Calls IVRServerDriverHost)
[ driver_pico.dll (Proxy Wrapper) ]
──> Intercepts: TrackedDevicePoseUpdated()
──> Computes: w_world = axis(q_t * q_{t-1}^-1) / dt
──> Populates: DriverPose_t.vecAngularVelocity & vecVelocity
▼ (Forwards original hardware communication)
[ driver_pico_orig.dll (Official Pico Driver) ]
code
Code
### Operation
1. The proxy forwards HmdDriverFactory and all initialization routines to driver_pico_orig.dll.
2. It wraps IVRServerDriverHost::TrackedDevicePoseUpdated without modifying the underlying HMD device object, preserving native DirectMode and OpenXR swapchains.
3. On every unique tracking update, it computes world-space angular velocity and linear velocity using high-precision performance counters (QPC) and applies an exponential filter (alpha = 0.80).
4. The synthesized vectors are populated into DriverPose_t.vecAngularVelocity and DriverPose_t.vecVelocity before the pose reaches SteamVR.

---

## Building from Source

### Prerequisites (in GitHub Codespaces)
The repository includes the LLVM-MinGW toolchain configuration.

### Build Steps
```bash
# 1. Configure the build environment
export LLVM_MINGW_ROOT="/workspaces/codespaces-blank/.tools/llvm-mingw"

# 2. Build both targets
ninja -C build-win64
Build outputs in build-win64/:
Pico4VRMotionTest.exe
driver_pico.dll


Driver fix Installation & Removal

Installation
Close SteamVR and Pico Connect (Stop the Pico Streaming Service from the Windows services, use task manager to make sure everything VR related is closed).
Navigate to Pico Connect\openvr_driver\bin\win64\ (your local Pico Connect installation path).

IMPORTANT!!: Rename driver_pico.dll to driver_pico_orig.dll

Copy the compiled driver_pico.dll into that folder.
Start Pico Connect and SteamVR.

Verification
Run Pico4VRMotionTest.exe inside VR. The audit should report:
Zero-Velocity Omissions on HMD: 0
FINAL VERDICT: TRACKING NORMAL (WORLD VELOCITY COMPLIANT)


Removal
Close SteamVR and Pico Connect (Stop the Pico Streaming Service from the Windows services, use task manager to make sure everything VR related is closed).
Delete driver_pico.dll.
Rename driver_pico_orig.dll back to driver_pico.dll