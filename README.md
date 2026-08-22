**DISCLAIMER:** This software is an experimental proof-of-concept provided "AS IS", without warranty of any kind, express or implied. Use at your own risk. The author assumes no responsibility or liability for any system instability, software conflicts, loss of data, or hardware issues resulting from the use of these tools or driver modifications. This project is independent and is not affiliated with, endorsed by, or associated with Pico Immersive Pte. Ltd., ByteDance, Valve Corporation, or SteamVR.

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
```bash
[ SteamVR / OpenVR (vrserver) ]
        │
        │ calls HmdDriverFactory / IServerTrackedDeviceProvider
        ▼
[ driver_pico.dll (Proxy Wrapper) ]
        │
        │ delegates provider/driver calls
        ▼
[ driver_pico_orig.dll (Official Pico Driver) ]
        │
        │ handles real hardware
        │ calls IVRServerDriverHost callbacks
        ▼
[ ProxyServerDriverHost inside driver_pico.dll ]
        │
        │ TrackedDevicePoseUpdated(unWhichDevice, newPose)
        │   if unWhichDevice == k_unTrackedDeviceIndex_Hmd:
        │       fixedPose = ProcessPose(newPose)
        │
        │ --- Velocity Synthesis (ProcessPose) ---
        │
        │  0. pose.poseIsValid?
        │        ├─ NO  → ω = v = 0
        │        │        hasPrev_ = false
        │        │        return (unchanged pose, no write)
        │        └─ YES → continue
        │
        │  1. Store previous pose + time:
        │        q_prev, p_prev, t_prev
        │
        │  2. Get current pose + time (QPC):
        │        q_t, p_t, t
        │
        │  3. dt = t - t_prev
        │        ├─ dt > 50ms   → ω = v = 0  (stale-gap reset)
        │        ├─ dt < 3ms    → keep previous ω, v  (skip sample)
        │        └─ 3ms..50ms   → continue to step 4
        │
        │  4. Compute relative rotation:
        │        dq = normalize( q_t * q_prev^-1 )
        │
        │  5. Extract rotation angle:
        │        θ = 2 * acos( clamp(dq.w, 0, 1) )
        │
        │  6. Extract rotation axis:
        │        axis = (dq.x, dq.y, dq.z) / sin(θ/2)
        │
        │  7. Compute angular velocity:
        │        ω_world = axis * (θ / dt)
        │
        │  8. Compute linear velocity:
        │        v_world = (p_t - p_prev) / dt
        │
        │  9. Smoothing (dt-normalized):
        │        α = 1 - exp(-dt / τ)          [τ ≈ 6.9ms]
        │        ω = α * ω_new + (1 - α) * ω_old
        │        v = α * v_new + (1 - α) * v_old
        │
        │ 10. Write into DriverPose_t (only if finite):
        │        pose.vecAngularVelocity = ω
        │        pose.vecVelocity        = v
        │      else: leave pose's original values untouched
        ▼
[ Real IVRServerDriverHost (SteamVR) ]
```

### Operation
1. The proxy forwards HmdDriverFactory and all initialization routines to driver_pico_orig.dll.
2. It wraps IVRServerDriverHost::TrackedDevicePoseUpdated without modifying the underlying HMD device object, preserving native DirectMode and OpenXR swapchains. Velocity synthesis only runs for k_unTrackedDeviceIndex_Hmd; all other devices pass through untouched.
3. On every tracking update, it first checks poseIsValid — an invalid pose immediately zeroes the synthesized velocity and returns without writing anything, so a lost-and-recovered HMD never reports stale pre-loss velocity. For valid poses, it gates on elapsed time (dt) since the last sample: too short (<3ms) skips the sample and keeps the last value; too long (>50ms) resets to zero rather than carrying a stale value across the gap; only the 3–50ms range computes a new world-space angular/linear velocity via QPC timestamps.
4. Smoothing uses an exponential filter with a dt-normalized coefficient (α = 1 − e^(−dt/τ), τ ≈ 6.9ms) instead of a fixed α = 0.80, so the filter's effective cutoff no longer drifts when the frame interval varies.
5. The synthesized vectors are populated into DriverPose_t.vecAngularVelocity and DriverPose_t.vecVelocity before the pose reaches SteamVR — but only when both are finite; otherwise the original driver's values are left in place.


### Driver fix Installation & Removal

### Installation
1. Close SteamVR and Pico Connect (Stop the Pico Streaming Service from the Windows services, use task manager to make sure everything VR related is closed).
2. Navigate to Pico Connect\openvr_driver\bin\win64\ (your local Pico Connect installation path).
3. IMPORTANT!!: Rename driver_pico.dll to driver_pico_orig.dll
4. Copy the fixed driver_pico.dll you downloaded from this repository into that folder (Pico Connect\openvr_driver\bin\win64\).
5. Start Pico Connect and SteamVR.

### Verification
1. Run Pico4VRMotionTest.exe it should open test in SteamVr. After the test, the audit should report:
2. Zero-Velocity Omissions on HMD: 0
3. FINAL VERDICT: TRACKING NORMAL (WORLD VELOCITY COMPLIANT)


### Removal
1. Close SteamVR and Pico Connect (Stop the Pico Streaming Service from the Windows services, use task manager to make sure everything VR related is closed).
2. Delete driver_pico.dll.
3. Rename driver_pico_orig.dll back to driver_pico.dll


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
```

