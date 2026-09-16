**DISCLAIMER:** This software is an experimental proof-of-concept provided "AS IS", without warranty of any kind, express or implied. Use at your own risk. The author assumes no responsibility or liability for any system instability, software conflicts, loss of data, or hardware issues resulting from the use of these tools or driver modifications. This project is independent and is not affiliated with, endorsed by, or associated with Pico Immersive Pte. Ltd., ByteDance, Valve Corporation, or SteamVR.

# Pico 4 SteamVR Tracking Diagnostics & Velocity Fix (PoC)

This repository contains two components:
1. **Pico4VRMotionTest**: A standalone DirectX 12 / OpenVR diagnostic tool that audits OpenVR 6DOF velocity reporting, measures head-tilt coordinate frame errors, and benchmarks Timewarp/Prediction across simulated framerates.
2. **driver_pico**: An OpenVR proxy wrapper .DLL for Pico Connect that adds missing world-space angular and linear velocity vectors in real time, with dynamic pose time offsetting.

---

## Background & Problem Description

The OpenVR specification requires TrackedDevicePose_t.vAngularVelocity and vVelocity to be provided in the **World tracking space** (radians/second and meters/second).

On Pico 4 (not Pico 4 Ultra) the drivers handle velocity data inconsistently:
* **Pico Connect:** Omits velocity reporting entirely, passing vAngularVelocity = (0, 0, 0) and vVelocity = (0, 0, 0) across all OpenVR pathways.

### Consequences of These Implementations
1. **SteamVR Motion Prediction:** When the head is tilted (20°–60°) and rotated, missing or local-frame velocity causes SteamVR's forward extrapolation to fail.
2. **Full-Body Tracking (FBT) & Space Calibration:** Users on the Pico Discord reported bugs with systems that reference the HMD pose (e.g., Fluxpose, can't verify, I don't own extra trackers).

---

## 1. How the Diagnostic Test Works (Pico4VRMotionTest)

The diagnostic tool runs directly inside SteamVR via DirectX 12 and renders an in-headset HUD placed at 1.5 meters depth.

### Test Procedure
* **Phase 1 (Upright Rotation, 10s):** Measures rotational velocity while the head remains upright (Tilt < 15°).
* **Phase 2 (Tilted Rotation, 15s):** Measures rotational velocity while the head is tilted (> 25°).
* **Phase 3 (Translational Leaning, 15s):** Measures linear velocity and prediction overshoot while leaning forward/backward (Z-axis).

### Analysis Performed
* **Ground-Truth Derivation:** Computes physical world angular and linear velocity directly from quaternion and position differentials (dq / dt, dp / dt).
* **Model Fitting:** Compares the driver's reported vector against frames to detect mismatches and linear prediction overshoot.
* **Dynamic Framerate Simulation:** Press keys **1-5** on your keyboard during the test to simulate 90 FPS, 45 FPS (reprojected), 60 FPS, heavy stalls, or a dynamic framerate sweep.
* **Multi-Pathway OpenVR Audit:** Queries OpenVR pose retrieval pathways simultaneously (Instantaneous, Predicted, Raw, Render, Game).

### Output Files
Results are written to logs/run_<timestamp>/:
* **summary.txt**: Summary report with pathway audit, per-phase breakdown, and final verdict.
* **tracking_errors.csv**: Sample-by-sample telemetry with timestamps, quaternions, raw reported vectors, derived vectors, and error metrics.
* **incident_transitions.csv**: Tracks pre- and post-incident telemetry immediately surrounding tracking failures or prediction errors for debugging.

---

## 2. How the Proxy Driver Fix Works (driver_pico.dll)

The proxy driver is a drop-in wrapper placed in Pico Connect's OpenVR driver directory.

```text
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
        │        ├─ dt > 350ms  → ω = v = 0  (stale-gap reset)
        │        ├─ dt < 0.2ms  → keep previous ω, v  (skip sample)
        │        └─ 0.2ms..350ms→ continue to step 4
        │
        │  4. Compute relative rotation & angle
        │  5. Compute angular velocity (ω_world)
        │  6. Compute linear velocity (v_world):
        │        Uses a jitter-decoupled step cadence estimator.
        │
        │  7. Smoothing & Cadence:
        │        Dynamic cutoff frequency (fc) based on movement speed.
        │        ω = α_rot * ω_new + (1 - α_rot) * ω_old
        │        v = α_lin * v_cadence + (1 - α_lin) * v_old
        │
        │  8. Frame Pacing & Overrides:
        │        pose.poseTimeOffset = 0.5 * step_interval (Neutralizes Pico's hardcoded offset)
        │        pose.vecAcceleration = 0
        │
        │  9. Write into DriverPose_t (only if finite)
        ▼
[ Real IVRServerDriverHost (SteamVR) ]
```

### Operation
1. The proxy forwards `HmdDriverFactory` and all initialization routines to `driver_pico_orig.dll`.
2. It wraps `IVRServerDriverHost::TrackedDevicePoseUpdated` without modifying the underlying HMD device object, preserving native DirectMode and OpenXR swapchains.
3. On every tracking update, it first checks `poseIsValid`. Valid poses gate on elapsed time (dt) to avoid relying on dropped or stale frames. 
4. Smoothing uses dynamic cutoff frequencies based on movement speed. Linear velocity incorporates a jitter-decoupled step cadence estimator to prevent micro-stutters.
5. The driver dynamically sets `pose.poseTimeOffset` to half the estimated hardware frame interval. This replaces Pico's hardcoded timing offset (which assumed a fixed frame rate) to ensure prediction is accurate across variable frame rates. It also zeroes out acceleration vectors to prevent SteamVR extrapolation overshoot.
6. The synthesized vectors are populated into `DriverPose_t.vecAngularVelocity` and `DriverPose_t.vecVelocity` before the pose reaches SteamVR. (Protected via `SRWLOCK` scheduling).

### Driver fix Installation & Removal

### Installation
1. Close SteamVR and Pico Connect (Stop the Pico Streaming Service from the Windows services, use task manager to make sure everything VR related is closed).
2. Navigate to `Pico Connect\openvr_driver\bin\win64\` (your local Pico Connect installation path).
3. **IMPORTANT!!**: Rename `driver_pico.dll` to `driver_pico_orig.dll`
4. Copy the fixed `driver_pico.dll` you downloaded from this repository into that folder.
5. Start Pico Connect and SteamVR.

### Verification
1. Run `Pico4VRMotionTest.exe` it should open the test in SteamVR. After the test, the audit should report:
2. Zero-Velocity Omissions on HMD: 0
3. Extrapolation Overshoot Events: 0 (or reduced)

### Removal
1. Close SteamVR and Pico Connect (Stop the Pico Streaming Service from the Windows services).
2. Delete `driver_pico.dll`.
3. Rename `driver_pico_orig.dll` back to `driver_pico.dll`.

## Building from Source

### Prerequisites (in GitHub Codespaces)
The repository includes the LLVM-MinGW toolchain configuration.

### Build Steps
```bash
# 1. Configure the build environment
export LLVM_MINGW_ROOT="/workspaces/codespaces-blank/.tools/llvm-mingw"

# 2. Build both targets
ninja -C build-win64
# Build outputs in build-win64/:
# Pico4VRMotionTest.exe
# driver_pico.dll
```