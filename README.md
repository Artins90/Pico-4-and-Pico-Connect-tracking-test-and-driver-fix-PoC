# Pico 4 SteamVR motion-prediction error test (DX12 + OpenVR)

This is a small Windows x64 diagnostic application intended for a Pico 4 / Pico 4 Pro / Pico 4 Enterprise connected to SteamVR.

It does **not** attempt to fix tracking, inject poses, or modify SteamVR/Pico software. It only observes the HMD pose/velocity exposed to a SteamVR application and looks for tracking/prediction mistakes.

## What it measures

The test samples the HMD at zero requested prediction and keeps a short history. For a configurable prediction horizon (default 11.1 ms), it asks:

1. If I take the pose and reported angular velocity from time `t`, where would that velocity predict the headset will be at `t + horizon`?
2. Where did the headset pose actually end up at `t + horizon`?
3. Does the reported angular velocity fit better when interpreted as tracking/world-space or as headset-local-space and rotated by the measured head orientation?
4. Does reported linear velocity predict the measured position change?
5. Are there pose invalidations, abrupt orientation jumps, or velocity discontinuities?

The app writes **tracking mistakes only** to CSV. Normal samples are not written as event rows.

Important: OpenVR's application-facing pose structure exposes velocity and angular velocity, but it does not expose a raw IMU acceleration stream. The tool therefore derives angular acceleration from changes in the reported angular-velocity vector; this is a diagnostic signal, not a claim that SteamVR provides raw acceleration to applications.

## Files

- `src/main.cpp` - complete DX12/OpenVR diagnostic.
- `CMakeLists.txt` - x64 Windows build using CMake + Ninja + clang-cl.
- `scripts/fetch_openvr.ps1` - downloads Valve's OpenVR SDK source tree.

## Portable build toolchain (no full Visual Studio IDE)

You need:

- LLVM/clang-cl for Windows.
- CMake.
- Ninja.
- Git.
- A Windows SDK (the DirectX 12 headers and libraries come from the Windows SDK).

A full Visual Studio IDE is not required. Microsoft publishes the Windows SDK separately; the current SDK download page lists Windows 11 SDK installers and ISOs. See the links in the project handoff message.

### 1. Fetch OpenVR

From PowerShell in this project directory:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\fetch_openvr.ps1
```

The script pulls Valve's `openvr` repository into `./openvr`.

### 2. Configure a 64-bit clang-cl + Ninja build

Open a normal PowerShell prompt where `clang-cl.exe` and `cmake.exe` are on `PATH`.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl
```

If `clang-cl` cannot locate the Windows SDK automatically, set `WindowsSdkDir`/`WindowsSdkVersion` in the environment or use a toolchain file that points clang-cl at your installed SDK. On a standard LLVM-for-Windows installation with the Windows SDK installed, clang-cl normally discovers the SDK through the Windows environment/registry.

### 3. Build

```powershell
cmake --build build --config Release
```

The executable will be:

```text
build\Pico4VRMotionTest.exe
```

`openvr_api.dll` is copied beside it automatically.

### 4. Run

Start SteamVR first and connect the Pico headset through your normal PCVR transport. Then launch:

```powershell
.\build\Pico4VRMotionTest.exe --horizon-ms 11.1 --log-dir logs
```

The application opens a minimal companion window and submits a minimal DX12 eye image with the diagnostic HUD to SteamVR.

The default 11.1 ms horizon is deliberately close to one frame at 90 Hz. You can test several horizons:

```powershell
.\build\Pico4VRMotionTest.exe --horizon-ms 5.5 --log-dir logs_5ms
.\build\Pico4VRMotionTest.exe --horizon-ms 11.1 --log-dir logs_11ms
.\build\Pico4VRMotionTest.exe --horizon-ms 16.7 --log-dir logs_17ms
```

## Suggested Pico 4 test procedure

Run the executable for at least 30–60 seconds per condition.

1. Hold the head level and make smooth yaw turns.
2. Roll the head about 15° to the left and repeat yaw.
3. Roll about 30° and repeat.
4. Roll about 45° and repeat.
5. Repeat while looking up/down while rolled.
6. Repeat the same motions in the opposite roll direction.
7. Run the same sequence with the headset as level as practical.

The important output is whether the angular-velocity/prediction residual increases with head roll/tilt, and whether the local-frame hypothesis consistently fits the measured pose better than the world-frame hypothesis.

## Outputs

Each run creates a directory containing:

- `tracking_errors.csv` - only samples classified as tracking mistakes/anomalies.
- `summary.txt` - aggregate counts and the strongest local-vs-world evidence.

The CSV includes:

- timestamp
- horizon_ms
- head_roll_deg
- angular_prediction_error_deg
- angular_velocity_error_world_deg_s
- angular_velocity_error_local_deg_s
- preferred_velocity_frame (`WORLD`, `LOCAL`, or `AMBIGUOUS`)
- linear_prediction_error_mm
- derived_angular_accel_jump_deg_s2
- issue flags

## Interpreting the suspected Pico bug

A result supporting a frame/coordinate bug would look like this:

- prediction/velocity residuals remain relatively low when the head is level;
- the residual rises as roll/tilt increases;
- the same motions show a materially lower residual under the LOCAL interpretation than the WORLD interpretation (or vice versa, depending on the exact driver/runtime convention);
- the effect is repeatable over both roll directions and several horizons.

That does **not** prove that Pico's internal implementation is specifically "headset up". It establishes an observable mismatch in the angular-velocity coordinate model seen by a SteamVR application. A driver/runtime trace or Pico-side raw-IMU capture would be needed to prove the internal source.

The two other issues you mentioned (full-body foot sliding and Steam Link's wobbly/drunk view) are deliberately **not merged into the same verdict**. This test can flag motion/pose residuals that accompany them, but it cannot identify whether the underlying cause is Pico filtering, Steam Link transport, a body-tracking solver, or something downstream.

## Notes on DX12 submission

SteamVR documents a dedicated `TextureType_DirectX12` path using `D3D12TextureData_t`. The application follows that path and transitions the eye image to `D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE` before submitting it.
