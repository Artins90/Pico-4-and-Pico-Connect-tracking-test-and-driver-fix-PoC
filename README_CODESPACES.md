# Building from GitHub Codespaces

This project creates a Windows x64 executable. The Codespace itself is Linux, so it uses LLVM-MinGW to cross-compile PE/COFF Windows binaries. The resulting `.exe` must still be copied to a Windows PC with SteamVR and the Pico headset to run the test.

## First setup

```bash
bash scripts/setup-codespaces.sh
```

## Build

```bash
bash scripts/build-codespaces.sh
```

Output:

```text
build-win64/Pico4VRMotionTest.exe
build-win64/openvr_api.dll
```

## VS Code / CMake Tools

After setup, select the `codespaces-win64` CMake preset. Then choose **Build**. CMake Tools will use `cmake/llvm-mingw-x86_64.cmake` and create the Windows executable in `build-win64/`.

## Important

This is a cross-compile only. Codespaces cannot run the application as a SteamVR VR application because the container does not have the Windows SteamVR runtime, Windows DirectX compositor, or the Pico PCVR connection.
