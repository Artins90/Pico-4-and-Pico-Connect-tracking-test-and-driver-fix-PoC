set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# LLVM-MinGW root is supplied by the environment so it remains valid
# when CMake re-runs this toolchain file from a try_compile directory.
if(DEFINED ENV{LLVM_MINGW_ROOT} AND NOT "$ENV{LLVM_MINGW_ROOT}" STREQUAL "")
    file(TO_CMAKE_PATH "$ENV{LLVM_MINGW_ROOT}" LLVM_MINGW_ROOT)
else()
    message(FATAL_ERROR
        "LLVM_MINGW_ROOT environment variable is not set.\n"
        "Example:\n"
        "  export LLVM_MINGW_ROOT=\"$PWD/.tools/llvm-mingw\"\n"
    )
endif()

if(NOT EXISTS "${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-clang++")
    message(FATAL_ERROR
        "LLVM-MinGW compiler not found at:\n"
        "  ${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-clang++"
    )
endif()

set(CMAKE_C_COMPILER
    "${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-clang"
    CACHE FILEPATH "C compiler" FORCE
)

set(CMAKE_CXX_COMPILER
    "${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-clang++"
    CACHE FILEPATH "C++ compiler" FORCE
)

set(CMAKE_RC_COMPILER
    "${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-windres"
    CACHE FILEPATH "Resource compiler" FORCE
)

set(CMAKE_AR
    "${LLVM_MINGW_ROOT}/bin/llvm-ar"
    CACHE FILEPATH "Archiver" FORCE
)

set(CMAKE_RANLIB
    "${LLVM_MINGW_ROOT}/bin/llvm-ranlib"
    CACHE FILEPATH "Ranlib" FORCE
)

set(CMAKE_STRIP
    "${LLVM_MINGW_ROOT}/bin/llvm-strip"
    CACHE FILEPATH "Strip" FORCE
)

set(CMAKE_FIND_ROOT_PATH
    "${LLVM_MINGW_ROOT}"
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Do not try to execute the Windows target during cross compilation.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)