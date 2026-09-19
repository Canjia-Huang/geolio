# Geolio

[![Linux CI](https://github.com/Canjia-Huang/geolio/actions/workflows/linux.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/linux.yml)
[![macOS CI](https://github.com/Canjia-Huang/geolio/actions/workflows/macos.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/macos.yml)
[![Windows CI](https://github.com/Canjia-Huang/geolio/actions/workflows/windows.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/windows.yml)
[![Nightly](https://github.com/Canjia-Huang/geolio/actions/workflows/nightly.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/nightly.yml)

[![Linux Consumer](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_linux.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_linux.yml)
[![macOS Consumer](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_macos.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_macos.yml)
[![Windows Consumer](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_windows.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_windows.yml)
[![Consumer Nightly](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_nightly.yml/badge.svg)](https://github.com/Canjia-Huang/geolio/actions/workflows/consumer_nightly.yml)

**Geolio** is a C++ library designed for performing various processing tasks in computer graphics (mainly mesh processing).

## Dependencies

**Supplied by you**

- [**Geogram**](https://github.com/BrunoLevy/geogram) — required. Located through `GEOGRAM_DIR` (environment variable or `-DGEOGRAM_DIR=...`), or an installed Geogram found by CMake.
- [**Eigen3**](https://eigen.tuxfamily.org) — required, via `find_package(Eigen3)`. A parent project that already provides the `Eigen3::Eigen` target is used as-is.

**Bundled as git submodules** in `third_party/` — clone with `--recurse-submodules`, and a project embedding geolio must run `git submodule update --init --recursive`:

- [**spdlog**](https://github.com/gabime/spdlog), [**CLI11**](https://github.com/CLIUtils/CLI11) — logging and CLI parsing, header-only.
- [**imoguizmo**](https://github.com/fknfilewalker/imoguizmo) — the ImGui/ImGuizmo integration used by `geobox`, on top of the imgui headers bundled with Geogram.
- [**googletest**](https://github.com/google/googletest) — test framework (1.12.1), built only when `GEOLIO_BUILD_TESTS=ON`.

**Fetched at configure time** with CMake FetchContent (so these need network access to GitHub), each one only when the option that uses it is on:

- [**LBFGS-Lite**](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) (tag `v2.3`) — header-only L-BFGS optimizer, with `GEOLIO_ENABLE_LBFGS_LITE=ON`.
- [**CoMISo**](https://github.com/libigl/CoMISo) — MIQ's mixed-integer solver, built as a static library with `GEOLIO_ENABLE_COMISO=ON`. Geolio consumes it through [its own fork](https://github.com/Canjia-Huang/CoMISo) pinned to a commit, so the changes geolio needs travel as commits instead of as a build-time patch. Not header-only, and it links BLAS: OpenBLAS bundled on Windows, the Accelerate framework on macOS, `find_package(BLAS REQUIRED)` elsewhere (e.g. `sudo apt-get install libblas-dev liblapack-dev`).
- [**libigl**](https://github.com/libigl/libigl) — header-only, needed only by MIQ. Geolio does **not** fetch it — see [MIQ](#mixed-integer-quadrangulation-miq).

## Building

```bash
git clone --recurse-submodules https://github.com/Canjia-Huang/geolio.git
cd geolio
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
```

| Option                     | Default                      | Description                                                          |
|----------------------------|------------------------------|----------------------------------------------------------------------|
| `GEOLIO_BUILD_TESTS`       | `ON` (`OFF` as a submodule)  | Build `Geolio_tests`                                                 |
| `GEOLIO_ENABLE_LBFGS_LITE` | `OFF`                        | LBFGS-Lite optimizer, fetched at configure time                      |
| `GEOLIO_ENABLE_COMISO`     | `OFF`                        | CoMISo solver and BLAS, fetched at configure time                    |
| `GEOLIO_ENABLE_MIQ`        | `OFF`                        | MIQ pipeline; forces `GEOLIO_ENABLE_COMISO=ON` and requires libigl   |

The main CI workflows (Linux, macOS, Windows, nightly) build with all three optional features `ON`; the consumer workflows build geolio embedded with the defaults. Run the test suite with:

```bash
ctest --test-dir build --output-on-failure -C Release
```

### Mixed-integer quadrangulation (MIQ)

`GEOLIO_ENABLE_MIQ=ON` compiles `src/geolio/miq` into the library and builds the `miq` app. It also forces `GEOLIO_ENABLE_COMISO=ON` — in the cache, even if you passed `-DGEOLIO_ENABLE_COMISO=OFF` — and makes libigl a hard requirement, supplied either way:

```bash
# A libigl source tree: header-only, so no install is needed (also read from the
# LIBIGL_DIR environment variable).
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOLIO_ENABLE_MIQ=ON -DLIBIGL_DIR=/path/to/libigl

# Or an installed libigl package: find_package(libigl) only sees the installed config
# file, which does not exist for a source checkout alone.
cmake -S /path/to/libigl -B /path/to/libigl-build -DCMAKE_BUILD_TYPE=Release
cmake --install /path/to/libigl-build --prefix /path/to/libigl-prefix
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOLIO_ENABLE_MIQ=ON \
      -Dlibigl_DIR=/path/to/libigl-prefix/lib/cmake/igl
```

`LIBIGL_DIR` accepts the libigl source root or its `include` directory. For the installed package, `libigl_DIR` must point at the directory holding `libigl-config.cmake`: libigl installs it under `<prefix>/lib/cmake/igl` while the package is named `libigl`, so `-DCMAKE_PREFIX_PATH=<prefix>` on its own does not find it. Tested with libigl `v2.5.0`.

## Using Geolio as a submodule

```bash
git submodule add https://github.com/Canjia-Huang/geolio.git third_party/geolio
git submodule update --init --recursive   # geolio's own submodules
```

```cmake
cmake_minimum_required(VERSION 3.30)
project(MyApp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Put geolio's shared library next to your executables so that it resolves at
# runtime (Windows; Linux and macOS manage this through RPATH).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(third_party/geolio)   # exposes the target Geolio::geolio

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Geolio::geolio)
```

CMake options are inherited, so set `GEOLIO_BUILD_TESTS` and the `GEOLIO_ENABLE_*` options **before** `add_subdirectory`. Geogram and Eigen3 are located exactly as in the standalone build.

## License

BSD 3-Clause — see [LICENSE](LICENSE).
