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

## Requirements / dependencies

- [**Geogram**](https://github.com/BrunoLevy/geogram) — geometry library, required at configure time (set `GEOGRAM_DIR=/path/to/geogram`).
- [**Eigen3**](https://eigen.tuxfamily.org) — header-only linear algebra library, required at configure time via `find_package(Eigen3)`.
- [**spdlog**](https://github.com/gabime/spdlog), [**CLI11**](https://github.com/CLIUtils/CLI11) and [**imoguizmo**](https://github.com/fknfilewalker/imoguizmo) — bundled as git submodules; used as header-only include paths. imoguizmo provides the ImGui/ImGuizmo integration used by the GeoBox application and relies on the imgui headers bundled with Geogram.
- [**googletest**](https://github.com/google/googletest) — test framework, bundled as the `third_party/googletest` git submodule (pinned to googletest 1.12.1) and built only when `BUILD_TESTS=ON`. Nothing is downloaded at configure time to run the test suite.
- [**LBFGS-Lite**](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) — header-only L-BFGS unconstrained optimizer (optional, enabled by default). It is **not** vendored: when `GEOLIO_ENABLE_LBFGS_LITE=ON` it is downloaded automatically at configure time via CMake FetchContent (pinned to tag `v2.3`).
- [**CoMISo**](https://github.com/libigl/CoMISo) — mixed-integer constrained solver used by the MIQ pipeline (optional, enabled by default). It is **not** vendored: when `GEOLIO_ENABLE_COMISO=ON` it is downloaded at configure time via CMake FetchContent, built as a static library, and linked into `geolio::third_party`. Geolio consumes it through [its own fork](https://github.com/Canjia-Huang/CoMISo) pinned to a commit, so that the changes geolio needs can be carried as commits (and offered back upstream) instead of as a build-time patch. Unlike the other optional dependencies CoMISo is **not** header-only and needs BLAS: `find_package(BLAS REQUIRED)` on non-Apple UNIX (e.g. `sudo apt-get install libblas-dev liblapack-dev`), the Accelerate framework on macOS, nothing extra on Windows.
- [**libigl**](https://github.com/libigl/libigl) — header-only geometry library, required **only** by the MIQ pipeline. Geolio does not download it: it must either be installed so that `find_package(libigl)` succeeds, or be pointed at with `LIBIGL_DIR=/path/to/libigl`. See [Enabling MIQ](#enabling-miq).

> Geolio must be cloned with `--recurse-submodules`, and a project that consumes geolio as a submodule must run `git submodule update --init --recursive`, so that the nested submodules above are present.

## Building the library

```bash
git clone --recurse-submodules https://github.com/Canjia-Huang/geolio.git
cd geolio
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
```

CMake options:

| Option                     | Default | Description                                                                  |
|----------------------------|---------|------------------------------------------------------------------------------|
| `BUILD_TESTS`              | `ON`    | Build the test suite (googletest, from the `third_party/googletest` submodule). |
| `GEOLIO_ENABLE_LBFGS_LITE` | `ON`    | Enable the LBFGS-Lite header-only optimizer; downloaded via FetchContent at configure time (requires network access to GitHub). |
| `GEOLIO_ENABLE_COMISO`     | `ON`    | Enable the CoMISo mixed-integer constrained solver; downloaded from geolio's CoMISo fork via FetchContent and built as a static library. Requires BLAS — see [Requirements](#requirements--dependencies). |
| `GEOLIO_ENABLE_MIQ`        | `OFF`   | Build the Mixed-Integer Quadrangulation pipeline (`src/geolio/miq`). Forces `GEOLIO_ENABLE_COMISO=ON` and additionally requires libigl — see [Enabling MIQ](#enabling-miq). |

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOLIO_ENABLE_LBFGS_LITE=OFF
```

### Enabling MIQ

`GEOLIO_ENABLE_MIQ` builds the Mixed-Integer Quadrangulation pipeline in `src/geolio/miq`. It is `OFF` by default, and turning it on has two consequences:

- **CoMISo is forced on.** MIQ cannot be built without it, so `GEOLIO_ENABLE_COMISO` is set to `ON` in the CMake cache even if you passed `-DGEOLIO_ENABLE_COMISO=OFF`.
- **libigl becomes required.** libigl is not downloaded by geolio, and configure fails with an explanatory error if it cannot be found. Supply it in either of two ways.

**1. An installed libigl package** (preferred). Note that `find_package(libigl)` needs a `libigl-config.cmake` file, which only exists once libigl has been installed — a source checkout alone is not enough:

```bash
cmake -S /path/to/libigl -B /path/to/libigl-build -DCMAKE_BUILD_TYPE=Release
cmake --install /path/to/libigl-build --prefix /path/to/libigl-prefix
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOLIO_ENABLE_MIQ=ON \
      -Dlibigl_DIR=/path/to/libigl-prefix/lib/cmake/igl
```

libigl installs its package config to `<prefix>/lib/cmake/igl`, while the package is named `libigl`. Since `find_package` matches the directory name against the package name, `-DCMAKE_PREFIX_PATH=<prefix>` alone will **not** find it — point `libigl_DIR` (or `CMAKE_PREFIX_PATH`) at the directory that actually contains `libigl-config.cmake`.

**2. A libigl source tree**, which needs no install at all. Because libigl is header-only, making its headers visible is sufficient:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOLIO_ENABLE_MIQ=ON \
      -DLIBIGL_DIR=/path/to/libigl
```

`LIBIGL_DIR` defaults to the `LIBIGL_DIR` environment variable, and may point either at the libigl source root or directly at its `include` directory. Tested with libigl `v2.5.0`.

## Using Geolio as a git submodule

Geolio is designed to be embedded into other projects as a git submodule and consumed through `add_subdirectory`.

### 1. Add the submodule

```bash
git submodule add https://github.com/Canjia-Huang/geolio.git third_party/geolio
git submodule update --init --recursive
```

`--recursive` also pulls geolio's own nested submodules (spdlog, CLI11, imoguizmo, googletest).

### 2. Link it from your `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.30)
project(MyApp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Centralize build outputs so the app and geolio's shared libraries/DLLs land in
# the same tree and are easy to find at runtime.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Embed geolio; exposes the target Geolio::geolio
add_subdirectory(third_party/geolio)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Geolio::geolio)
```

CMake options are inherited from the parent project, so the optional dependencies are controlled by setting `GEOLIO_ENABLE_LBFGS_LITE`, `GEOLIO_ENABLE_COMISO` and `GEOLIO_ENABLE_MIQ` **before** `add_subdirectory`.

The output directories above put the built `Geolio.dll` next to your executable (in `build/bin/<config>` on Windows), so it is resolved at runtime without extra setup. Without them, on Windows the DLL lands in geolio's nested build subdirectory and must be added to `PATH`; Linux/macOS resolve the shared library automatically through RPATH.

### 3. Make Geogram, Eigen3 and libigl visible

The submodule build finds Geogram and Eigen3 through the same mechanisms as the standalone build. Set `GEOGRAM_DIR` (environment variable or `-DGEOGRAM_DIR=...`), and make sure Eigen3 is installed (see [Requirements](#requirements--dependencies)) if it is not already available:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOGRAM_DIR=/path/to/geogram
```

If the parent project already provides an `Eigen3::Eigen` target (e.g. through its own `find_package(Eigen3)`), geolio detects it and skips its own lookup.

libigl is only needed when `GEOLIO_ENABLE_MIQ` is enabled, and is located the same way as in the standalone build — an installed package via `libigl_DIR`, or a source tree via `LIBIGL_DIR` (see [Enabling MIQ](#enabling-miq)).

## License

BSD 3-Clause — see [LICENSE](LICENSE).
