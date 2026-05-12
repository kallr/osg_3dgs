# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`osg_3dgs` is a C++17 desktop 3D Gaussian Splatting (3DGS) viewer using **OpenSceneGraph (OSG)** for rendering and **Qt5** for the GUI. It loads `.splat` (compact binary) and `.ply` Gaussian splat files and renders them on the GPU using instanced quads with custom GLSL shaders.

## Build

**Dependencies:** Qt5 (Widgets, Core, Gui, OpenGL), OSG >= 3.6.5 (osg, osgDB, osgUtil, osgViewer, osgGA). Set `OSG_ROOT` environment variable if not in standard paths. GLM is no longer required.

```bash
# Linux/macOS
mkdir build && cd build && cmake .. && make -j4

# Windows: CMake generates build/osg_gaussian_draw.sln — open in Visual Studio
```

**Post-build (manual):** Copy the `shader/` directory into `build/bin/shader/` so the runtime shader loader can find it. The shaders are loaded at runtime from the filesystem relative to the executable.

No test framework. No linting/formatting configuration.

## Architecture

### Data Flow

1. **Load:** `GraphicsWindowQt::loadModule()` branches on extension — `.splat`/`.ply` go to `GaussianDrawObj`, anything else to `osgDB::readNodeFile()`.
2. **Parse:** `readSplatFile()` or `readPlyFile()` converts file data into `MI_GaussianPoint[]` (position + RGBA color + scale + quaternion + SH coefficients). Covariance is NOT precomputed on CPU — it's computed on the GPU.
3. **Scene setup (`getNode()`):** Creates one instanced quad geometry (4 vertices as 2 triangles, drawn N times) and TBOs:
   - `indexBuffer` (TBO unit 0): sorted indices
   - `dataBuffer` (TBO unit 1): 4 Vec4f per splat (pos+alpha, scale+R, quat.xyz+G, quat.w+B)
   - `shBuffer` (TBO unit 2, optional): 12 Vec4f per splat (45 SH coefficients)
   Configures premultiplied alpha blending (`GL_ONE / GL_ONE_MINUS_SRC_ALPHA`).
4. **Per-frame callbacks:**
   - `GaussianSortCallback` (Camera pre-draw): detects view matrix changes, triggers background sort.
   - `SSCallback` (StateSet): uploads `view`, `proj`, `viewport_size` uniforms each frame.
   - `IndexBufferUpdateCallback` (on index TBO): applies sort results from background thread.
5. **GPU render:** `gaussian.vert` computes 3D covariance from scale+quaternion on GPU (`R * S² * Rᵀ`), projects via Jacobian (`J * W * sigma * Wᵀ * Jᵀ`), computes ellipse extents from eigenvalues, positions quad vertices, and passes inverse 2D covariance to fragment shader. `gaussian.frag` evaluates `exp(-0.5 * dot(d, cov2Dinv * d))` and discards below alpha threshold.

### Key Files

| File | Role |
|------|------|
| `osg/GaussianDrawObj.h/.cpp` | Core: file parsing, TBO setup, background radix sort thread, OSG callbacks |
| `osg/osgWindow.h/.cpp` | Qt↔OSG bridge: dual-inherits `QOpenGLWidget` + `osgViewer::Viewer`, translates Qt events to OSG, 50ms render timer |
| `shader/gaussian.vert/.frag` | GLSL 4.30 — GPU covariance computation, Jacobian projection, eigenvalue ellipse, inverse cov2D Gaussian evaluation |
| `shader/point.vert/.frag` | Simpler fallback: renders splats as fixed-size `GL_POINTS` without Gaussian math |
| `tools/miniply.h/.cpp` | Vendored MIT PLY parser (by Vilya Harvey) |
| `mainwindow.cpp/.h` | Thin Qt shell: toolbar + `QFileDialog` → `loadModule()` |

### Covariance Construction (GPU)

Covariance is computed entirely on the GPU in the vertex shader:
- `computeCovariance(scale, quat)` builds `R * S² * Rᵀ` from the stored scale vector and quaternion.
- This saves CPU memory (4+4 floats vs 12 floats per splat) and eliminates CPU-GPU bandwidth for the covariance matrix.

### Depth Sort

`GaussianSortThread` runs a background radix sort (8-bit, 4-pass LSB radix sort on float-as-uint32 depth keys). Sorting is triggered automatically when the view matrix changes. The sort produces back-to-front ordering. User can also force a re-sort by pressing **C**.

### Camera / Navigation

`TrackballManipulator` handles mouse-based orbit/zoom. `fullScreen()` auto-fits camera to scene bounding box via `osg::ComputeBoundsVisitor`. A global `osg::Camera*` pointer is used by callbacks to read the current view/projection matrices.
