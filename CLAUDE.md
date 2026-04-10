# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`osg_3dgs` is a C++17 desktop 3D Gaussian Splatting (3DGS) viewer using **OpenSceneGraph (OSG)** for rendering and **Qt5** for the GUI. It loads `.splat` (compact binary) and `.ply` Gaussian splat files and renders them on the GPU using instanced quads with custom GLSL shaders.

## Build

**Dependencies:** Qt5 (Widgets, Core, Gui, OpenGL), OSG >= 3.6.5 (osg, osgDB, osgUtil, osgViewer, osgGA), GLM. Set `OSG_ROOT` and `GLM_DIR` environment variables if not in standard paths.

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
2. **Parse:** `GaussianDrawObj::readSplatFile()` or `readFlyFile()` converts file data into `MI_GaussianPoint[]` (position + RGBA color + 3×3 covariance matrix stored as three `Vec4f` rows).
3. **Scene setup (`getNode()`):** Creates one instanced quad geometry (4 vertices, drawn N times) and two Texture Buffer Objects (TBOs): `dataBuffer` (5 Vec4f per splat: pos, color, sigma1-3) and `indexBuffer` (sorted indices, one `int` per splat). Configures alpha blending (`DST_ALPHA/ONE` for RGB, `ZERO/ONE_MINUS_SRC_ALPHA` for alpha).
4. **Per-frame callbacks:**
   - `SSCallback` (StateSet): uploads `view`, `proj`, `viewport_size` uniforms each frame.
   - `BatchObjTextureCBEx` (on index TBO): checks `bDirty` flag and view direction change (dot-product threshold) → triggers CPU depth sort → uploads new sorted index buffer to TBO.
5. **GPU render:** `gaussian.vert` reads sorted instance index from `indexBuffer` TBO, fetches splat data from `dataBuffer` TBO, projects the 3D covariance via Jacobian of perspective projection (`J * W * sigma * W^T * J^T`), decomposes 2D covariance into eigenvectors/eigenvalues to position quad vertices along the ellipse axes. `gaussian.frag` evaluates `exp(-dot(pos,pos))` Gaussian falloff and discards fragments beyond 2σ.

### Key Files

| File | Role |
|------|------|
| `osg/GaussianDrawObj.h/.cpp` | Core: file parsing, covariance construction, TBO setup, depth sort (counting sort with 65535 buckets), OSG callbacks |
| `osg/osgWindow.h/.cpp` | Qt↔OSG bridge: dual-inherits `QOpenGLWidget` + `osgViewer::Viewer`, translates Qt events to OSG, 50ms render timer |
| `shader/gaussian.vert/.frag` | GLSL 4.30 — mathematical core of 3DGS (Jacobian projection, eigendecomposition, Gaussian evaluation) |
| `shader/point.vert/.frag` | Simpler fallback: renders splats as fixed-size `GL_POINTS` without Gaussian math |
| `tools/miniply.h/.cpp` | Vendored MIT PLY parser (by Vilya Harvey) |
| `mainwindow.cpp/.h` | Thin Qt shell: toolbar + `QFileDialog` → `loadModule()` |

### Covariance Construction

Both file formats produce the same 3×3 covariance matrix `sigma = RS * (RS)^T` where:
- **`.splat`:** rotation is decoded from 4 normalized `uint8` bytes (quaternion), scale is stored directly as `float[3]`.
- **`.ply`:** scale properties (`scale_0..2`) are exponentiated (`exp(scale)`), rotation quaternion from `rot_0..3` properties. Colors from `f_dc_0..2` (spherical harmonics DC: multiply by `1/(2*sqrt(π)) ≈ 0.282` then shift by 0.5), opacity via sigmoid.

### Depth Sort

`runSortAndUpdate()` implements a counting sort (65535 buckets by distance² from camera position), producing a back-to-front order. Sort is triggered when `bDirty == true` **and** the view direction dot-product has changed beyond a threshold. User can force a re-sort by pressing **C**.

### Camera / Navigation

`TrackballManipulator` handles mouse-based orbit/zoom. `fullScreen()` auto-fits camera to scene bounding box via `osg::ComputeBoundsVisitor`. A global `osg::Camera*` pointer (`g_pCamera` in `osgWindow.cpp`) is used by `GaussianDrawObj` callbacks to read the current view/projection matrices.
