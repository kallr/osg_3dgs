# OSG 3D Gaussian Splatting 渲染器

基于 OpenSceneGraph (OSG) 的实时 3D Gaussian Splatting (3DGS) 渲染器。使用 GPU 实例化渲染和自定义 GLSL 4.30 着色器，通过 Texture Buffer Object (TBO) 传递高斯点数据，实现高质量实时渲染。

---

## 功能特性

- 支持 `.splat` 和 `.ply` 两种高斯点云格式
- GPU 实例化渲染（Instanced Quad），每个高斯点一个实例
- 球谐函数 (Spherical Harmonics) 0–3 阶支持（每点 45 个 SH 系数），实现视角相关颜色
- 基于 Jacobian 的 3D→2D 协方差投影，特征值分解定位椭圆 Quad
- CPU 端 Counting Sort 深度排序（65535 桶），Back-to-front Alpha 混合
- 按键 `C` 触发重新排序，视角变化超过阈值时自动跳过无效排序
- Qt5 图形界面 + OSG Trackball 交互式相机控制
- 自动场景包围盒适配

---

## 技术栈

| 组件 | 说明 |
|------|------|
| 语言 | C++17 |
| 3D 引擎 | OpenSceneGraph (OSG) ≥ 3.6.5 |
| GUI | Qt5（Widgets、Core、Gui、OpenGL） |
| 数学库 | GLM（header-only） |
| PLY 解析 | miniply（内置，MIT 协议，by Vilya Harvey） |
| 图形 API | OpenGL 4.3+（GLSL 430 core） |
| 构建系统 | CMake 3.12+ |

---

## 目录结构

```
osg_3dgs/
├── CMakeLists.txt            # CMake 构建配置
├── main.cpp                  # 程序入口
├── mainwindow.h/cpp          # Qt 主窗口（工具栏 + 文件打开）
├── osg/
│   ├── osgWindow.h/cpp       # Qt↔OSG 桥接（QOpenGLWidget + osgViewer::Viewer）
│   └── GaussianDrawObj.h/cpp # 核心：文件解析、TBO 构建、深度排序、OSG 回调
├── shader/
│   ├── gaussian.vert         # 高斯泼溅顶点着色器（协方差投影 + SH 求值）
│   ├── gaussian.frag         # 高斯泼溅片段着色器（Gaussian falloff）
│   ├── point.vert            # 简单点渲染顶点着色器
│   └── point.frag            # 简单点渲染片段着色器
└── tools/
    ├── miniply.h/cpp         # PLY 文件解析器（第三方）
    └── tools.h/cpp           # 工具函数（路径获取等）
```

---

## 渲染管线

```
加载文件 → 解析为 MI_GaussianPoint[] → 构建 TBO (dataBuffer + indexBuffer + shBuffer)
                                              ↓
每帧回调: SSCallback 上传 view/proj/viewport → BatchObjTextureCBEx 检查排序
                                              ↓
GPU: gaussian.vert 读取 TBO → Jacobian 投影协方差 → 特征值分解 → 定位 Quad 顶点
                                              ↓
GPU: gaussian.frag → exp(-dot(pos,pos)) Gaussian 衰减 → Alpha 混合输出
```

### TBO 数据布局

| TBO | 绑定点 | 内容 |
|-----|--------|------|
| `indexBuffer` | 0 | 排序后的索引（每点 1 个 Vec4f，仅用 x 分量） |
| `dataBuffer` | 1 | 每点 5 个 Vec4f：position、color、sigma1、sigma2、sigma3 |
| `shBuffer` | 2 | 每点 12 个 Vec4f（45 个 SH float + 3 padding），按通道分组 |

---

## 核心类 API

### `GaussianDrawObj`

| 方法 | 说明 |
|------|------|
| `GaussianDrawObj(const std::string& filePath)` | 构造，自动根据扩展名选择 `.splat` 或 `.ply` 解析 |
| `osg::ref_ptr<osg::Node> getNode()` | 返回完整的 OSG 场景节点（含 TBO、着色器、混合状态） |
| `setDirty(bool)` | 标记/清除排序脏标志 |
| `runSortAndUpdate(viewProj, indexImage)` | 执行 Counting Sort 并更新 index TBO |

### `GraphicsWindowQt`

| 方法 | 说明 |
|------|------|
| `loadModule(const std::string& file)` | 加载模型文件（`.splat`/`.ply` 走高斯渲染，其他走 osgDB） |
| `fullScreen()` | 自动适配相机到场景包围盒 |

---

## 编译构建

### 前置依赖

- CMake 3.12+
- C++17 编译器（MSVC 2017+ / GCC 7+ / Clang 5+）
- Qt5（Widgets、Core、Gui、OpenGL）
- OpenSceneGraph ≥ 3.6.5
- GLM（header-only）

### 环境变量（可选）

- `OSG_ROOT` — OSG 安装根目录（含 `include/`、`debug/lib/`、`release/lib/`）
- `GLM_DIR` — GLM 头文件所在目录

### 编译步骤

```bash
git clone <repo_url>
cd osg_3dgs
mkdir build && cd build
cmake ..
```

**Linux/macOS:**
```bash
make -j4
```

**Windows:**
CMake 生成 `build/osg_gaussian_draw.sln`，用 Visual Studio 打开编译。

构建过程会自动将 `shader/` 目录复制到可执行文件所在目录（`build/bin/shader/`），无需手动操作。

---

## 使用方法

1. 运行 `osg_gaussian_draw`（位于 `build/bin/` 目录）
2. 点击工具栏 **"打开文件"** 按钮，选择 `.splat` 或 `.ply` 文件
3. 按 **`C`** 键触发深度排序，获得正确的透明度混合效果
4. 鼠标操作：左键旋转、中键平移、滚轮缩放

---

## 代码集成示例

```cpp
#include "osg/GaussianDrawObj.h"

// 设置主相机（排序回调需要读取相机矩阵）
setMainCamera(viewer->getCamera());

// 创建高斯渲染对象
GaussianDrawObj* pObj = new GaussianDrawObj("model.splat");

// 获取节点加入场景图
osg::ref_ptr<osg::Node> gsNode = pObj->getNode();
root->addChild(gsNode);

// 触发排序
pObj->setDirty(true);
```

---

## 支持的文件格式

### `.splat` 格式

32 字节紧凑二进制格式，每个高斯点：

| 字段 | 类型 | 说明 |
|------|------|------|
| position | float[3] | 世界坐标 |
| scale | float[3] | 缩放系数 |
| color | uint8[4] | RGBA (0-255) |
| rotation | uint8[4] | 四元数 (0-255 映射到 -1~1) |

不支持球谐系数，仅使用固定颜色。

### `.ply` 格式

标准 PLY 格式（Binary/ASCII），支持属性：

- `x, y, z` — 位置
- `f_dc_0, f_dc_1, f_dc_2` — 0 阶球谐 DC 项（乘以 `SH_C0` + 0.5 得到基础色）
- `opacity` — 不透明度（经 sigmoid 变换）
- `scale_0, scale_1, scale_2` — 对数缩放（经 `exp()` 变换）
- `rot_0, rot_1, rot_2, rot_3` — 旋转四元数
- `f_rest_0` ~ `f_rest_44` — 1-3 阶球谐系数（可选，存在时启用视角相关颜色）

---

## 着色器说明

| 着色器 | 说明 |
|--------|------|
| `gaussian.vert` | 协方差矩阵 Jacobian 投影、2D 特征值分解定位 Quad、球谐函数求值 |
| `gaussian.frag` | `exp(-dot(pos,pos))` Gaussian 衰减，超过 2σ 丢弃 |
| `point.vert/frag` | 简单固定大小点渲染（备用管线） |

### 混合模式

```
RGB:   SRC=DST_ALPHA,  DST=ONE
Alpha: SRC=ZERO,       DST=ONE_MINUS_SRC_ALPHA
```

深度测试关闭，依赖 CPU 排序保证正确的 Back-to-front 渲染顺序。

---

## 联系作者

- **Email**: kallr@163.com
- **QQ**: 1600349661
