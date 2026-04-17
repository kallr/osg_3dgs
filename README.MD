# OSG 3D Gaussian Splatting 渲染器

基于 OpenSceneGraph (OSG) 的实时 3D Gaussian Splatting (3DGS) 渲染器。使用 GPU 加速的自定义 GLSL 着色器实现高质量高斯点云实时渲染。

---

## 功能特性

- 支持 `.splat` 和 `.ply` 两种点云格式
- GPU 加速的高斯泼溅渲染（自定义 GLSL 着色器）
- 球谐函数 (Spherical Harmonics) 支持，0–3 阶（每个高斯点 45 个 SH 系数），实现视角相关的颜色渲染
- 深度排序（基于 counting sort，65535 个桶），按键 `c` 触发排序
- 基于 Back-to-front 的 Alpha 混合渲染
- Qt5 图形界面，支持打开文件对话框
- OSG Trackball 交互式相机控制
- 自动场景适配

---

## 技术栈

| 组件 | 说明 |
|------|------|
| 语言 | C++17 |
| 3D 引擎 | OpenSceneGraph (OSG) ≥ 3.6.5 |
| GUI | Qt5（Widgets、Core、Gui、OpenGL） |
| 数学库 | GLM（header-only） |
| PLY 解析 | miniply（内置，MIT 协议） |
| 图形 API | OpenGL 4.3+ |
| 构建系统 | CMake 3.12+ |

---

## 目录结构

```
osg_3dgs/
├── CMakeLists.txt              # 顶层 CMake 配置
├── README.MD                   # 项目说明文档
├── osg/                        # 核心渲染模块
│   ├── CMakeLists.txt
│   ├── GaussianDrawObj.h/cpp   # 核心渲染类，加载和渲染高斯点云
│   ├── GraphicsWindowQt.h/cpp  # Qt + OSG 窗口集成
│   ├── MainWindow.h/cpp        # Qt 主窗口（文件打开等）
│   └── main.cpp                # 程序入口
├── shader/                     # GLSL 着色器
│   ├── gaussian.vert           # 高斯泼溅顶点着色器（含 SH 求值）
│   ├── gaussian.frag           # 高斯泼溅片段着色器
│   ├── point.vert              # 简单点渲染顶点着色器
│   └── point.frag              # 简单点渲染片段着色器
└── tools/                      # 工具库
    ├── miniply.h/cpp           # PLY 文件解析器（第三方）
    └── ...                     # 其他工具函数
```

---

## 核心类 API

### `GaussianDrawObj`

核心渲染类，负责高斯点云的加载、排序和渲染。

| 方法 | 说明 |
|------|------|
| `GaussianDrawObj(const std::string& filePath)` | 构造函数，传入 `.splat` 或 `.ply` 文件路径 |
| `osg::ref_ptr<osg::Node> getNode()` | 返回 OSG 场景节点，加入场景图即可渲染 |
| `setDirty()` | 标记需要重新排序 |
| `runSortAndUpdate()` | 执行高斯点深度排序 |

---

## 编译构建

### 前置依赖

- CMake 3.12+
- C++17 编译器
- Qt5（Widgets、Core、Gui、OpenGL）
- OpenSceneGraph ≥ 3.6.5
- GLM（header-only）

### 编译步骤

1. 从 GitHub 克隆代码：

   ```bash
   git clone <repo_url>
   cd osg_3dgs
   ```

2. 执行构建：

   ```bash
   mkdir build && cd build
   cmake ..
   make -j4
   ```

   > Windows 下可使用 Visual Studio 或 CMake GUI 进行构建。

3. 构建过程会自动将 `shader/` 目录复制到可执行文件所在目录，无需手动操作。

4. 确保系统已安装 Qt5 和 OSG 运行时库。

---

## 使用方法

1. 执行编译产物 `osg_gaussian_draw`。
2. 点击 **"打开文件"** 按钮，选择 `.splat` 或 `.ply` 文件。
3. 按键盘 **`c`** 键执行一次高斯点深度排序，即可获得正确的渲染效果。
4. 使用鼠标进行交互式相机控制（旋转、缩放、平移）。

---

## 代码示例

开发者可通过以下方式将高斯渲染集成到自己的 OSG 场景中：

```cpp
#include "GaussianDrawObj.h"

// 创建高斯渲染对象
std::string file = "C:\\model.splat";
GaussianDrawObj* pObj = new GaussianDrawObj(file);

// 获取 OSG 节点并加入场景
osg::ref_ptr<osg::Node> pGsNode = pObj->getNode();
root->addChild(pGsNode);
```

---

## 支持的文件格式

### `.splat` 格式

32 字节二进制格式，每个高斯点包含：位置、缩放、颜色、四元数旋转。

### `.ply` 格式

ASCII / Binary PLY 格式，支持球谐系数（`f_rest_0` ~ `f_rest_44`）。

---

## 着色器说明

| 着色器 | 说明 |
|--------|------|
| `gaussian.vert` / `gaussian.frag` | 主渲染管线，实现高斯泼溅和球谐函数求值 |
| `point.vert` / `point.frag` | 简单点渲染备用管线 |

---

## 联系作者

- **Email**: kallr@163.com
- **QQ**: 1600349661
