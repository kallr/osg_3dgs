# OSG 3D Gaussian Splatting 渲染器

基于 OpenSceneGraph (OSG) 的实时 3D Gaussian Splatting (3DGS) 核心库，附带 Qt5 测试查看器。核心库支持多种 3DGS 文件格式、球谐颜色、基于视角的 LOD、后台深度排序，以及 GLSL 高斯投影与透明混合渲染。

---

## 功能特性

- 支持 `.splat`、`.ply`、`.spz`、`.ksplat`、`.rad` 高斯点云格式
- 集成 GaussForge 多格式 IO，`.ply` / `.splat` 失败时保留本地解析器 fallback
- Qt5 图形界面 + OSG `TrackballManipulator` 相机交互
- 支持从命令行、工具栏 **Open** 或拖放文件打开模型
- GPU 实例化 Quad 渲染：每个高斯点作为一个 billboard/ellipse 实例绘制
- GLSL 4.30 顶点/片段着色器：协方差投影、椭圆定位、Gaussian falloff、Premultiplied Alpha
- 球谐函数 (Spherical Harmonics) 0–3 阶支持（每点最多 45 个 SH 系数）
- 后台线程 CPU Radix Sort 深度排序，视角变化时自动触发，按 **C** 可手动触发
- 普通高斯点云使用二叉 LOD 树，`.rad` 使用格式内置层级信息做 native LOD
- 自动场景包围盒适配，中键双击可重新适配视角
- 预留 OpenGL Compute Shader / SSBO 管线：深度计算、GPU bitonic sort、tile binning shader 已纳入工程；当前运行默认仍使用 CPU 排序，tile binning 默认关闭

---

## 技术栈

| 组件 | 说明 |
|------|------|
| 语言 | C++17 |
| 测试查看器 | Qt5 Widgets 与 QOpenGLWidget |
| 3D 引擎 | OpenSceneGraph (OSG) ≥ 3.6.5 |
| 图形 API | OpenGL 4.3+ / GLSL 430 core |
| 构建系统 | CMake 3.12+ |
| 多格式 IO | GaussForge（内置，支持 ply/splat/spz/ksplat 等读取） |
| PLY fallback | miniply（内置，MIT 协议，by Vilya Harvey） |
| 压缩支持 | zlib / Qt5::Zlib（用于 `.spz` 与 `.rad`） |

---

## 目录结构

```text
osg_3dgs/
├── CMakeLists.txt                 # CMake 构建配置、依赖查找、shader 复制
├── main.cpp                       # 程序入口；支持 argv[1] 自动加载文件
├── mainwindow.h/cpp               # Qt 主窗口与 Open 工具栏
├── include/osg_3dgs/GaussianModel.h # 外部 C++ 调用接口
├── osg/
│   ├── osgWindow.h/cpp            # Qt QOpenGLWidget ↔ osgViewer::Viewer 桥接
│   ├── GaussianModel.cpp          # 模型接口、拾取、编辑历史与 PLY 写出
│   ├── GaussianDrawObj.h/cpp      # 装配 OSG 节点、渲染状态、回调与资源
│   ├── SparkDatasetLoader.*       # GaussForge 与 PLY/SPLAT 回退加载
│   ├── SparkRadDatasetLoader.*    # RAD 元数据、启动页与取景范围
│   ├── SparkOsgDrawable.*         # RAD 流式加载、LOD 与排序交接
│   ├── SparkLodSelector.*         # RAD LOD 遍历和页面预算
│   ├── SparkLodWorker.*           # 后台 LOD 遍历
│   ├── SparkGpuResources.*        # TBO/SSBO 与部分上传
│   ├── SparkSortPass.*            # 后台 CPU 排序与 GPU 排序配置
│   └── GaussianLODTree.h/cpp      # 普通点云的屏幕空间 LOD 树
├── shader/
│   ├── gaussian.vert              # 高斯顶点着色器：协方差投影 + SH 求值
│   ├── gaussian.frag              # 高斯片段着色器：Gaussian falloff + premul alpha
│   ├── gaussian_depth.comp        # 计算深度 key（预留 compute 管线）
│   ├── gaussian_sort.comp         # GPU bitonic sort（预留 compute 管线）
│   ├── gaussian_tile_clear.comp   # tile binning 清理（默认关闭）
│   ├── gaussian_tile_bin.comp     # tile binning 分桶（默认关闭）
│   ├── gaussian_tile_raster.comp  # tile binning 光栅（默认关闭）
│   ├── point.vert
│   └── point.frag
├── 3rdparty/
│   ├── GaussForge/                # 多格式高斯点云 IO
│   └── spz/                       # Google SPZ 解码相关代码
└── tools/
    └── miniply.h/cpp              # PLY fallback 解析器
```

---

## 渲染管线

```text
加载文件
  ├─ .spz/.ksplat/.ply/.splat → GaussForge 优先
  ├─ .ply/.splat               → 本地 fallback 解析器
  └─ .rad                      → zlib 解压 + RAD 层级/属性解码
        ↓
转换为 MI_GaussianPoint[]
        ↓
构建数据纹理 / TBO：indexBuffer + dataBuffer + shBuffer
        ↓
准备 LOD：普通格式可选 GaussianLODTree，RAD 使用原生 childStart/childCount
        ↓
每帧更新与绘制
  ├─ RAD 后台准备页面和选择 LOD；渲染线程提交驻留页
  ├─ 相机或活动集合变化时请求后台 CPU Radix Sort
  └─ 更新索引 TBO 和实例数；RAD 在新排序就绪前保留旧绘制集合及其页面
        ↓
GPU instanced quad 渲染
  ├─ gaussian.vert 读取 TBO，投影 3D 协方差为 2D 椭圆
  ├─ 计算 0–3 阶 SH 视角相关颜色
  └─ gaussian.frag 输出 premultiplied alpha Gaussian
```

### TBO 数据布局

| TBO | 绑定点 | 内容 |
|-----|--------|------|
| `indexBuffer` | 0 | 当前绘制顺序索引；受 LOD 与深度排序更新 |
| `dataBuffer` | 1 | 每点 4 个 `vec4`：`position.xyz + alpha`、`scale.xyz + premul.r`、`rotation.xyz + premul.g`、`rotation.w + 0 + 0 + premul.b` |
| `shBuffer` | 2 | 每点 12 个 `vec4`（45 个 SH float + padding），按通道分组 `[R0..R14, G0..G14, B0..B14]` |
| `packedDataBuffer` | 3 | `RGBA32UI`；PackedSplat 每点 1 个 texel，ExtSplat 每点 2 个 texel 并保留 F32 中心 |
| `covSplatBuffer` | 4 | 可选直接协方差数据，每点 2 个 `vec4` |

---

## 核心模块

### `osg_gaussian_draw`：Qt 测试程序

- `MainWindow` 负责工具栏、文件对话框和命令行加载。
- `GraphicsWindowQt` 负责 OSG 视图、相机操作、选中标记和包围框显示，使用 `QSaveFile` 提交保存。
- 高斯模型通过 `GaussianModel` 加载；其他格式继续交给 `osgDB::readNodeFile`。

### `osg_3dgs::core`：高斯核心库

公开接口位于 [`GaussianModel.h`](include/osg_3dgs/GaussianModel.h)。它提供加载、OSG 节点创建、包围框和统计查询、点选与框选、按 ID 删除与撤销，以及二进制 PLY 写出。`GaussianDrawObj` 和 Spark 组件在库内负责排序、LOD、流式加载与渲染。

### `SparkSortPass`

CPU 排序线程按活动索引读取位置快照，用 8-bit radix sort（4 轮，32-bit key）生成从远到近的绘制顺序。RAD 默认按相机距离排序，普通格式默认按视线深度排序。

### `GaussianLODTree`

对非 `.rad` 点云构建二叉层级。遍历时根据节点屏幕空间半径决定是否展开，并可通过 `maxActiveCount` 控制绘制点数上限。

---

## 支持的文件格式

### `.splat`

32 字节紧凑二进制格式，每个高斯点：

| 字段 | 类型 | 说明 |
|------|------|------|
| position | `float[3]` | 世界坐标 |
| scale | `float[3]` | 线性空间缩放 |
| color | `uint8[4]` | RGBA，映射到 0–1 |
| rotation | `uint8[4]` | 四元数分量映射到 -1–1 后归一化 |

`.splat` 不包含高阶 SH，使用固定颜色。

### `.ply`

支持常见 3DGS PLY 属性：

- `x, y, z`：位置
- `f_dc_0, f_dc_1, f_dc_2`：0 阶 SH / DC 颜色
- `opacity`：不透明度（sigmoid）
- `scale_0, scale_1, scale_2`：对数缩放（读取后 `exp`）
- `rot_0, rot_1, rot_2, rot_3`：旋转四元数
- `f_rest_0` ~ `f_rest_44`：1–3 阶 SH 系数（可选）

### `.spz`

Google 压缩高斯格式。需要 zlib 或 Qt5::Zlib 支持；由 GaussForge + SPZ 解码代码处理。

### `.ksplat`

分块/压缩高斯格式，通过 GaussForge 读取并转换为统一 IR。

### `.rad`

RAD 层级压缩格式。读取器解析 `RAD0` header、chunk 表、属性编码，并使用 zlib raw inflate 解压 chunk。当前实现面向 Spark RAD：

- 支持 Spark 常见属性编码：`center=f32/float`、`alpha=f16`、`rgb=r8_delta`、`scales=ln_0r8`、`orientation=oct88r8`、`sh1=s8`、`child_count=u16`、`child_start=u32`
- 初始加载 root/首批 resident chunk，随后根据视角和 native LOD 层级按优先级请求缺失 chunk
- 使用 `_radGlobalToLocal` / `_radLocalToGlobal` / `_radSlotToChunk` / `_radChunkSlots` 维护 global splat、resident slot、chunk 之间的映射
- chunk 解码只在主/update 路径应用到 resident slot；loader 线程只读取 raw bytes
- 页面替换前先提交完整的细/粗层覆盖，确保 active/depth indices 不再引用待淘汰页；随后清理该页的 global/local 映射并复用物理页
- active set 只绘制 resident 且当前 LOD 选中的 local slot，排序也只针对 bounded active set，而不是全量 RAD 数据集
- RAD native LOD 使用 candidate 总量计数：已输出 candidate 与 frontier/fallback candidate 都计入 `OSG_3DGS_RAD_MAX_ACTIVE`，避免低 cap 下继续过度展开
- 缺失 child chunk 时保留 coarse parent fallback 可见，同时继续按优先级 queue missing chunk，避免 partial child split 造成空洞/闪烁
- RAD active-set commit 带 hysteresis / min-change gate，减少轻微相机移动造成的 draw index reshuffle 与 CPU sort churn
- 标准遍历将祖先、已驻留子页和缺失子页一并计入页面预算；整组子节点所需页面放不下时保留粗父节点，不发出无法容纳的子页请求
- 请求队列按当前视角的所需页面及优先级替换；取消旧请求后拒绝迟到的数据，避免历史优先级长期占用缓存
- active/depth 页面成员缓存只在集合成员或槽位映射变化时重算；单纯深度排序不再扫描全部点
- LOD 评分使用独立紧凑节点记录，只读取中心、尺度、alpha 和 LOD 尺寸，避免从完整 Gaussian 的旋转和 SH 数据跨缓存行取值；结果与完整记录路径保持一致
- 缺页的 canonical RAD 子树按页范围保留粗父节点；异常或部分映射自动回退逐 child 检查
- RAD shader 默认启用 Spark 风格 blur compensation 和 radius clamp；LOD opacity inflate 默认关闭。RAD SH 默认启用（当 `maxSh > 0` 时），可通过 `OSG_3DGS_RAD_DISABLE_SH=1` 关闭

RAD 分页路径默认使用 ExtSplat 保留 F32 中心坐标，默认驻留预算为 `6291456` 槽（按每页 65536 槽折算为 96 页）。移动和静止的点数目标均为 250 万。相机静止 350 ms 后可批量应用请求页；若显式配置了更低的移动预算，此时也会切换到静止预算。350 ms 是触发延迟。点数与页面预算都可能保留粗层覆盖，达到点数上限或缺页请求归零并不表示已加载全部源数据。

> `.spz` 与 `.rad` 均依赖 zlib。没有 zlib 时可在配置阶段禁用这两种格式。

---

## 编译构建

### 前置依赖

- CMake 3.12+
- C++17 编译器（MSVC 2017+ / GCC 7+ / Clang 5+）
- OpenGL 开发库；Qt5 Widgets 用于测试查看器
- OpenSceneGraph ≥ 3.6.5
- zlib 或带 zlib 头文件的 `Qt5::Zlib`（默认构建所需，用于 `.spz` 和 `.rad`）

### OSG 路径

可通过 CMake 变量或环境变量指定 OSG：

- `OSG_ROOT`：OSG 安装根目录，期望包含 `include/`、`debug/lib/`、`release/lib/`
- 或使用 `OSG_DIR` / 系统默认搜索路径

### Windows 示例

```bash
cmake -S . -B build -DOSG_ROOT=/path/to/osg
cmake --build build --config Release
```

默认生成两个目标：

```text
build/bin/osg_gaussian_draw.exe  # Qt 测试程序
build/lib/osg_3dgs_core.lib      # C++ 静态库
```

当前 CMake 的 OSG 链接配置使用 Windows `.lib` 文件名；其他平台需要先适配链接配置。构建后会自动把 `shader/` 复制到可执行文件所在目录。

只构建核心库时设 `-DOSG_3DGS_BUILD_GUI=OFF`。安装独立 zlib 可启用 SPZ/RAD 且无需 Qt；没有独立 zlib 时可使用 Qt 自带的 zlib，或设 `-DOSG_3DGS_ENABLE_ZLIB_FORMATS=OFF` 禁用这两种格式。

### 在其他 C++ 项目中使用

在宿主项目中提供 OSG 和 zlib 的配置，再添加本仓库并链接库目标：

```cmake
set(OSG_3DGS_BUILD_GUI OFF CACHE BOOL "Build the Qt test viewer")
add_subdirectory(path/to/osg_3dgs osg_3dgs)
target_link_libraries(my_app PRIVATE osg_3dgs::core)
```

渲染时，把 `shader/` 目录随宿主程序部署，并在加载时传入它的路径。用宿主相机创建节点并将节点添加到场景中，例如：

```cpp
#include <osg_3dgs/GaussianModel.h>

auto model = osg_3dgs::GaussianModel::load(modelPath, shaderDirectory);
if (model) {
    if (auto node = model->createSceneNode(*camera))
        sceneRoot->addChild(node.get());
}
// 宿主保存 model，在需要渲染该模型的整个期间保持它存活。
```

每个模型实例绑定一台 OSG 相机。建议宿主使用单线程 OSG Viewer；在编辑和销毁期间不要并发遍历该模型。销毁模型会清空它的场景节点，并从相机移除它自己的绘制回调。

仅加载或编辑数据时可省略着色器路径，也无需创建窗口或 OpenGL 上下文。`pickPoint` 和 `pickRectangle` 返回稳定的源点 ID，可传给 `deletePoints`；`undoDeletion` 返回恢复的 ID。用 `canEdit` 检查模型是否可编辑（目前 RAD 和启用 LOD 的模型不可编辑）。`writePly` 接收二进制 `std::ostream`，由调用方负责文件打开和保存提交。

---

## 使用方法

本分支不附带模型。请使用自己的高斯模型文件，示例中的 `path/to/model` 表示本地路径。

### GUI 打开

启动 `osg_gaussian_draw` 后，点击工具栏 **Open** 或按 `Ctrl+O`，选择 `.ply` / `.splat` / `.spz` / `.ksplat` / `.rad` 文件。也可以把单个本地模型文件拖到窗口里。打开失败时，状态栏会提示，原模型仍保留在视图中。

标题栏显示当前文件名；状态栏显示模型点数，RAD 模型还会显示当前驻留的块数。

### 命令行打开

```bash
./osg_gaussian_draw path/to/model.ply
```

Windows：

```bash
build/bin/osg_gaussian_draw.exe path/to/model.splat
```

### 交互操作

| 操作 | 说明 |
|------|------|
| 左键拖拽 | 旋转视图 |
| 中键或右键拖拽 | 平移视图，旋转中心跟随鼠标移动 |
| `Shift` + 中键或右键拖拽 | 快速平移，贴近模型时也能大幅移动 |
| `Ctrl` + 中键或右键拖拽 | 小幅平移，便于对准细节 |
| 滚轮或触控板垂直滚动 | 拉近或拉远视图 |
| `Ctrl` + 滚轮 | 小步调整远近 |
| `Shift` + 滚轮 | 沿视线前后移动，可越过旋转中心；同时按 `Ctrl` 可小步移动 |
| `F` 或中键双击 | 保持当前观察方向，将模型重新适配到视口 |
| `B`（View → Show Bounding Box） | 显示或隐藏用于取景的模型包围框；删除和撤销后按剩余点更新，RAD 使用取景范围 |
| `Home` | 恢复模型打开时的相机视角 |
| `Ctrl+Shift+S` | 保存视图截图及同名相机 JSON |
| `F11` / `Esc` | 切换全屏 / 退出全屏 |
| `C` | 手动请求一次深度排序 |
| `S`（OSG StatsHandler） | OSG 统计信息开关 |

### 选择和删除高斯点

PLY、SPLAT、SPZ 和 KSPLAT 模型可以编辑。点击工具栏 **Select Points** 或按 `V` 进入选择模式；再次按 `V` 恢复左键旋转。编辑时仍可用中键或右键平移、滚轮缩放。

| 操作 | 说明 |
|------|------|
| 左键点击 | 选择点击位置附近、中心最靠前的一个点 |
| 左键拖动 | 框选所有投影中心落在框内的点，包括被遮挡的点 |
| `Shift` + 点击或框选 | 追加到当前选择 |
| `Ctrl` + 点击或框选 | 切换命中点的选择状态 |
| `Ctrl+D` | 清空选择 |
| `Delete` | 删除选中的点 |
| `Ctrl+Z` | 撤销上一次删除并选中恢复的点 |

青色标记表示选中的高斯点中心；大量选中时会按比例抽样显示标记，状态栏仍显示实际选中数和已删除数。删除后按 `F` 可按剩余点重新取景。

通过 **File → Save Model As…** 或 `Ctrl+S` 将剩余点另存为二进制高斯 `.ply` 文件。新文件可重新打开，保留点的位置、缩放、旋转、透明度及球谐颜色。另存不会改写源文件；未另存的修改只保留在当前会话。所有点均被删除时不能另存，RAD 模型暂时只读。

---

## RAD / Spark 运行参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `OSG_3DGS_RAD_MAX_RESIDENT` | `6291456` | RAD 驻留 slot 上限，默认 96 页；允许范围 `65536`–`8388608`，最多 128 页，增大将增加内存与显存占用 |
| `OSG_3DGS_RAD_MAX_ACTIVE` | 未设置 | 显式设置后固定移动和静止两种状态的点数预算，范围 `10000`–`4000000`；优先于下面两个预算参数；未设置时默认移动和静止均为 250 万 |
| `OSG_3DGS_RAD_MOVING_MAX_ACTIVE` | `2500000` | 移动预算，范围 `10000`–静止预算；设为 `600000` 恢复之前的自适应降细节模式，减少移动时的排序/渲染负载；固定 `MAX_ACTIVE` 优先 |
| `OSG_3DGS_RAD_IDLE_MAX_ACTIVE` | `2500000` | 静止细化点数目标，范围 `600000`–`4000000`；设置固定 `MAX_ACTIVE` 后此参数不生效 |
| `OSG_3DGS_RAD_IDLE_DELAY_MS` | `350` | 静止批量加载的等待时间，范围 `0`–`5000` ms；移动预算小于静止预算时也控制升预算时机。默认两者相同，细节无需等此延迟才开始选择 |
| `OSG_3DGS_RAD_VIEWPORT_WEIGHT` | `0.1` | 完全屏外节点的最低细化权重，范围 `0.01`–`1`；保留粗层覆盖，边缘大节点保持原优先级。设 `1` 恢复原圆锥优先级；非对称/正交投影及 GPU 位移等路径自动回退 |
| `OSG_3DGS_RAD_VIEWPORT_STATS` | 关闭 | 设 `1` 在后台遍历完成后统计点中心位于屏内、屏外和背后的数量；会增加诊断扫描，不等同于完整高斯椭圆的可见数量 |
| `OSG_3DGS_RAD_INITIAL_CHUNKS` | `1` | 初始同步加载 chunk 数量，通常保持 root chunk 即可 |
| `OSG_3DGS_RAD_LOAD_CHUNKS_PER_UPDATE` | `2` | 每次 LOD update 最多应用的 streamed raw chunks；在 CPU sort 压力较高或 active 很大时内部还会自动收缩到 `1` |
| `OSG_3DGS_RAD_IDLE_LOAD_BATCH` | `32` | 静止时累计加载最多 32 个已请求分块再遍历 LOD；每帧上传上限不变，视角变化、队列耗尽或缓存阻塞会提前结束批次。范围 `1`–`32`，设为 `16` 恢复之前批次，设为 `1` 每轮上传后遍历 |
| `OSG_3DGS_RAD_PREPARE_WORKERS` | `2` | 后台读取、解压、转换及打包的并发数，范围 `1`–`3`；同步准备模式默认 `1`，设 `1` 对照单线程 |
| `OSG_3DGS_RAD_READY_CHUNKS` | `4` | 就绪页加当前有效准备任务的容量，范围 `1`–`8`；同步准备模式默认 `8` |
| `OSG_3DGS_RAD_RETARGET_ON_STOP` | `1` | 稳定 80 ms 后取消一次旧视角任务，让当前视角尽早开始；连续移动不反复取消。会失去该任务未发布的预加载信息，最终恢复并非每次更快；设 `0` 恢复等待旧任务 |
| `OSG_3DGS_RAD_ACTIVE_PAGE_SUMMARY` | `1` | 后台标准遍历同时记录实际 active 页集合；提交版本和驻留版本一致时复用，避免主线程全量索引扫描。旧显示页仍独立保护；设 `0` 对照原扫描路径 |
| `OSG_3DGS_RAD_CLEAR_EVICTED_POINTS` | `0` | 分页回收默认只使槽映射/元数据失效，随后由新页覆盖点数据；设 `1` 恢复整页 CPU 点清空作对照。未映射槽内容无效，不能以透明度判断是否驻留 |
| `OSG_3DGS_RAD_RESUME_TRAVERSAL` | `0` | 实验性缺页前缀续算；严格校验视角、参数和依赖页后跳过已完成工作。局部遍历更快，但完整刷新收益尚不稳定，默认关闭 |
| `OSG_3DGS_RAD_TERMINAL_RANGES` | `0` | 实验性按连续页范围合并叶节点查询；选点结果一致，微测有局部收益，真实完整刷新尚无稳定改善 |
| `OSG_3DGS_RAD_WAIT_SORT_HANDOFF` | `1` | 部分页面已上传但剩余页被旧显示占用时，等待已请求的 CPU 排序交接后继续上传，避免无效遍历；视角、预算或投影变化退出等待，设 `0` 对照旧调度 |
| `OSG_3DGS_RAD_SYNC_LOD` | `0` | 默认后台遍历；设为 `1` 用同步遍历做对照诊断 |
| `OSG_3DGS_RAD_PRIORITY_CACHE` | `1` | 复用同相机、同页面的 LOD 优先级，减少静止细化的重复计算；96 页最多额外约 30 MiB CPU 内存，设 `0` 关闭；选点及最终画质不变 |
| `OSG_3DGS_RAD_LOD_COMPACT_RECORDS` | `1`（RAD） | LOD 评分读取 32 字节紧凑节点记录，减少完整 Gaussian 的 CPU 缓存访问；96 页约增加 192 MiB CPU 记录内存；设 `0` 回退完整点记录路径，选点及最终画质不变 |
| `OSG_3DGS_RAD_FOUR_ARY_HEAP` | `1` | 标准 LOD 使用四叉优先队列，设 `0` 对照旧二叉堆；同优先级节点的选择顺序可能不同 |
| `OSG_3DGS_RAD_BYPASS_TERMINAL_HEAP` | `1` | 已有页面优先级覆盖的有限分数叶节点直接进入结果，避免无意义堆操作；设 `0` 关闭，点数和页面预算不变 |
| `OSG_3DGS_RAD_SCORING_CACHE` | `0` | 实验性按需评分输入缓存，记录位置、大小和静态支持半径；96 页额外约占用 126 MiB CPU 内存，默认关闭 |
| `OSG_3DGS_RAD_SYNC_PREPARE` | `0` | 默认后台解压、转换及打包；设为 `1` 在主线程准备数据做对照。GPU 驻留预算不变，后台准备会额外占用 CPU 内存 |
| `OSG_3DGS_MIN_SORT_INTERVAL_MS` | `RAD 默认 8` / 其他 `0` | 最小排序间隔（毫秒）；RAD 默认启用轻度节流，避免相机微动时每帧都重排几十万 splat |
| `OSG_3DGS_RAD_LOD_LEVEL` | `0.786` | 用户级 LOD；数值越大细化阈值越小，实际细节仍受点数和页面预算限制 |
| `OSG_3DGS_RAD_LOD_QUALITY` | unset / 约 `0.08` | 兼容旧脚本的底层 pixel-scale 阈值乘子；设置后优先于 `OSG_3DGS_RAD_LOD_LEVEL`。数值越小质量越高/加载越多，数值越大速度越高/更粗糙 |
| `OSG_3DGS_RAD_ACTIVE_STABLE_FRAMES` | `GPU sort 默认 5` / 其他 `4` | 小变化候选等待实际渲染帧数，期间保留已完成结果，不重复全量遍历；相机或数据改变则作废候选 |
| `OSG_3DGS_RAD_ACTIVE_MIN_CHANGE_ABS` | `8192` | RAD active-set 绝对变化阈值；超过后立即 commit |
| `OSG_3DGS_RAD_ACTIVE_MIN_CHANGE_FRAC` | `GPU sort 默认 0.08` / 其他 `0.04` | RAD active-set 相对变化阈值；按当前 active 数量计算，超过后立即 commit |
| `OSG_3DGS_RAD_DISABLE_SORT` | unset | 设为 `1` 时禁用 RAD sort，直接按 active order 绘制 |
| `OSG_3DGS_RAD_FORCE_CPU_SORT` | unset | 设为 `1` 时强制 CPU sort；当前默认即后台 CPU radix sort |
| `OSG_3DGS_SORT_RADIAL` | RAD `1`，普通格式 `0` | RAD 默认与 Spark 一致，按相机距离排序透明高斯；设为 `0` 切回视线深度排序 |
| `OSG_3DGS_RAD_ENABLE_GPU_SORT` | unset | 设为 `1` 尝试实验性 GPU sort，容量上限 `524288`，不适合默认 250 万点细化预算 |
| `OSG_3DGS_RAD_DISABLE_SH` | unset | 默认启用 RAD SH（当 `maxSh > 0` 时）；设为 `1` 后禁用 RAD SH 以提高交互性能 |
| `OSG_3DGS_SUPPORT_RADIUS_SCALE` | RAD `1.0`，普通格式 `0.9` | splat 支撑半径缩放 |
| `OSG_3DGS_MAX_SCREEN_RADIUS` | RAD `512`，普通格式 `40` | 单个 splat 最大屏幕半径 |
| `OSG_3DGS_BLUR_AMOUNT` | `0.3` | Spark 风格 2D covariance anti-alias blur，降低可提高局部清晰度 |
| `OSG_3DGS_MIN_ALPHA` | `0.5/255` | fragment discard 的最小 alpha 阈值 |
| `OSG_3DGS_ENABLE_BLUR_COMPENSATION` | RAD 默认开 | 非 RAD 可用该变量开启 Spark blur compensation |
| `OSG_3DGS_SPARK_RADIUS_CLAMP` | RAD 默认开 | 非 RAD 可用该变量开启 Spark 风格半径 clamp |
| `OSG_3DGS_LOD_INFLATE` | 默认关 | 设为 `1` 开启 LOD opacity inflate |
| `OSG_3DGS_PACKED_ENCODING` | RAD 分页默认 `2` | `2` 为 ExtSplat，保留 F32 中心；`1` 为 PackedSplat |

---

## 着色器与混合

| 着色器 | 说明 |
|--------|------|
| `gaussian.vert` | 读取高斯数据，计算 3D→2D 协方差投影，求 2D 椭圆轴向与 SH 颜色 |
| `gaussian.frag` | 根据椭圆局部坐标计算 `exp(-0.5 * d²)` 衰减并输出 premul alpha |
| `point.vert/frag` | 备用点渲染 shader |
| `gaussian_depth.comp` | 预留：计算深度 key |
| `gaussian_sort.comp` | 预留：GPU bitonic sort |
| `gaussian_tile_*.comp` | 预留：tile binning / raster pipeline，当前默认关闭 |

Premultiplied Alpha 混合：

```text
SRC_FACTOR = GL_ONE
DST_FACTOR = GL_ONE_MINUS_SRC_ALPHA
```

深度测试关闭，透明关系依赖 back-to-front 排序。

RAD 默认按与相机的距离排序。需要按视线方向的深度排序时，可设 `OSG_3DGS_SORT_RADIAL=0`。

---

## 当前实现说明与限制

- Tile raster 是实验路径。RAD 需同时设置 `OSG_3DGS_RAD_ENABLE_GPU_SORT=1`、`OSG_3DGS_ENABLE_TILE_BINNING=1` 和 `OSG_3DGS_ENABLE_TILE_RASTER=1`，且相关 shader 可加载；默认走 CPU 排序和实例化 quad。
- 通过 OSG 状态配置非空 Splat Edit 位移时，实例化 quad 使用编辑后的中心，排序键仍使用原始位置。位移改变深度顺序时可能出现错误的透明混合。
- RAD 使用 bounded active-set CPU radix sort，并带有 pending sort coalescing；同一 active-set version 已有未完成排序时不会重复快照/提交。
- RAD active-set commit 带 hysteresis 与最小变化阈值，小幅 camera-only candidate 会等待稳定帧，明显变化、首帧和 chunk 应用后的完整 candidate 会快速提交。
- RAD 始终使用文件内层级信息做运行时 LOD。普通格式默认禁用 LOD，可用 `OSG_3DGS_ENABLE_LOD=1` 启用。
- RAD streaming 已实现 dirty data/SH slot range 上传；draw index 由 LOD active set 与排序结果更新，不在 chunk load 后用旧 `depthIndex` 直接全量刷新。
- RAD traversal 区分完整 candidate 与 fallback candidate；缺失 child 时保留 coarse parent，可继续 queue chunk，invalid range / active cap / invalid slot 不会盲目替换已有稳定 active set。
- 标准遍历只保留本轮预算内依赖页面的保护集合，并将缺页纳入预算；拒绝展开的分支保留粗父节点。安全提交后释放旧显示页，再加载新视角所需页面。
- `missingStops=0` 表示本轮准入页面内没有缺失子节点；若 `residentCapStops>0`，仍有分支因缓存预算而保持粗层。
- GPU sort 容量上限为 `524288`；超出容量时只绘制容量范围内的活动索引。默认 CPU 排序支持更高的点数预算。
- Release 构建已验证通过；MSVC 可能仍报告 `C4819` 源文件编码警告，当前属于非阻塞清理项，可后续统一保存为 UTF-8。
- RAD loader 采用轻量 JSON 字符串解析，不是完整 JSON parser；需要输入格式符合当前 reader 预期。
- 本项目关闭深度测试并依赖透明排序；极端交叉透明场景仍可能出现近似误差。

---

## 联系作者

- **Email**: kallr@163.com
- **QQ**: 1600349661
