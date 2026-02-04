# gRPC Planner Service

基于 gRPC 的高性能路径规划服务，将 C++ OMPL 规划算法封装为可远程调用的服务。

## 项目概述

本项目将 PcsFmtTest.cpp 中的核心功能重构为一个 gRPC 服务，提供 4 个主要接口：

1. **UpdatePointCloud** - 更新点云数据
2. **ConvertToMesh** - 将拟合曲面转换为三角网格
3. **GetClosestPoint** - 获取曲面上的最近点
4. **PlanTrajectory** - 规划曲面上的轨迹

## 项目结构

```
grpc_planner/
├── planner.proto              # Protobuf 定义文件
├── planner_service.h          # 核心服务类头文件
├── planner_service.cpp        # 核心服务类实现
├── grpc_server.cpp            # gRPC 服务器实现
├── grpc_client.py             # Python 客户端示例
├── CMakeLists.txt             # CMake 构建文件
├── build.sh                   # 编译脚本
├── run_server.sh              # 服务器启动脚本
└── README.md                  # 本文档
```

## 依赖项

### C++ 依赖
- CMake >= 3.10
- C++17 编译器
- gRPC
- Protobuf
- OMPL
- PCL (Point Cloud Library)
- Eigen3
- MuJoCo Client

### Python 依赖
```bash
pip install grpcio grpcio-tools numpy
```

## 编译步骤

### 1. 生成 Python Protobuf 文件

```bash
cd /home/fraslab/zzx/Docker/ompl_docker/grpc_planner

# 生成 Python gRPC 代码
python3 -m grpc_tools.protoc \
    -I. \
    --python_out=. \
    --grpc_python_out=. \
    planner.proto
```

这将生成：
- `planner_pb2.py` - Protobuf 消息定义
- `planner_pb2_grpc.py` - gRPC 服务定义

### 2. 编译 C++ 服务器

```bash
# 创建构建目录
mkdir -p build
cd build

# 配置 CMake
cmake ..

# 编译
make -j$(nproc)
```

编译成功后会生成可执行文件：`grpc_planner_server`

## 使用方法

### 启动服务器

```bash
# 设置环境变量（可选）
export PCD_FILE="/path/to/your/pointcloud.pcd"
export MODEL_FILE="/path/to/your/mujoco_model.xml"
export GRPC_SERVER_ADDRESS="0.0.0.0:50051"

# 启动服务器
./build/grpc_planner_server
```

服务器默认监听 `0.0.0.0:50051`

### 使用 Python 客户端

```bash
# 运行示例客户端
python3 grpc_client.py
```

### Python 客户端 API 示例

```python
from grpc_client import PlannerClient
import numpy as np

# 连接到服务器
client = PlannerClient('localhost:50051')

# 1. 更新点云（可选）
points = np.random.rand(100, 3) * 2.0  # 100 个随机点
success, message = client.update_point_cloud(points)

# 2. 转换为网格
success, triangles, message = client.convert_to_mesh(resolution=64.0)
print(f"生成了 {len(triangles)} 个三角形")

# 3. 获取最近点
success, result, message = client.get_closest_point(1.0, 1.0, 1.0)
print(f"最近点: {result['closest_point']}")
print(f"UV 参数: {result['uv']}")

# 4. 规划轨迹
success, trajectory, planning_time, message = client.plan_trajectory(
    start_u=0.2, start_v=0.2,
    goal_u=0.8, goal_v=0.8,
    num_samples=5000,
    planning_timeout=20.0
)

if success:
    print(f"规划成功！用时 {planning_time:.3f} 秒")
    print(f"轨迹点数: {len(trajectory)}")
    for i, point in enumerate(trajectory):
        print(f"点 {i}: 位置={point['position']}, "
              f"姿态={point['orientation']}, "
              f"法向={point['normal']}")

# 关闭连接
client.close()
```

## 接口详细说明

### 1. UpdatePointCloud

更新点云数据并重新拟合曲面。

**输入:**
- `points`: 点云数据的扁平数组 [x1,y1,z1,x2,y2,z2,...]
- `num_points`: 点的数量

**输出:**
- `success`: 是否成功
- `message`: 响应消息

### 2. ConvertToMesh

将拟合的 NURBS 曲面转换为三角网格。

**输入:**
- `resolution`: 采样分辨率（默认 64）

**输出:**
- `success`: 是否成功
- `triangles`: 三角形列表，每个三角形包含 9 个值 [x1,y1,z1,x2,y2,z2,x3,y3,z3]
- `num_triangles`: 三角形数量
- `message`: 响应消息

### 3. GetClosestPoint

获取曲面上距离给定点最近的点。

**输入:**
- `x, y, z`: 输入点的三维坐标

**输出:**
- `success`: 是否成功
- `closest_x, closest_y, closest_z`: 最近点的坐标
- `u, v`: 最近点的 UV 参数
- `message`: 响应消息

### 4. PlanTrajectory

在曲面上规划从起点到终点的轨迹。

**输入:**
- `start_u, start_v`: 起点的 UV 参数
- `goal_u, goal_v`: 终点的 UV 参数
- `num_samples`: 采样点数量（默认 5000）
- `planning_timeout`: 规划超时时间（秒，默认 20.0）

**输出:**
- `success`: 是否成功
- `trajectory`: 轨迹点列表，每个点包含：
  - `x, y, z`: 位置坐标
  - `psi, theta`: 姿态角（偏航角和俯仰角）
  - `nx, ny, nz`: 法向量
- `planning_time`: 规划用时（秒）
- `num_trajectory_points`: 轨迹点数量
- `message`: 响应消息

## 性能特点

- **零拷贝传输**: 使用 Protobuf 二进制格式，最小化序列化开销
- **线程安全**: 使用互斥锁保护共享资源
- **高效规划**: 使用 PCS-FMT 算法进行路径规划
- **低延迟通信**: 支持 Unix Domain Socket（同机器）或 TCP（跨机器）

## 环境变量配置

| 变量名 | 说明 | 默认值 |
|--------|------|--------|
| `PCD_FILE` | 点云文件路径 | `/home/wsl/proj/skyvortex_mujoco/assets/NURBS.pcd` |
| `MODEL_FILE` | MuJoCo 模型文件路径 | `/home/wsl/proj/skyvortex_mujoco/scene.xml` |
| `GRPC_SERVER_ADDRESS` | 服务器监听地址 | `0.0.0.0:50051` |

## 故障排查

### 编译错误

1. **找不到 gRPC**: 确保已安装 gRPC 并设置了正确的 CMake 路径
2. **找不到 OMPL**: 检查 OMPL 是否正确安装
3. **找不到 PCL**: 安装 PCL 库 `sudo apt-get install libpcl-dev`

### 运行时错误

1. **无法加载点云文件**: 检查 `PCD_FILE` 环境变量是否正确
2. **无法加载 MuJoCo 模型**: 检查 `MODEL_FILE` 环境变量是否正确
3. **端口被占用**: 修改 `GRPC_SERVER_ADDRESS` 使用其他端口

## 与原始代码的对比

| 特性 | 原始 PcsFmtTest.cpp | gRPC 服务 |
|------|---------------------|-----------|
| 运行方式 | 单次执行 | 持续运行，循环接收请求 |
| 接口 | 命令行参数 | gRPC 远程调用 |
| 点云更新 | 需要重启程序 | 动态更新 |
| 跨语言支持 | 仅 C++ | 支持 Python、Go、Java 等 |
| 可视化 | 内置 GLFW 渲染 | 移除（由客户端负责） |
| 碰撞检测 | MuJoCo 全真碰撞 | MuJoCo 全真碰撞 |

## 下一步扩展

1. **添加流式接口**: 支持实时轨迹更新
2. **批量规划**: 支持一次规划多条轨迹
3. **参数配置接口**: 动态调整规划器参数
4. **状态查询接口**: 查询服务器状态和统计信息
5. **Docker 容器化**: 简化部署流程

## 许可证

与原项目保持一致。

## 联系方式

如有问题，请联系项目维护者。
