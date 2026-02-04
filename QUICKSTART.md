# 快速启动指南

## 一键编译和运行

### 1. 编译项目

```bash
cd /home/fraslab/zzx/Docker/ompl_docker/grpc_planner
./build.sh
```

### 2. 启动服务器

```bash
./run_server.sh
```

### 3. 运行客户端（新终端）

```bash
cd /home/fraslab/zzx/Docker/ompl_docker/grpc_planner
python3 grpc_client.py
```

## 自定义配置

### 使用自定义点云和模型文件

```bash
export PCD_FILE="/path/to/your/pointcloud.pcd"
export MODEL_FILE="/path/to/your/mujoco_model.xml"
export GRPC_SERVER_ADDRESS="0.0.0.0:50051"

./run_server.sh
```

### 修改服务器端口

```bash
export GRPC_SERVER_ADDRESS="0.0.0.0:8080"
./run_server.sh
```

客户端连接时也需要修改：

```python
client = PlannerClient('localhost:8080')
```

## 常见问题

### Q: 编译时找不到 gRPC

**A:** 安装 gRPC：

```bash
# Ubuntu/Debian
sudo apt-get install -y libgrpc++-dev libprotobuf-dev protobuf-compiler-grpc

# 或从源码编译
git clone --recurse-submodules -b v1.50.0 https://github.com/grpc/grpc
cd grpc
mkdir -p cmake/build
cd cmake/build
cmake ../..
make -j$(nproc)
sudo make install
```

### Q: Python 客户端报错 "No module named 'grpc'"

**A:** 安装 Python gRPC 库：

```bash
pip3 install grpcio grpcio-tools numpy
```

### Q: 服务器启动失败，提示找不到点云文件

**A:** 检查环境变量或使用绝对路径：

```bash
export PCD_FILE="/absolute/path/to/your/pointcloud.pcd"
./run_server.sh
```

### Q: 如何在 Docker 容器中运行？

**A:** 确保容器内已安装所有依赖，然后：

```bash
# 在容器内
cd /home/fraslab/zzx/Docker/ompl_docker/grpc_planner
./build.sh
./run_server.sh
```

如果需要从宿主机访问，确保端口映射正确：

```bash
docker run -p 50051:50051 your_image
```

## 接口测试示例

### 测试 1: 获取最近点

```python
from grpc_client import PlannerClient

client = PlannerClient('localhost:50051')
success, result, message = client.get_closest_point(1.0, 1.0, 1.0)

if success:
    print(f"最近点: {result['closest_point']}")
    print(f"UV: {result['uv']}")

client.close()
```

### 测试 2: 转换为网格

```python
from grpc_client import PlannerClient

client = PlannerClient('localhost:50051')
success, triangles, message = client.convert_to_mesh(resolution=64.0)

if success:
    print(f"生成了 {len(triangles)} 个三角形")
    # 保存为文件或可视化
    import numpy as np
    np.save('mesh_triangles.npy', np.array(triangles))

client.close()
```

### 测试 3: 规划轨迹

```python
from grpc_client import PlannerClient

client = PlannerClient('localhost:50051')

# 规划从 (0.2, 0.2) 到 (0.8, 0.8) 的轨迹
success, trajectory, planning_time, message = client.plan_trajectory(
    start_u=0.2, start_v=0.2,
    goal_u=0.8, goal_v=0.8,
    num_samples=5000,
    planning_timeout=20.0
)

if success:
    print(f"规划成功！用时 {planning_time:.3f} 秒")
    print(f"轨迹点数: {len(trajectory)}")

    # 保存轨迹
    import json
    with open('trajectory.json', 'w') as f:
        json.dump(trajectory, f, indent=2)

client.close()
```

## 性能优化建议

1. **增加采样点数**: 提高规划质量，但会增加计算时间
   ```python
   num_samples=10000  # 默认 5000
   ```

2. **调整超时时间**: 对于复杂场景，增加超时时间
   ```python
   planning_timeout=30.0  # 默认 20.0 秒
   ```

3. **使用 Unix Domain Socket**: 如果客户端和服务器在同一台机器上
   ```bash
   export GRPC_SERVER_ADDRESS="unix:///tmp/planner.sock"
   ```

## 下一步

- 查看 [README.md](README.md) 了解详细文档
- 查看 [grpc_client.py](grpc_client.py) 了解完整的客户端 API
- 修改 [planner_service.cpp](planner_service.cpp) 自定义规划器参数
