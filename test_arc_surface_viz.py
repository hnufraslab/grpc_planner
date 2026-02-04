#!/usr/bin/env python3
"""
测试脚本：生成弧面点云并测试 gRPC 服务（带可视化）
"""

import grpc
import numpy as np
import sys
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，不显示窗口
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# 导入生成的 protobuf 文件
import planner_pb2
import planner_pb2_grpc


def generate_arc_surface_points(num_u=20, num_v=20, radius=2.0, height=3.0):
    """
    生成弧面点云数据
    参数:
        num_u: u 方向采样点数
        num_v: v 方向采样点数
        radius: 弧面半径
        height: 弧面高度
    返回:
        points: 扁平化的点云数组 [x1,y1,z1,x2,y2,z2,...]
        num_points: 点的数量
    """
    print(f"生成弧面点云: {num_u}x{num_v} = {num_u*num_v} 个点")
    print(f"参数: radius={radius}, height={height}")

    points = []

    # 生成弧面参数化点云
    # u: [0, 1] -> 角度 [0, π]
    # v: [0, 1] -> 高度 [0, height]
    for i in range(num_u):
        u = i / (num_u - 1)  # [0, 1]
        theta = u * np.pi    # [0, π]

        for j in range(num_v):
            v = j / (num_v - 1)  # [0, 1]
            z = v * height       # [0, height]

            # 弧面方程: x = r*cos(θ), y = r*sin(θ), z = z
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)

            points.extend([x, y, z])

    num_points = num_u * num_v
    print(f"生成了 {num_points} 个点")
    print(f"点云范围: x=[{min(points[0::3]):.2f}, {max(points[0::3]):.2f}], "
          f"y=[{min(points[1::3]):.2f}, {max(points[1::3]):.2f}], "
          f"z=[{min(points[2::3]):.2f}, {max(points[2::3]):.2f}]")

    return points, num_points


def visualize_mesh(triangles, trajectory=None, title="网格和轨迹可视化"):
    """
    可视化三角网格和轨迹
    参数:
        triangles: 三角形列表，每个三角形是9个值的列表
        trajectory: 轨迹点列表（可选）
        title: 图表标题
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制三角网格
    if triangles and len(triangles) > 0:
        print(f"绘制 {len(triangles)} 个三角形...")

        # 转换三角形数据为顶点数组
        verts = []
        for tri in triangles:
            if len(tri.vertices) >= 9:
                v1 = [tri.vertices[0], tri.vertices[1], tri.vertices[2]]
                v2 = [tri.vertices[3], tri.vertices[4], tri.vertices[5]]
                v3 = [tri.vertices[6], tri.vertices[7], tri.vertices[8]]
                verts.append([v1, v2, v3])

        # 创建多边形集合
        poly = Poly3DCollection(verts, alpha=0.3, facecolor='cyan',
                               edgecolor='blue', linewidths=0.1)
        ax.add_collection3d(poly)

        # 计算边界
        all_points = np.array(verts).reshape(-1, 3)
        x_min, x_max = all_points[:, 0].min(), all_points[:, 0].max()
        y_min, y_max = all_points[:, 1].min(), all_points[:, 1].max()
        z_min, z_max = all_points[:, 2].min(), all_points[:, 2].max()

        print(f"网格范围: x=[{x_min:.2f}, {x_max:.2f}], "
              f"y=[{y_min:.2f}, {y_max:.2f}], z=[{z_min:.2f}, {z_max:.2f}]")

    # 绘制轨迹
    if trajectory and len(trajectory) > 0:
        print(f"绘制轨迹: {len(trajectory)} 个点")

        # 提取轨迹坐标
        traj_x = [p.x for p in trajectory]
        traj_y = [p.y for p in trajectory]
        traj_z = [p.z for p in trajectory]

        # 绘制轨迹线
        ax.plot(traj_x, traj_y, traj_z, 'r-', linewidth=3, label='轨迹')

        # 标记起点和终点
        ax.scatter([traj_x[0]], [traj_y[0]], [traj_z[0]],
                  c='green', s=200, marker='o', label='起点', edgecolors='black', linewidths=2)
        ax.scatter([traj_x[-1]], [traj_y[-1]], [traj_z[-1]],
                  c='red', s=200, marker='s', label='终点', edgecolors='black', linewidths=2)

        # 绘制法向量（每隔几个点绘制一次）
        step = max(1, len(trajectory) // 10)  # 最多显示10个法向量
        for i in range(0, len(trajectory), step):
            p = trajectory[i]
            # 法向量长度
            scale = 0.2
            ax.quiver(p.x, p.y, p.z,
                     p.nx * scale, p.ny * scale, p.nz * scale,
                     color='orange', arrow_length_ratio=0.3, linewidth=1.5)

    # 设置坐标轴
    ax.set_xlabel('X', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')

    # 设置相同的坐标轴比例
    if triangles and len(triangles) > 0:
        max_range = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
        mid_x = (x_max + x_min) / 2.0
        mid_y = (y_max + y_min) / 2.0
        mid_z = (z_max + z_min) / 2.0
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # 添加图例
    ax.legend(loc='upper right', fontsize=10)

    # 设置视角
    ax.view_init(elev=20, azim=45)

    plt.tight_layout()

    # 保存图片
    output_file = 'grpc_planner_visualization.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n可视化图片已保存: {output_file}")

    # 不显示窗口，直接关闭
    plt.close()


def test_grpc_service(server_address='localhost:50051'):
    """
    测试 gRPC 服务的所有接口
    """
    print("=" * 60)
    print("开始测试 gRPC 规划服务（带可视化）")
    print("=" * 60)

    # 创建 gRPC 通道
    print(f"\n连接到服务器: {server_address}")
    channel = grpc.insecure_channel(server_address)
    stub = planner_pb2_grpc.PlannerServiceStub(channel)

    mesh_triangles = None
    trajectory_points = None

    try:
        # ========================================
        # 测试 1: UpdatePointCloud
        # ========================================
        print("\n" + "=" * 60)
        print("测试 1: UpdatePointCloud - 更新点云数据")
        print("=" * 60)

        points, num_points = generate_arc_surface_points(
            num_u=30,
            num_v=30,
            radius=2.0,
            height=3.0
        )

        request = planner_pb2.UpdatePointCloudRequest(
            points=points,
            num_points=num_points
        )

        print(f"发送请求: {num_points} 个点")
        response = stub.UpdatePointCloud(request)

        print(f"响应: success={response.success}")
        print(f"消息: {response.message}")

        if not response.success:
            print("❌ 点云更新失败!")
            return False

        print("✅ 点云更新成功!")

        # ========================================
        # 测试 2: ConvertToMesh
        # ========================================
        print("\n" + "=" * 60)
        print("测试 2: ConvertToMesh - 转换为网格")
        print("=" * 60)

        request = planner_pb2.ConvertToMeshRequest(
            resolution=64
        )

        print("发送请求: resolution=64")
        response = stub.ConvertToMesh(request)

        print(f"响应: success={response.success}")
        print(f"三角形数量: {response.num_triangles}")
        print(f"消息: {response.message}")

        if response.success and response.num_triangles > 0:
            print(f"✅ 网格转换成功! 生成了 {response.num_triangles} 个三角形")
            mesh_triangles = response.triangles
            # 显示第一个三角形的坐标
            if len(response.triangles) > 0:
                tri = response.triangles[0]
                print(f"第一个三角形顶点:")
                print(f"  v1: ({tri.vertices[0]:.3f}, {tri.vertices[1]:.3f}, {tri.vertices[2]:.3f})")
                print(f"  v2: ({tri.vertices[3]:.3f}, {tri.vertices[4]:.3f}, {tri.vertices[5]:.3f})")
                print(f"  v3: ({tri.vertices[6]:.3f}, {tri.vertices[7]:.3f}, {tri.vertices[8]:.3f})")
        else:
            print("❌ 网格转换失败!")

        # ========================================
        # 测试 3: GetClosestPoint
        # ========================================
        print("\n" + "=" * 60)
        print("测试 3: GetClosestPoint - 查找最近点")
        print("=" * 60)

        # 测试点：在弧面附近
        test_x, test_y, test_z = 1.5, 0.5, 1.5

        request = planner_pb2.GetClosestPointRequest(
            x=test_x,
            y=test_y,
            z=test_z
        )

        print(f"查询点: ({test_x}, {test_y}, {test_z})")
        response = stub.GetClosestPoint(request)

        print(f"响应: success={response.success}")
        if response.success:
            print(f"最近点: ({response.closest_x:.3f}, {response.closest_y:.3f}, {response.closest_z:.3f})")
            print(f"UV 参数: u={response.u:.3f}, v={response.v:.3f}")

            # 计算距离
            dist = np.sqrt((test_x - response.closest_x)**2 +
                          (test_y - response.closest_y)**2 +
                          (test_z - response.closest_z)**2)
            print(f"距离: {dist:.3f}")
            print("✅ 最近点查找成功!")
        else:
            print(f"消息: {response.message}")
            print("❌ 最近点查找失败!")

        # ========================================
        # 测试 4: PlanTrajectory
        # ========================================
        print("\n" + "=" * 60)
        print("测试 4: PlanTrajectory - 规划轨迹")
        print("=" * 60)

        request = planner_pb2.PlanTrajectoryRequest(
            start_u=0.3,
            start_v=0.3,
            goal_u=0.5,
            goal_v=0.5,
            num_samples=5000,
            planning_timeout=20.0
        )

        print(f"起点 UV: ({request.start_u}, {request.start_v})")
        print(f"终点 UV: ({request.goal_u}, {request.goal_v})")
        print(f"采样数: {request.num_samples}, 超时: {request.planning_timeout}s")
        print("开始规划...")

        response = stub.PlanTrajectory(request)

        print(f"\n响应: success={response.success}")
        print(f"规划时间: {response.planning_time:.3f}s")
        print(f"轨迹点数: {response.num_trajectory_points}")
        print(f"消息: {response.message}")

        if response.success and response.num_trajectory_points > 0:
            print(f"✅ 轨迹规划成功!")
            trajectory_points = response.trajectory
            print(f"\n轨迹详情:")
            print(f"  起点: ({response.trajectory[0].x:.3f}, {response.trajectory[0].y:.3f}, {response.trajectory[0].z:.3f})")
            print(f"  终点: ({response.trajectory[-1].x:.3f}, {response.trajectory[-1].y:.3f}, {response.trajectory[-1].z:.3f})")
            print(f"  法向量(起点): ({response.trajectory[0].nx:.3f}, {response.trajectory[0].ny:.3f}, {response.trajectory[0].nz:.3f})")
            print(f"  姿态(起点): psi={response.trajectory[0].psi:.3f}, theta={response.trajectory[0].theta:.3f}")
        else:
            print("❌ 轨迹规划失败!")

        print("\n" + "=" * 60)
        print("所有测试完成!")
        print("=" * 60)

        # ========================================
        # 可视化
        # ========================================
        if mesh_triangles:
            print("\n" + "=" * 60)
            print("生成可视化...")
            print("=" * 60)
            visualize_mesh(mesh_triangles, trajectory_points, 
                          title="弧面网格和轨迹可视化")

        return True

    except grpc.RpcError as e:
        print(f"\n❌ gRPC 错误: {e.code()}")
        print(f"详情: {e.details()}")
        return False
    except Exception as e:
        print(f"\n❌ 异常: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        channel.close()


if __name__ == "__main__":
    server_address = sys.argv[1] if len(sys.argv) > 1 else 'localhost:50051'
    success = test_grpc_service(server_address)
    sys.exit(0 if success else 1)
