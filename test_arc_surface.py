#!/usr/bin/env python3
"""
测试脚本：生成弧面点云并测试 gRPC 服务
"""

import grpc
import numpy as np
import sys

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


def test_grpc_service(server_address='localhost:50051'):
    """
    测试 gRPC 服务的所有接口
    """
    print("=" * 60)
    print("开始测试 gRPC 规划服务")
    print("=" * 60)

    # 创建 gRPC 通道
    print(f"\n连接到服务器: {server_address}")
    channel = grpc.insecure_channel(server_address)
    stub = planner_pb2_grpc.PlannerServiceStub(channel)

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
            planning_timeout=10.0
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
