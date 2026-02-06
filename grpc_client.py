#!/usr/bin/env python3
"""
Python Client for gRPC Planner Service

This client demonstrates how to call the 5 interfaces provided by the planner service:
1. UpdatePointCloud - Update point cloud data
2. ConvertToMesh - Convert fitted surface to mesh triangles
3. GetClosestPoint - Get closest point on surface
4. PlanTrajectory - Plan trajectory on surface
5. GetSurfacePoint - Get surface point and normal at UV coordinates

Usage:
    python grpc_client.py
"""

import grpc
import numpy as np
import planner_pb2
import planner_pb2_grpc
import sys
import time


class PlannerClient:
    """Client for interacting with the gRPC Planner Service"""

    def __init__(self, server_address='localhost:50051'):
        """
        Initialize the client

        Args:
            server_address: Address of the gRPC server (default: localhost:50051)
        """
        self.channel = grpc.insecure_channel(server_address)
        self.stub = planner_pb2_grpc.PlannerServiceStub(self.channel)
        print(f"Connected to server at {server_address}")

    def close(self):
        """Close the gRPC channel"""
        self.channel.close()

    def update_point_cloud(self, points):
        """
        Interface 1: Update point cloud data

        Args:
            points: numpy array of shape (N, 3) containing point coordinates

        Returns:
            bool: True if successful
            str: Response message
        """
        print("\n=== Calling UpdatePointCloud ===")

        if isinstance(points, np.ndarray):
            if len(points.shape) != 2 or points.shape[1] != 3:
                raise ValueError("Points must be a numpy array of shape (N, 3)")
            flat_points = points.flatten().tolist()
            num_points = points.shape[0]
        else:
            raise TypeError("Points must be a numpy array")

        request = planner_pb2.UpdatePointCloudRequest(
            points=flat_points,
            num_points=num_points
        )

        try:
            response = self.stub.UpdatePointCloud(request)
            print(f"Success: {response.success}")
            print(f"Message: {response.message}")
            return response.success, response.message
        except grpc.RpcError as e:
            print(f"RPC failed: {e}")
            return False, str(e)

    def convert_to_mesh(self, resolution=64.0):
        """
        Interface 2: Convert fitted surface to mesh triangles

        Args:
            resolution: Sampling resolution (default: 64.0)

        Returns:
            bool: True if successful
            list: List of triangles, each triangle is a list of 9 values [x1,y1,z1,x2,y2,z2,x3,y3,z3]
            str: Response message
        """
        print("\n=== Calling ConvertToMesh ===")
        print(f"Resolution: {resolution}")

        request = planner_pb2.ConvertToMeshRequest(resolution=resolution)

        try:
            response = self.stub.ConvertToMesh(request)
            print(f"Success: {response.success}")
            print(f"Message: {response.message}")
            print(f"Number of triangles: {response.num_triangles}")

            triangles = []
            for triangle in response.triangles:
                triangles.append(list(triangle.vertices))

            return response.success, triangles, response.message
        except grpc.RpcError as e:
            print(f"RPC failed: {e}")
            return False, [], str(e)

    def get_closest_point(self, x, y, z):
        """
        Interface 3: Get closest point on fitted surface

        Args:
            x, y, z: Input point coordinates

        Returns:
            bool: True if successful
            dict: Dictionary containing closest_point (x,y,z) and uv parameters
            str: Response message
        """
        print("\n=== Calling GetClosestPoint ===")
        print(f"Input point: ({x}, {y}, {z})")

        request = planner_pb2.GetClosestPointRequest(x=x, y=y, z=z)

        try:
            response = self.stub.GetClosestPoint(request)
            print(f"Success: {response.success}")
            print(f"Message: {response.message}")

            if response.success:
                result = {
                    'closest_point': (response.closest_x, response.closest_y, response.closest_z),
                    'uv': (response.u, response.v)
                }
                print(f"Closest point: {result['closest_point']}")
                print(f"UV parameters: {result['uv']}")
                return response.success, result, response.message
            else:
                return response.success, {}, response.message

        except grpc.RpcError as e:
            print(f"RPC failed: {e}")
            return False, {}, str(e)

    def plan_trajectory(self, start_u, start_v, goal_u, goal_v,
                       num_samples=5000, planning_timeout=20.0):
        """
        Interface 4: Plan trajectory on surface from start UV to goal UV

        Args:
            start_u, start_v: Start point UV parameters
            goal_u, goal_v: Goal point UV parameters
            num_samples: Number of samples for planner (default: 5000)
            planning_timeout: Planning timeout in seconds (default: 20.0)

        Returns:
            bool: True if successful
            list: List of trajectory points, each point is a dict with keys:
                  'position' (x,y,z), 'orientation' (psi,theta),
                  'surface_point' (sx,sy,sz), 'normal' (nx,ny,nz)
            float: Planning time in seconds
            str: Response message
        """
        print("\n=== Calling PlanTrajectory ===")
        print(f"Start UV: ({start_u}, {start_v})")
        print(f"Goal UV: ({goal_u}, {goal_v})")
        print(f"Num samples: {num_samples}")
        print(f"Timeout: {planning_timeout}s")

        request = planner_pb2.PlanTrajectoryRequest(
            start_u=start_u,
            start_v=start_v,
            goal_u=goal_u,
            goal_v=goal_v,
            num_samples=num_samples,
            planning_timeout=planning_timeout
        )

        try:
            response = self.stub.PlanTrajectory(request)
            print(f"Success: {response.success}")
            print(f"Message: {response.message}")

            if response.success:
                print(f"Planning time: {response.planning_time:.3f}s")
                print(f"Number of trajectory points: {response.num_trajectory_points}")

                trajectory = []
                for point in response.trajectory:
                    trajectory.append({
                        'position': (point.x, point.y, point.z),
                        'orientation': (point.psi, point.theta),
                        'surface_point': (point.sx, point.sy, point.sz),
                        'normal': (point.nx, point.ny, point.nz)
                    })

                return response.success, trajectory, response.planning_time, response.message
            else:
                return response.success, [], 0.0, response.message

        except grpc.RpcError as e:
            print(f"RPC failed: {e}")
            return False, [], 0.0, str(e)

    def get_surface_point(self, u, v):
        """
        Interface 5: Get surface point and normal at UV coordinates

        Args:
            u, v: UV parameters (0-1)

        Returns:
            bool: True if successful
            dict: Dictionary containing 'position' (x,y,z) and 'normal' (nx,ny,nz)
            str: Response message
        """
        print("\n=== Calling GetSurfacePoint ===")
        print(f"UV parameters: ({u}, {v})")

        request = planner_pb2.GetSurfacePointRequest(u=u, v=v)

        try:
            response = self.stub.GetSurfacePoint(request)
            print(f"Success: {response.success}")
            print(f"Message: {response.message}")

            if response.success:
                result = {
                    'position': (response.x, response.y, response.z),
                    'normal': (response.nx, response.ny, response.nz)
                }
                print(f"Surface point: {result['position']}")
                print(f"Normal vector: {result['normal']}")
                return response.success, result, response.message
            else:
                return response.success, {}, response.message

        except grpc.RpcError as e:
            print(f"RPC failed: {e}")
            return False, {}, str(e)


def main():
    """Example usage of the PlannerClient"""

    # Connect to server
    client = PlannerClient('localhost:50051')

    try:
        # Example 1: Update point cloud (optional, if you have new point cloud data)
        # Generate some example point cloud data
        print("\n" + "="*60)
        print("Example 1: Update Point Cloud")
        print("="*60)

        # Create a simple grid of points as example
        # In real usage, you would load actual point cloud data
        # points = np.random.rand(100, 3) * 2.0  # 100 random points
        # success, message = client.update_point_cloud(points)
        print("Skipping point cloud update (using existing data)")

        # Example 2: Convert surface to mesh
        print("\n" + "="*60)
        print("Example 2: Convert Surface to Mesh")
        print("="*60)

        success, triangles, message = client.convert_to_mesh(resolution=64.0)
        if success:
            print(f"Received {len(triangles)} triangles")
            if len(triangles) > 0:
                print(f"First triangle vertices: {triangles[0]}")

        # Example 3: Get closest point on surface
        print("\n" + "="*60)
        print("Example 3: Get Closest Point")
        print("="*60)

        success, result, message = client.get_closest_point(1.0, 1.0, 1.0)
        if success:
            print(f"Result: {result}")

        # Example 4: Plan trajectory
        print("\n" + "="*60)
        print("Example 4: Plan Trajectory")
        print("="*60)

        success, trajectory, planning_time, message = client.plan_trajectory(
            start_u=0.2, start_v=0.2,
            goal_u=0.8, goal_v=0.8,
            num_samples=5000,
            planning_timeout=20.0
        )

        if success:
            print(f"\nTrajectory summary:")
            print(f"  Total points: {len(trajectory)}")
            print(f"  Planning time: {planning_time:.3f}s")
            if len(trajectory) > 0:
                print(f"  Start position: {trajectory[0]['position']}")
                print(f"  Start orientation: {trajectory[0]['orientation']}")
                print(f"  Start normal: {trajectory[0]['normal']}")
                print(f"  End position: {trajectory[-1]['position']}")
                print(f"  End orientation: {trajectory[-1]['orientation']}")
                print(f"  End normal: {trajectory[-1]['normal']}")

        # Example 5: Get surface point at UV coordinates
        print("\n" + "="*60)
        print("Example 5: Get Surface Point")
        print("="*60)

        success, result, message = client.get_surface_point(0.5, 0.5)
        if success:
            print(f"Result: {result}")

    finally:
        client.close()
        print("\n" + "="*60)
        print("Client closed")
        print("="*60)


if __name__ == '__main__':
    main()
