#include <grpcpp/grpcpp.h>
#include "planner.grpc.pb.h"
#include "planner_service.h"
#include <iostream>
#include <memory>
#include <string>
#include <cstdlib>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
// Note: planner_service::PlannerService is the gRPC service interface from protobuf
// ::PlannerService is our implementation class
using planner_service::UpdatePointCloudRequest;
using planner_service::UpdatePointCloudResponse;
using planner_service::ConvertToMeshRequest;
using planner_service::ConvertToMeshResponse;
using planner_service::GetClosestPointRequest;
using planner_service::GetClosestPointResponse;
using planner_service::GetSurfacePointRequest;
using planner_service::GetSurfacePointResponse;
using planner_service::PlanTrajectoryRequest;
using planner_service::PlanTrajectoryResponse;

// Helper function to get environment variable with default value
std::string getEnvVar(const std::string& var, const std::string& defaultValue) {
    const char* value = std::getenv(var.c_str());
    return value ? std::string(value) : defaultValue;
}

// ============================================================================
// gRPC Service Implementation
// ============================================================================

class PlannerServiceImpl final : public planner_service::PlannerService::Service {
public:
    PlannerServiceImpl()
        : planner_() {
        std::cout << "gRPC PlannerService initialized" << std::endl;
    }

    // Interface 1: Update Point Cloud
    Status UpdatePointCloud(ServerContext* context,
                           const UpdatePointCloudRequest* request,
                           UpdatePointCloudResponse* response) override {
        std::cout << "\n=== UpdatePointCloud called ===" << std::endl;
        std::cout << "Received " << request->num_points() << " points" << std::endl;

        std::vector<double> points(request->points().begin(), request->points().end());

        bool success = planner_.updatePointCloud(points, request->num_points());

        response->set_success(success);
        if (success) {
            response->set_message("Point cloud updated successfully");
            std::cout << "Point cloud updated successfully" << std::endl;
        } else {
            response->set_message("Failed to update point cloud: " + planner_.getLastError());
            std::cerr << "Failed to update point cloud: " << planner_.getLastError() << std::endl;
        }

        return Status::OK;
    }

    // Interface 2: Convert to Mesh
    Status ConvertToMesh(ServerContext* context,
                        const ConvertToMeshRequest* request,
                        ConvertToMeshResponse* response) override {
        std::cout << "\n=== ConvertToMesh called ===" << std::endl;
        std::cout << "Resolution: " << request->resolution() << std::endl;

        double resolution = request->resolution() > 0 ? request->resolution() : 64.0;
        std::vector<std::vector<double>> triangles;

        bool success = planner_.convertToMesh(resolution, triangles);

        response->set_success(success);
        if (success) {
            response->set_num_triangles(triangles.size());

            for (const auto& tri : triangles) {
                auto* triangle = response->add_triangles();
                for (double val : tri) {
                    triangle->add_vertices(val);
                }
            }

            response->set_message("Mesh generated successfully with " +
                                std::to_string(triangles.size()) + " triangles");
            std::cout << "Mesh generated with " << triangles.size() << " triangles" << std::endl;
        } else {
            response->set_message("Failed to convert to mesh: " + planner_.getLastError());
            std::cerr << "Failed to convert to mesh: " << planner_.getLastError() << std::endl;
        }

        return Status::OK;
    }

    // Interface 3: Get Closest Point
    Status GetClosestPoint(ServerContext* context,
                          const GetClosestPointRequest* request,
                          GetClosestPointResponse* response) override {
        std::cout << "\n=== GetClosestPoint called ===" << std::endl;
        std::cout << "Input point: (" << request->x() << ", "
                  << request->y() << ", " << request->z() << ")" << std::endl;

        double closest_x, closest_y, closest_z, u, v;

        bool success = planner_.getClosestPoint(
            request->x(), request->y(), request->z(),
            closest_x, closest_y, closest_z, u, v
        );

        response->set_success(success);
        if (success) {
            response->set_closest_x(closest_x);
            response->set_closest_y(closest_y);
            response->set_closest_z(closest_z);
            response->set_u(u);
            response->set_v(v);
            response->set_message("Closest point found");

            std::cout << "Closest point: (" << closest_x << ", "
                      << closest_y << ", " << closest_z << ")" << std::endl;
            std::cout << "UV parameters: (" << u << ", " << v << ")" << std::endl;
        } else {
            response->set_message("Failed to find closest point: " + planner_.getLastError());
            std::cerr << "Failed to find closest point: " << planner_.getLastError() << std::endl;
        }

        return Status::OK;
    }

    // Interface 4: Plan Trajectory
    Status PlanTrajectory(ServerContext* context,
                         const PlanTrajectoryRequest* request,
                         PlanTrajectoryResponse* response) override {
        std::cout << "\n=== PlanTrajectory called ===" << std::endl;
        std::cout << "Start UV: (" << request->start_u() << ", " << request->start_v() << ")" << std::endl;
        std::cout << "Goal UV: (" << request->goal_u() << ", " << request->goal_v() << ")" << std::endl;

        int num_samples = request->num_samples() > 0 ? request->num_samples() : 5000;
        double timeout = request->planning_timeout() > 0 ? request->planning_timeout() : 20.0;

        std::vector<::PlannerService::TrajectoryPoint> trajectory;
        double planning_time;

        bool success = planner_.planTrajectory(
            request->start_u(), request->start_v(),
            request->goal_u(), request->goal_v(),
            trajectory, planning_time,
            num_samples, timeout
        );

        response->set_success(success);
        if (success) {
            response->set_planning_time(planning_time);
            response->set_num_trajectory_points(trajectory.size());

            for (const auto& tp : trajectory) {
                auto* point = response->add_trajectory();
                point->set_x(tp.x);
                point->set_y(tp.y);
                point->set_z(tp.z);
                point->set_psi(tp.psi);
                point->set_theta(tp.theta);
                point->set_sx(tp.sx);
                point->set_sy(tp.sy);
                point->set_sz(tp.sz);
                point->set_nx(tp.nx);
                point->set_ny(tp.ny);
                point->set_nz(tp.nz);
            }

            response->set_message("Trajectory planned successfully with " +
                                std::to_string(trajectory.size()) + " points in " +
                                std::to_string(planning_time) + " seconds");

            std::cout << "Trajectory planned: " << trajectory.size()
                      << " points in " << planning_time << " seconds" << std::endl;
        } else {
            response->set_message("Failed to plan trajectory: " + planner_.getLastError());
            std::cerr << "Failed to plan trajectory: " << planner_.getLastError() << std::endl;
        }

        return Status::OK;
    }

    // Interface 5: Get Surface Point
    Status GetSurfacePoint(ServerContext* context,
                          const GetSurfacePointRequest* request,
                          GetSurfacePointResponse* response) override {
        std::cout << "\n=== GetSurfacePoint called ===" << std::endl;
        std::cout << "UV parameters: (" << request->u() << ", " << request->v() << ")" << std::endl;

        double x, y, z, nx, ny, nz;

        bool success = planner_.getSurfacePoint(
            request->u(), request->v(),
            x, y, z, nx, ny, nz
        );

        response->set_success(success);
        if (success) {
            response->set_x(x);
            response->set_y(y);
            response->set_z(z);
            response->set_nx(nx);
            response->set_ny(ny);
            response->set_nz(nz);
            response->set_message("Surface point retrieved successfully");

            std::cout << "Surface point: (" << x << ", " << y << ", " << z << ")" << std::endl;
            std::cout << "Normal vector: (" << nx << ", " << ny << ", " << nz << ")" << std::endl;
        } else {
            response->set_message("Failed to get surface point: " + planner_.getLastError());
            std::cerr << "Failed to get surface point: " << planner_.getLastError() << std::endl;
        }

        return Status::OK;
    }

private:
    ::PlannerService planner_;
};

// ============================================================================
// Main Function
// ============================================================================

void RunServer() {
    // Get configuration from environment variables
    std::string server_address = getEnvVar("GRPC_SERVER_ADDRESS", "0.0.0.0:50051");

    std::cout << "Starting gRPC Planner Service..." << std::endl;
    std::cout << "Server Address: " << server_address << std::endl;
    std::cout << "Note: Point cloud will be provided via UpdatePointCloud interface" << std::endl;
    std::cout << "Note: No MuJoCo collision detection (collision checking disabled)" << std::endl;

    PlannerServiceImpl service;

    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "\n========================================" << std::endl;
    std::cout << "Server listening on " << server_address << std::endl;
    std::cout << "========================================\n" << std::endl;

    server->Wait();
}

int main(int argc, char** argv) {
    RunServer();
    return 0;
}
