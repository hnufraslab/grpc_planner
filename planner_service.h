#ifndef PLANNER_SERVICE_H
#define PLANNER_SERVICE_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include <ompl/base/AdaptiveDiscreteMotionValidator.h>
#include "invkin.h"
#include "nurbs.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <vector>
#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;

/**
 * @brief Planner service class that provides surface fitting and trajectory planning
 *
 * This class encapsulates the core functionality from PcsFmtTest.cpp and provides
 * a clean interface for gRPC service calls.
 */
class PlannerService {
public:
    /**
     * @brief Constructor
     */
    PlannerService();

    /**
     * @brief Destructor
     */
    ~PlannerService();

    /**
     * @brief Interface 1: Update point cloud data
     * @param points Flat array of point coordinates [x1,y1,z1,x2,y2,z2,...]
     * @param num_points Number of points
     * @return true if successful
     */
    bool updatePointCloud(const std::vector<double>& points, int num_points);

    /**
     * @brief Interface 2: Convert fitted surface to mesh triangles
     * @param resolution Sampling resolution (default 64)
     * @param triangles Output vector of triangles, each with 9 values [x1,y1,z1,x2,y2,z2,x3,y3,z3]
     * @return true if successful
     */
    bool convertToMesh(double resolution, std::vector<std::vector<double>>& triangles);

    /**
     * @brief Interface 3: Get closest point on fitted surface
     * @param x Input point x coordinate
     * @param y Input point y coordinate
     * @param z Input point z coordinate
     * @param closest_x Output closest point x
     * @param closest_y Output closest point y
     * @param closest_z Output closest point z
     * @param u Output UV parameter u
     * @param v Output UV parameter v
     * @return true if successful
     */
    bool getClosestPoint(double x, double y, double z,
                        double& closest_x, double& closest_y, double& closest_z,
                        double& u, double& v);

    /**
     * @brief Interface 4: Plan trajectory on surface from start UV to goal UV
     * @param start_u Start point u parameter
     * @param start_v Start point v parameter
     * @param goal_u Goal point u parameter
     * @param goal_v Goal point v parameter
     * @param trajectory Output trajectory points
     * @param planning_time Output planning time in seconds
     * @param num_samples Number of samples for planner (default 5000)
     * @param timeout Planning timeout in seconds (default 20.0)
     * @return true if successful
     */
    struct TrajectoryPoint {
        double x, y, z;           // UAV position
        double psi, theta;        // UAV orientation (yaw, pitch)
        double sx, sy, sz;        // Surface base point
        double nx, ny, nz;        // Surface normal vector
    };

    bool planTrajectory(double start_u, double start_v,
                       double goal_u, double goal_v,
                       std::vector<TrajectoryPoint>& trajectory,
                       double& planning_time,
                       int num_samples = 5000,
                       double timeout = 20.0);

    /**
     * @brief Interface 5: Get surface point and normal at UV coordinates
     * @param u UV parameter u (0-1)
     * @param v UV parameter v (0-1)
     * @param x Output surface point x coordinate
     * @param y Output surface point y coordinate
     * @param z Output surface point z coordinate
     * @param nx Output normal vector x component
     * @param ny Output normal vector y component
     * @param nz Output normal vector z component
     * @return true if successful
     */
    bool getSurfacePoint(double u, double v,
                        double& x, double& y, double& z,
                        double& nx, double& ny, double& nz);

    /**
     * @brief Check if surface has been fitted
     */
    bool isSurfaceFitted() const;

    /**
     * @brief Get error message from last operation
     */
    std::string getLastError() const;

private:
    // Core components
    std::unique_ptr<sr::Nurbs> nurbs_;
    std::unique_ptr<dp::InvKin> ik_;

    // Configuration
    std::vector<double> weights_;
    std::vector<double> clip_bound_;
    double link_length_;

    // State
    bool surface_fitted_;
    bool mesh_converted_;
    std::string last_error_;
    mutable std::mutex mutex_;

    // Helper functions
    bool isStateValid(const ob::State* state);
    bool isStateValidForQ(const ob::State* state);
};

#endif // PLANNER_SERVICE_H
