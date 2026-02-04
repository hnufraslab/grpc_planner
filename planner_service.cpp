#include "planner_service.h"
#include <ompl/util/Time.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <iostream>

// ============================================================================
// Constructor and Destructor
// ============================================================================

PlannerService::PlannerService()
    : weights_({1.0, 1.0, 1.0, 3.0, 6.0})
    , clip_bound_({0.1, 0.9, 0.1, 0.9})
    , link_length_(0.96)
    , surface_fitted_(false)
    , mesh_converted_(false)
{
    std::cout << "PlannerService initialized successfully" << std::endl;
    std::cout << "Waiting for point cloud data via UpdatePointCloud interface..." << std::endl;
}

PlannerService::~PlannerService() {
    // Cleanup handled by unique_ptr
}

// ============================================================================
// Interface 1: Update Point Cloud
// ============================================================================

bool PlannerService::updatePointCloud(const std::vector<double>& points, int num_points) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        if (points.size() != static_cast<size_t>(num_points * 3)) {
            last_error_ = "Invalid point data size";
            return false;
        }

        // Create PCL point cloud from flat array
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = num_points;
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->points.resize(num_points);

        for (int i = 0; i < num_points; ++i) {
            cloud->points[i].x = points[i * 3 + 0];
            cloud->points[i].y = points[i * 3 + 1];
            cloud->points[i].z = points[i * 3 + 2];
        }

        // Create or update NURBS surface
        if (!nurbs_) {
            // First time: create NURBS with point cloud
            nurbs_ = std::make_unique<sr::Nurbs>(cloud);
            std::cout << "NURBS surface created with " << num_points << " points" << std::endl;
        } else {
            // Update existing NURBS surface
            nurbs_->updatePointCloud(cloud);
            std::cout << "NURBS surface updated with " << num_points << " points" << std::endl;
        }

        // Fit the surface
        if (nurbs_->fitSurface() != 0) {
            last_error_ = "Failed to fit surface";
            surface_fitted_ = false;
            return false;
        }

        surface_fitted_ = true;
        mesh_converted_ = false;  // Need to regenerate mesh

        // Initialize or update inverse kinematics
        if (!ik_) {
            ik_ = std::make_unique<dp::InvKin>(nurbs_.get());
            ik_->setLinkLength(link_length_);
            ik_->setClipBound(clip_bound_);
            std::cout << "Inverse kinematics initialized" << std::endl;
        }

        std::cout << "Surface fitted successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("Exception in updatePointCloud: ") + e.what();
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

// ============================================================================
// Interface 2: Convert to Mesh
// ============================================================================

bool PlannerService::convertToMesh(double resolution, std::vector<std::vector<double>>& triangles) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        if (!surface_fitted_) {
            last_error_ = "Surface not fitted yet";
            return false;
        }

        // Convert surface to mesh
        if (nurbs_->convertToMesh(resolution) != 0) {
            last_error_ = "Failed to convert surface to mesh";
            return false;
        }

        mesh_converted_ = true;

        // Get mesh triangles
        const Eigen::MatrixXd& mesh_triangles = nurbs_->getMeshTriangles();
        int num_triangles = mesh_triangles.rows();

        triangles.clear();
        triangles.reserve(num_triangles);

        for (int i = 0; i < num_triangles; ++i) {
            std::vector<double> triangle(9);
            for (int j = 0; j < 9; ++j) {
                triangle[j] = mesh_triangles(i, j);
            }
            triangles.push_back(triangle);
        }

        std::cout << "Mesh generated with " << num_triangles << " triangles" << std::endl;
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("Exception in convertToMesh: ") + e.what();
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

// ============================================================================
// Interface 3: Get Closest Point
// ============================================================================

bool PlannerService::getClosestPoint(double x, double y, double z,
                                     double& closest_x, double& closest_y, double& closest_z,
                                     double& u, double& v) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        if (!surface_fitted_) {
            last_error_ = "Surface not fitted yet";
            return false;
        }

        Eigen::Vector3d input_point(x, y, z);

        // Get closest point on surface
        if (nurbs_->getClosestPoint(input_point, u, v) != 0) {
            last_error_ = "Failed to find closest point";
            return false;
        }

        // Get position at UV
        Eigen::Vector3d closest_point;
        if (nurbs_->getPos(u, v, closest_point) != 0) {
            last_error_ = "Failed to get position at UV";
            return false;
        }

        closest_x = closest_point.x();
        closest_y = closest_point.y();
        closest_z = closest_point.z();

        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("Exception in getClosestPoint: ") + e.what();
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

// ============================================================================
// Interface 4: Plan Trajectory
// ============================================================================

bool PlannerService::planTrajectory(double start_u, double start_v,
                                    double goal_u, double goal_v,
                                    std::vector<TrajectoryPoint>& trajectory,
                                    double& planning_time,
                                    int num_samples,
                                    double timeout) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        if (!surface_fitted_) {
            last_error_ = "Surface not fitted yet";
            return false;
        }

        trajectory.clear();

        // Setup parameter space (UV space)
        uint paramDimensionsNum = 2;
        uint stateDimensionsNum = 5;

        auto space = std::make_shared<ob::RealVectorStateSpace>(paramDimensionsNum);
        auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(stateDimensionsNum);

        ob::RealVectorBounds bounds(paramDimensionsNum);
        bounds.setLow(0.0);
        bounds.setHigh(1.0);
        space->setBounds(bounds);

        // Create space information
        auto si = std::make_shared<ob::SpaceInformation>(space);
        
        // Set state validity checker using lambda
        si->setStateValidityChecker([this](const ob::State* state) {
            return this->isStateValid(state);
        });

        // Setup motion validator
        auto motionValidator = std::make_shared<ob::AdaptiveDiscreteMotionValidator>(si);
        si->setMotionValidator(motionValidator);
        si->setup();

        // Calculate cost resolution
        Eigen::Matrix<double, 5, 1> weightsVec;
        weightsVec << weights_[0], weights_[1], weights_[2], weights_[3], weights_[4];

        auto qStart = ik_->xToQ(start_u, start_v);
        auto qGoal = ik_->xToQ(goal_u, goal_v);
        double estimateResol = (qStart - qGoal).cwiseProduct(weightsVec).norm() * 0.05;
        motionValidator->setCostResolution(estimateResol);

        // Setup state space for output
        auto stateSi = std::make_shared<ob::SpaceInformation>(stateSpace);
        stateSi->setStateValidityChecker([this](const ob::State* state) {
            return this->isStateValidForQ(state);
        });

        // Create optimization objective
        auto opt = std::make_shared<ob::EstimatePathLengthOptimizationObjective>(si);

        // Create problem definition
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);

        // Set start and goal states
        ob::ScopedState<> start(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_u;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_v;

        ob::ScopedState<> goal(space);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_u;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_v;

        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(opt);

        // Create PCS-FMT planner
        auto planner = std::make_shared<og::PCSFMT>(si, stateSi, ik_.get());
        planner->setNumSamples(num_samples);
        planner->setWeights(weights_);
        planner->setProblemDefinition(pdef);
        planner->setup();

        std::cout << "Planning from (" << start_u << "," << start_v 
                  << ") to (" << goal_u << "," << goal_v << ")" << std::endl;

        // Solve
        ompl::time::point start_time = ompl::time::now();
        ob::PlannerStatus solved = planner->ob::Planner::solve(timeout);
        planning_time = ompl::time::seconds(ompl::time::now() - start_time);

        if (!solved) {
            last_error_ = "Planning failed - no solution found";
            std::cout << "Planning failed after " << planning_time << " seconds" << std::endl;
            return false;
        }

        std::cout << "Planning succeeded in " << planning_time << " seconds" << std::endl;

        // Get solution path (in UV space)
        ob::PathPtr path = pdef->getSolutionPath();

        // Smooth the UV path
        og::PathSimplifier psUV(si);
        psUV.smoothBSpline(*path->as<og::PathGeometric>(), 3, 0.005);

        // Extract trajectory with all information in one loop
        auto uvStates = path->as<og::PathGeometric>()->getStates();
        trajectory.reserve(uvStates.size());

        for (auto uvState : uvStates) {
            auto u = uvState->as<ob::RealVectorStateSpace::StateType>()->values[0];
            auto v = uvState->as<ob::RealVectorStateSpace::StateType>()->values[1];

            // Get UAV state (position and orientation)
            auto q = ik_->xToQ(u, v);

            // Get surface base point and normal
            auto q_s = ik_->xToQs(u, v);

            TrajectoryPoint tp;
            // UAV position and orientation from q
            tp.x = q(0);
            tp.y = q(1);
            tp.z = q(2);
            tp.psi = q(3);
            tp.theta = q(4);

            // Surface base point and normal from q_s
            // q_s contains: [sx, sy, sz, nx, ny, nz, x, y, z, psi, theta]
            tp.sx = q_s(0);
            tp.sy = q_s(1);
            tp.sz = q_s(2);
            tp.nx = q_s(3);
            tp.ny = q_s(4);
            tp.nz = q_s(5);

            trajectory.push_back(tp);
        }

        std::cout << "Trajectory generated with " << trajectory.size() << " points" << std::endl;
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("Exception in planTrajectory: ") + e.what();
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

bool PlannerService::isStateValid(const ob::State* state) {
    // auto u = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    // auto v = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    // auto q = ik_->xToQ(u, v);
    // auto ret = mujoco_client_->isCollision(std::vector<double>(q.data(), q.data() + q.size()));
    // return !ret;
    return true;
}

bool PlannerService::isStateValidForQ(const ob::State* state) {
    // auto q = state->as<ob::RealVectorStateSpace::StateType>()->values;
    // auto ret = mujoco_client_->isCollision(std::vector<double>(q, q + 5));
    // return !ret;
    return true;
}

bool PlannerService::isSurfaceFitted() const {
    return surface_fitted_;
}

std::string PlannerService::getLastError() const {
    return last_error_;
}
