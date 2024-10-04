#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/STL.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/boost/graph/Euler_operations.h>

#include <vector>
#include <iostream>

#include "coverage_path_planning/action/coverage_action.hpp"

namespace coverage_path_planning {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                        Point;
typedef CGAL::Delaunay_triangulation_3<K>                 Delaunay;
typedef CGAL::Surface_mesh<Point>                         Mesh;

class CoveragePathPlanning {
    public:
        using CoverageAction = coverage_path_planning::action::CoverageAction;
        using GoalHandleCoverageAction = rclcpp_action::ServerGoalHandle<CoverageAction>;

        explicit CoveragePathPlanning(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                 std::vector<rclcpp::Parameter> parameters = {});
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<CoverageAction>::SharedPtr action_server_;    
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CoverageAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCoverageAction> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleCoverageAction> goal_handle);
        void execute(const std::shared_ptr<GoalHandleCoverageAction> goal_handle);

        void simplify_mesh(Mesh& mesh); 
};
}
#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_