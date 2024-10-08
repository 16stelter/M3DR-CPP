#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/STL.h>

#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>

#include <CGAL/draw_surface_mesh.h>
#include <CGAL/draw_point_set_3.h>
#include <CGAL/draw_polyhedron.h>

#include <CGAL/IO/Color.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/Qt/Basic_viewer.h>

#include <vector>
#include <iostream>

#include "coverage_path_planning/action/coverage_action.hpp"

namespace coverage_path_planning {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                        Point;
typedef CGAL::Surface_mesh<Point>                         Mesh;
typedef K::Vector_3 Vector;

struct Graphics_scene_options_black_points:
  public CGAL::Graphics_scene_options<CGAL::Point_set_3<Point>, typename CGAL::Point_set_3<Point>::const_iterator,
                                      typename CGAL::Point_set_3<Point>::const_iterator,
                                      typename CGAL::Point_set_3<Point>::const_iterator>
{
  bool colored_vertex(const CGAL::Point_set_3<Point>&, typename CGAL::Point_set_3<Point>::const_iterator) const
  { return true; }
  CGAL::IO::Color vertex_color(const CGAL::Point_set_3<Point>&, typename CGAL::Point_set_3<Point>::const_iterator) const
  { return CGAL::IO::Color(0,0,255); }
};
 

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
        std::pair<std::vector<Mesh::Face_index>, std::vector<Mesh::Face_index>> face_safety(Mesh& mesh, double threshold, Mesh::Property_map<Mesh::Face_index, CGAL::Color>& colormap);
        CGAL::Point_set_3<Point> get_centroids(Mesh& mesh, std::vector<Mesh::Face_index> fsafe);
        std::vector<std::vector<Mesh::Face_index>> get_connected_components(Mesh& mesh, std::vector<Mesh::Face_index> fsafe);


        void simplify_mesh(Mesh& mesh); 
};
}
#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_