#ifndef COVERAGE_PATH_PLANNING_INCLUDE_COVERAGE_PATH_PLANNING_H_
#define COVERAGE_PATH_PLANNING_INCLUDE_COVERAGE_PATH_PLANNING_H_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/duration.hpp>

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
#include <CGAL/Polygon_mesh_processing/intersection.h>

#include <CGAL/draw_surface_mesh.h>
#include <CGAL/draw_point_set_3.h>
#include <CGAL/draw_polyhedron.h>

#include <CGAL/IO/Color.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/Qt/Basic_viewer.h>

#include <vector>
#include <iostream>

#include "coverage_path_planning/action/coverage_action.hpp"
#include "coverage_path_planning/msg/point_arr.hpp"
#include "coverage_path_planning/LKHWrapper.h"


namespace coverage_path_planning {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                          Point;
typedef K::Point_2                                          Point_2;
typedef CGAL::Surface_mesh<Point>                           Mesh;
typedef K::Vector_3                                         Vector;

class CoveragePathPlanning {
    public:
        using CoverageAction = coverage_path_planning::action::CoverageAction;
        using GoalHandleCoverageAction = rclcpp_action::ServerGoalHandle<CoverageAction>;

        explicit CoveragePathPlanning(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                 std::vector<rclcpp::Parameter> parameters = {});
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<CoverageAction>::SharedPtr action_server_;    

        // Default ROS2 action server functions
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CoverageAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCoverageAction> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleCoverageAction> goal_handle);

        /*
        * Main execute function. Reads a given stl file, analyzes terrain and then solves a TSP to find a coverage path.
        */
        void execute(const std::shared_ptr<GoalHandleCoverageAction> goal_handle);

        /*
        * Reads an stl, turns it into a surface mesh and simplifies it.
        * 
        * std::string path: Filepath to the stl
        * 
        * return: Simplified surface mesh
        */
        Mesh stl_to_mesh(std::string path, float reduction_factor);

        /*
        * Given a mesh and a vector of faces, creates a new mesh that only contains those faces.
        * 
        * Mesh& mesh: The original mesh
        * std::vector<Mesh::Face_index> faces: The faces to be copied
        * 
        * return: The new mesh containining only the selected faces
        */
        Mesh get_sub_mesh(Mesh& mesh, std::vector<Mesh::Face_index> faces);

        /*
        * Projects a 3D mesh onto the xy plane. This is necessary, as a Surface_mesh<Point_2> does not count as a TriangularMesh
        * making some required functions unavailable
        * 
        * Mesh mesh3: 3D input mesh
        * 
        * return: Surface_mesh<Point_3> containing 2D projection
        */
        Mesh flatten_mesh(Mesh mesh3);

        /*
        * Calculates the distance matrix for the intra region case. If the straight line between two points crosses an obstacle,
        * it is increased by 100000.
        * 
        * CGAL::Point_set_3<Point> points: Points to be connected
        * Mesh obstacles: Obstacles to be avoided
        * 
        * return: Distance matrix
        */
        std::vector<std::vector<double>> get_distance_matrix(CGAL::Point_set_3<Point> points, Mesh obstacles);

        /*
        * Calculates the distance matrix for the inter region case. No obstacles, but the weight of subsequent pairs of points 
        * (i.e. 0 & 1, 2 & 3, 4 & 5, ...) is set to 0, as they should come from the same region.
        * 
        * CGAL::Point_set_3<Point> points: Points to be connected
        * 
        * return: Distance matrix
        */
        std::vector<std::vector<double>> get_distance_matrix(CGAL::Point_set_3<Point> points);

        /*
        * Checks whether the straight line between two points intersects with any face of a given mesh.
        *
        * Point p1, p2: Two input points to be connected
        * Mesh obstacles: Mesh containing obstacle faces
        * 
        * return: True if intersects, false otherwise
        */
        bool intersects_obstacle(Point p1, Point p2, Mesh obstacles);

        /*
        * Finds safe and unsafe triangles in a mesh based on a threshold.
        *
        * Mesh& mesh: Surface mesh to be analyzed
        * double threshold: Threshold in degrees. Triangles with a slope of less or equal the threshold are safe, otherwise unsafe.
        * Mesh::Property_map<Mesh::Face_index, CGAL::Color>& colormap: Colormap of mesh
        * 
        * return: First vector is all safe faces, second vector is all unsafe faces
        */
        std::pair<std::vector<Mesh::Face_index>, std::vector<Mesh::Face_index>> face_safety(Mesh& mesh, double threshold, Mesh::Property_map<Mesh::Face_index, CGAL::Color>& colormap);

        /*
        * Given a surface mesh and a list of faces, finds the centroid of each face.
        *
        * Mesh& mesh: Surface mesh to be analyzed
        * std::vector<Mesh::Face_index> faces: Vector of faces to be analyzed
        * 
        * return: Point set of all centroids
        */
        CGAL::Point_set_3<Point> get_centroids(Mesh& mesh, std::vector<Mesh::Face_index> faces);

        /*
        * Given a surface mesh, finds all connected components and returns them in a vector of vectors.
        *
        * Mesh& mesh: Surface mesh to be analyzed
        * 
        * return: Inner vectors contain all faces that belong to a certain component, outer vector contains all components
        */
        std::vector<std::vector<Mesh::Face_index>> get_connected_components(Mesh& mesh);

        /*
        * Formats the individual paths into a ros2 action result
        * 
        * std::shared_ptr<coverage_path_planning::action::CoverageAction::Result> result: Pointer to the result message
        * CGAL::Point_set_3<Point> endpoints: First and last point of each region, in order
        * CGAL::Point_set_3<Point> inter_path: Calculated path between all regions
        * std::vector<CGAL::Point_set_3<Point>> intra_paths: Calculated paths inside each region
        * 
        * return: Ros2 action result pointer, fields are now filled in
        */
        std::shared_ptr<coverage_path_planning::action::CoverageAction::Result> create_final_path(std::shared_ptr<coverage_path_planning::action::CoverageAction::Result> result, CGAL::Point_set_3<Point> endpoints, CGAL::Point_set_3<Point> inter_path, std::vector<CGAL::Point_set_3<Point>> intra_paths);


};
}
#endif  // COVERAGE_PATH_PLANNING_INCLUDE_COVERAGE_PATH_PLANNING_H_