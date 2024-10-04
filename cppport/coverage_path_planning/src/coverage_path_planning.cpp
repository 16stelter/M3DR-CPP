#include "coverage_path_planning/coverage_path_planning.hpp"

namespace coverage_path_planning {

CoveragePathPlanning::CoveragePathPlanning(rclcpp::Node::SharedPtr node, 
                                           const std::string &ns, 
                                           std::vector<rclcpp::Parameter> parameters) 
: node_(node)
{
    action_server_ = rclcpp_action::create_server<CoverageAction>(
        node_,
        "coverage_action",
        std::bind(&CoveragePathPlanning::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CoveragePathPlanning::handle_cancel, this, std::placeholders::_1),
        std::bind(&CoveragePathPlanning::handle_accepted, this, std::placeholders::_1)
    );
}

rclcpp_action::GoalResponse CoveragePathPlanning::handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                              std::shared_ptr<const CoverageAction::Goal> goal)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal request with input string: %s", goal->map_filepath.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

rclcpp_action::CancelResponse CoveragePathPlanning::handle_cancel(const std::shared_ptr<GoalHandleCoverageAction> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void CoveragePathPlanning::handle_accepted(const std::shared_ptr<GoalHandleCoverageAction> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&CoveragePathPlanning::execute, this, _1), goal_handle}.detach();
  }

void CoveragePathPlanning::execute(const std::shared_ptr<GoalHandleCoverageAction> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Executing goal...");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CoverageAction::Feedback>();
    auto result = std::make_shared<CoverageAction::Result>();

    std::string map_filepath = goal->map_filepath;

    try 
    {
      std::ifstream stl_file(map_filepath, std::ifstream::in);

      Mesh mesh;
      CGAL::IO::read_STL(stl_file, mesh);
      stl_file.close();

      RCLCPP_INFO(node_->get_logger(), "Nodes: %d", mesh.number_of_vertices());

      double min_z = std::numeric_limits<double>::max();

      for (auto v : vertices(mesh)) {
          double z = mesh.point(v).z();
          if (z < min_z && z != 0.0) {
              min_z = z;
          }
      }

      Mesh::Halfedge_index  hr; 
      for (auto v : vertices(mesh)) {
        if(abs(mesh.point(v).z() - min_z) < 1e-2)
        {
          Mesh::Halfedge_index  h = halfedge(v, mesh);
          hr = CGAL::Euler::remove_center_vertex(h, mesh);
        }
      }

      CGAL::Euler::remove_face(hr, mesh);

      CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<Mesh> stop(0.01);
      int num_collapsed = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop);

      mesh.collect_garbage();



      RCLCPP_INFO(node_->get_logger(), "Collapsed Nodes: %d", mesh.number_of_vertices());

      CGAL::draw(mesh); // visualization for debug

      std::ofstream out("output_mesh.off");
      out << mesh;
      out.close();
      result->success = true;
      RCLCPP_INFO(node_->get_logger(), "Finished processing string: %s", map_filepath.c_str());
    }
    catch (const char* msg) 
    {
      RCLCPP_INFO(node_->get_logger(), "Exception: %s", msg);
      result->success = false;
    }
    
    goal_handle->succeed(result);
}

} // namespace coverage_path_planning


int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("coverage_path_planning");

  // Create cpp node
  [[maybe_unused]] coverage_path_planning::CoveragePathPlanning coverage_path_planning(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}