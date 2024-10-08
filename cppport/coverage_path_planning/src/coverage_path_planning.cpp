#include "coverage_path_planning/coverage_path_planning.hpp"

namespace PMP = CGAL::Polygon_mesh_processing;
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
    Mesh mesh;

    try 
    {
      std::ifstream stl_file(map_filepath, std::ifstream::in);

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
    }
    catch (const char* msg) 
    {
      RCLCPP_INFO(node_->get_logger(), "Exception: %s", msg);
      result->success = false;
    }

    auto colormap = mesh.add_property_map<Mesh::Face_index, CGAL::Color>("f:color", CGAL::Color(255, 255, 255)).first;
    auto [fsafe, fobstacle] = face_safety(mesh, 15, colormap); 
    auto cen = get_centroids(mesh, fsafe);
    auto components = get_connected_components(mesh, fsafe);

    // add color map to the mesh to visualize different regions later

    RCLCPP_INFO(node_->get_logger(), "Collapsed Nodes: %d", mesh.number_of_vertices());

    CGAL::Graphics_scene scene; // visualization for debug
    CGAL::add_to_graphics_scene(mesh, scene);
    for (const auto& point : cen.points()) {
      RCLCPP_INFO(node_->get_logger(), "Point: (%f, %f, %f)", point.x(), point.y(), point.z());
      scene.add_text(point, "x"); // weirdly, drawing points doesnt work so we do text instead
    }
    CGAL::draw_graphics_scene(scene);

    std::ofstream out("output_mesh.off");
    out << mesh;
    out.close();
    result->success = true;
    RCLCPP_INFO(node_->get_logger(), "Finished processing string: %s", map_filepath.c_str());

    goal_handle->succeed(result);
}

std::pair<std::vector<Mesh::Face_index>, std::vector<Mesh::Face_index>> CoveragePathPlanning::face_safety(Mesh& mesh, double threshold, Mesh::Property_map<Mesh::Face_index, CGAL::Color>& colormap)
{
    std::vector<Mesh::Face_index> fsafe;
    std::vector<Mesh::Face_index> fobstacle;

    for (auto f : mesh.faces())
    {
      Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, mesh);
      double angle = std::acos(normal.z()) * 180 / M_PI;
      
      if (angle < 15)
      {
        colormap[f] = CGAL::Color(0, 255, 0);
        fsafe.push_back(f);
      }
      else
      {
        colormap[f] = CGAL::Color(255, 0, 0);
        fobstacle.push_back(f);
      }
    }
    return {fsafe, fobstacle};
}

CGAL::Point_set_3<Point> CoveragePathPlanning::get_centroids(Mesh& mesh, std::vector<Mesh::Face_index> fsafe)
{
  CGAL::Point_set_3<Point> cen;
  for (auto f : fsafe)
  {
    auto h = mesh.halfedge(f);

    Point v1 = mesh.point(mesh.target(h));
    Point v2 = mesh.point(mesh.target(mesh.next(h)));
    Point v3 = mesh.point(mesh.target(mesh.next(mesh.next(h))));

    Point centroid = CGAL::centroid(v1, v2, v3);

    cen.insert(centroid);
  }
  return cen;
}

std::vector<std::vector<Mesh::Face_index>> CoveragePathPlanning::get_connected_components(Mesh& mesh, std::vector<Mesh::Face_index> fsafe)
{
  Mesh msafe;
  std::map<Point, Mesh::Vertex_index> vertex_map;

  for (const auto& f : fsafe) {
    auto h = mesh.halfedge(f);
    std::vector<Mesh::Vertex_index> vs;

    for (int i = 0; i < 3; ++i) {
      Point p = mesh.point(mesh.target(h));
      auto it = vertex_map.find(p);
      if (it == vertex_map.end()) {
        Mesh::Vertex_index idx = msafe.add_vertex(p);
        vertex_map[p] = idx;
      }
      vs.push_back(vertex_map[p]);
      h = mesh.next(h); 
    }
    msafe.add_face(vs);
  }

  //CGAL::draw(msafe); //Debug
  
  std::vector<std::size_t> components(num_faces(msafe));
  auto fcm = boost::make_iterator_property_map(components.begin(), get(boost::face_index, msafe));

  std::size_t num_components = CGAL::Polygon_mesh_processing::connected_components(msafe, fcm);

  RCLCPP_INFO(node_->get_logger(), "Connected components: %zu", num_components);

  for (const auto& f : faces(msafe)) {
      RCLCPP_INFO(node_->get_logger(), "Face %d is in component %zu", f.idx(), components[f]);
  }



  std::vector<std::vector<Mesh::Face_index>> map;
  return map;
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