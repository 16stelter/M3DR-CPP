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
    // ros2 action server goal setup
    RCLCPP_INFO(node_->get_logger(), "Executing goal...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CoverageAction::Feedback>();
    auto result = std::make_shared<CoverageAction::Result>();

    Mesh mesh = stl_to_mesh(goal->map_filepath); 
    RCLCPP_INFO(node_->get_logger(), "Mesh Vertices: %d", mesh.number_of_vertices());
    // add color map so we can show safe and obstacle triangles in different colors. 
    auto colormap = mesh.add_property_map<Mesh::Face_index, CGAL::Color>("f:color", CGAL::Color(255, 255, 255)).first;

    auto [fsafe, fobstacle] = face_safety(mesh, 15, colormap);
    Mesh msafe = get_sub_mesh(mesh, fsafe);
    Mesh mobstacle = get_sub_mesh(mesh, fobstacle);
    Mesh mobstacle2 = flatten_mesh(mobstacle); // set z = 0 to fake a 2d mesh (Mesh<Point_2> is not a TriangleMesh so doesn't work correctly)
    std::vector<std::vector<Mesh::Face_index>> components = get_connected_components(msafe);

    CGAL::Graphics_scene scene;
    LKHWrapper lkh;
    for (std::vector<Mesh::Face_index> c : components) {
      CGAL::Point_set_3<Point> cen = get_centroids(msafe, c);
      std::vector<std::vector<double>> dist = get_distance_matrix(cen, mobstacle2);
      std::string package_path = ament_index_cpp::get_package_share_directory("coverage_path_planning");
      auto path = lkh.execute(cen, dist, package_path + "/resources/LKH-3.0.11/LKH", package_path);
      for (const auto& point : cen.points()) {
        scene.add_text(point, "x");
      }
    }

    CGAL::add_to_graphics_scene(mesh, scene);
    CGAL::draw_graphics_scene(scene);

    result->success = true;
    RCLCPP_INFO(node_->get_logger(), "Finished processing string: %s", goal->map_filepath.c_str());

    goal_handle->succeed(result);
}

Mesh CoveragePathPlanning::stl_to_mesh(std::string path)
{
  Mesh mesh;
  try 
  {
    std::ifstream stl_file(path, std::ifstream::in);

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
      if(abs(mesh.point(v).z() - min_z) < 1e-2) // small tolerance so all bottom vertices are removed
      {
        Mesh::Halfedge_index  h = halfedge(v, mesh);
        hr = CGAL::Euler::remove_center_vertex(h, mesh);
      }
    }

    CGAL::Euler::remove_face(hr, mesh); // previous step leaves one large face from merging the others. Remove that face here.

    CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<Mesh> stop(0.01);
    int num_collapsed = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop);

    mesh.collect_garbage();
  }
  catch (const char* msg) 
  {
    RCLCPP_INFO(node_->get_logger(), "Exception: %s", msg);
  }
  return mesh;
}

Mesh CoveragePathPlanning::get_sub_mesh(Mesh& mesh, std::vector<Mesh::Face_index> faces)
{
  Mesh subm;
  std::map<Point, Mesh::Vertex_index> vertex_map;

  for (const auto& f : faces) {
    auto h = mesh.halfedge(f);
    std::vector<Mesh::Vertex_index> vs;

    for (int i = 0; i < 3; ++i) {
      Point p = mesh.point(mesh.target(h));
      auto it = vertex_map.find(p);
      if (it == vertex_map.end()) {
        Mesh::Vertex_index idx = subm.add_vertex(p);
        vertex_map[p] = idx;
      }
      vs.push_back(vertex_map[p]);
      h = mesh.next(h); 
    }
    subm.add_face(vs);
  }
  return subm;
}

Mesh CoveragePathPlanning::flatten_mesh(Mesh mesh3)
{
  Mesh mesh2;

  for (auto v : vertices(mesh3)) {
        Point p = mesh3.point(v);
        Point p2(p.x(), p.y(), 0);
        mesh2.add_vertex(p2);
    }
  for (auto f : faces(mesh3)) {
    std::vector<Mesh::Vertex_index> v2;
    for (auto v : CGAL::vertices_around_face(mesh3.halfedge(f), mesh3)) {
      v2.push_back(Mesh::Vertex_index(v));
    }
    mesh2.add_face(v2);
  }

  return mesh2;
}

std::vector<std::vector<double>> CoveragePathPlanning::get_distance_matrix(CGAL::Point_set_3<Point> points, Mesh obstacles)
{
  std::vector<std::vector<double>> distance_matrix(points.size(), std::vector<double>(points.size(), 0));

  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      double dist = std::sqrt(CGAL::squared_distance(points.point(i), points.point(j)));
      if(intersects_obstacle(points.point(i), points.point(j), obstacles))
      {
        dist += 100000;
      }
      distance_matrix[i][j] = dist;
      distance_matrix[j][i] = dist;
    }
  }

  return distance_matrix;
}

bool CoveragePathPlanning::intersects_obstacle(Point p1, Point p2, Mesh obstacles)
{
  Point p1_2(p1.x(), p1.y(), 0);
  Point p2_2(p2.x(), p2.y(), 0);
  std::vector<Point> points = {p1_2, p2_2};
  auto range = boost::make_iterator_range(points.begin(), points.end());
  return PMP::do_intersect(obstacles, range);
}

std::pair<std::vector<Mesh::Face_index>, std::vector<Mesh::Face_index>> CoveragePathPlanning::face_safety(Mesh& mesh, 
                                                                                                          double threshold, 
                                                                                                          Mesh::Property_map<Mesh::Face_index, 
                                                                                                          CGAL::Color>& colormap)
{
    std::vector<Mesh::Face_index> fsafe;
    std::vector<Mesh::Face_index> fobstacle;

    for (auto f : mesh.faces())
    {
      Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, mesh);
      double angle = std::acos(normal.z()) * 180 / M_PI;
      
      if (angle <= 15)
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

CGAL::Point_set_3<Point> CoveragePathPlanning::get_centroids(Mesh& mesh, std::vector<Mesh::Face_index> faces)
{
  CGAL::Point_set_3<Point> cen;
  for (auto f : faces)
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

std::vector<std::vector<Mesh::Face_index>> CoveragePathPlanning::get_connected_components(Mesh& mesh)
{  
  std::vector<std::size_t> components(num_faces(mesh));
  auto fcm = boost::make_iterator_property_map(components.begin(), get(boost::face_index, mesh));

  std::size_t num_components = CGAL::Polygon_mesh_processing::connected_components(mesh, fcm);

  RCLCPP_INFO(node_->get_logger(), "Connected components: %zu", num_components);

  std::vector<std::vector<Mesh::Face_index>> map;
  for (int i = 0; static_cast<long unsigned int>(i) < num_components; i++)
  {
    std::vector<Mesh::Face_index> v;
    map.push_back(v);
  }
  for (const auto& f: faces(mesh)) 
  {
    map[components[f]].push_back(f);
  }
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