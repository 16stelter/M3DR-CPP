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
    CGAL::Point_set_3<Point> endpoints;
    std::string package_path = ament_index_cpp::get_package_share_directory("coverage_path_planning");
    std::vector<CGAL::Point_set_3<Point>> intra_paths;
    
    for (std::vector<Mesh::Face_index> c : components) {
      CGAL::Point_set_3<Point> cen = get_centroids(msafe, c);
      std::vector<std::vector<double>> dist = get_distance_matrix(cen, mobstacle2);
      auto path = lkh.execute(cen, dist, package_path + "/resources/LKH-3.0.11/LKH", package_path);
      intra_paths.push_back(path);
      bool first = true;
      Point p1;
      endpoints.insert(path.point(*path.begin()));
      endpoints.insert(path.point(*(--path.end())));
      for (Point p : path.points())
      {
        if (first)
        {
          first = false;
        }
        else
        {
          scene.add_segment(p1, p, CGAL::Color(0, 0, 255));
        }
        p1 = p;
      }
    }

    std::vector<std::vector<double>> dist = get_distance_matrix(endpoints);
    auto inter_path = lkh.execute(endpoints, dist, package_path + "/resources/LKH-3.0.11/LKH", package_path);
    Point p1;
    int count = 0;
    for (Point p : inter_path.points())
    {
      if (!count == 0 && count % 2 == 0)
      {
          scene.add_segment(p1, p, CGAL::Color(125, 0, 125));
      }
      p1 = p;
      count++;
    }

    if(goal->debug_mode == 2)
    {
      CGAL::add_to_graphics_scene(mesh, scene);
    }
    if(goal->debug_mode > 0)
    {
      CGAL::draw_graphics_scene(scene);
    }

    result = create_final_path(result, endpoints, inter_path, intra_paths);

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

    CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<Mesh> stop(0.005);
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
      if (i != j && j > i) {
        double dist = std::sqrt(CGAL::squared_distance(points.point(i), points.point(j)));
        if(intersects_obstacle(points.point(i), points.point(j), obstacles))
        {
          dist += 100000;
        }
        distance_matrix[i][j] = dist;
        distance_matrix[j][i] = dist;
      }
    }
  }

  return distance_matrix;
}

std::vector<std::vector<double>> CoveragePathPlanning::get_distance_matrix(CGAL::Point_set_3<Point> points)
{
  std::vector<std::vector<double>> distance_matrix(points.size(), std::vector<double>(points.size(), 0));

  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      if (i != j && j > i) {
        double dist = (i % 2 == 0 && j == i + 1) ? 0 : std::sqrt(CGAL::squared_distance(points.point(i), points.point(j)));
        distance_matrix[i][j] = dist;
        distance_matrix[j][i] = dist;
      }
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

std::shared_ptr<coverage_path_planning::action::CoverageAction::Result> CoveragePathPlanning::create_final_path(std::shared_ptr<coverage_path_planning::action::CoverageAction::Result> result, CGAL::Point_set_3<Point> endpoints, CGAL::Point_set_3<Point> inter_path, std::vector<CGAL::Point_set_3<Point>> intra_paths)
{
  std::vector<geometry_msgs::msg::Point> inter_p;
  std::vector<int> region_order;
  std::vector<bool> inverse(inter_path.size()/2, false);
  for(int i = 0; i < static_cast<int>(inter_path.size()); i++)
  {
    geometry_msgs::msg::Point p;
    p.x = inter_path.point(i).x();
    p.y = inter_path.point(i).y();
    p.z = inter_path.point(i).z();
    inter_p.push_back(p);
    if(i % 2 == 0)
    {
      for (int j = 0; j < static_cast<int>(endpoints.size()); j++) {
        if (endpoints.point(j) == inter_path.point(i)) {
          inverse[floor(j/2)] = (j % 2 == 1); // Mark regions as inverted if entered through second endpoint
          region_order.push_back(floor(j/2));
          break;
        }
      }
    }
  }
  std::vector<std::vector<geometry_msgs::msg::Point>> intra_p;
  for(int i = 0; i < static_cast<int>(intra_paths.size()); i++)
  {
    coverage_path_planning::msg::PointArr pointArr;
    std::vector<geometry_msgs::msg::Point> region;
    if(inverse[i]) 
    {
      for(int j = static_cast<int>(intra_paths[i].size())-1; j >= 0; j--)
      {
        geometry_msgs::msg::Point p;
        p.x = intra_paths[i].point(j).x();
        p.y = intra_paths[i].point(j).y();
        p.z = intra_paths[i].point(j).z();
        pointArr.data.push_back(p);
      }
    }
    else 
    {
      for(int j = 0; j < static_cast<int>(intra_paths[i].size()); j++)
      {
        geometry_msgs::msg::Point p;
        p.x = intra_paths[i].point(j).x();
        p.y = intra_paths[i].point(j).y();
        p.z = intra_paths[i].point(j).z();
        pointArr.data.push_back(p);
      }
    }
    result->intra_paths.push_back(pointArr);
  }
  result->inter_path = inter_p;
  result->region_order = region_order;
  return result;
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