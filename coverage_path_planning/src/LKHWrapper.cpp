#include "coverage_path_planning/LKHWrapper.h"

std::tuple<CGAL::Point_set_3<Point>, float, std::vector<int>> LKHWrapper::execute(CGAL::Point_set_3<Point> points, std::vector<std::vector<double>> dist, std::string lkh_path, std::string output_path, int runs, bool is_inter_path)
{
  if(points.size() == 1)
  {
    CGAL::Point_set_3<Point> path;
    path.insert(points.point(0));
    return std::make_tuple(path, 0, std::vector<int>());
  }
  else if(points.size() == 2)
  {
    CGAL::Point_set_3<Point> path;
    path.insert(points.point(0));
    path.insert(points.point(1));
    float cost = dist[0][1];
    return std::make_tuple(path, cost, std::vector<int>());
  }

  write_problem(dist, output_path, is_inter_path);
  write_params(output_path, runs);
  int result = system((lkh_path + " " + output_path + "/params.par").c_str());
  return result_to_pointset(points, output_path);
};

void LKHWrapper::write_problem(std::vector<std::vector<double>> dist, std::string output_path, bool is_inter_path)
{
  std::cerr << (output_path + "/problem.tsp");
	std::ofstream problem_file(output_path + "/problem.tsp");
  problem_file << std::fixed << std::setprecision(15);
  if (!problem_file.is_open()) {
    std::cerr << "Failed to open file: " << strerror(errno) << std::endl;
  }
	problem_file << "NAME: TSP\nTYPE: TSP\nDIMENSION: " << dist.size() << "\nEDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: FULL_MATRIX\n SCALE: 1000000\nEDGE_WEIGHT_SECTION\n";
	for (size_t i = 0; i < dist.size(); i++) {
	  for (size_t j = 0; j < dist[i].size(); j++) {
		  problem_file << dist[i][j] << " ";
	  }
	  problem_file << "\n";
	}
  if(is_inter_path)
  {
    problem_file << "FIXED_EDGES_SECTION" << "\n";
    for (size_t i = 0; i < dist.size()-2; i = i+2) 
    {
      problem_file << i+1 << " " << i+2 << "\n";
    }

  }
  problem_file << "EOF";
  problem_file.close();
};

void LKHWrapper::write_params(std::string output_path, int runs)
{
  std::ofstream param_file(output_path + "/params.par");
  param_file << "PROBLEM_FILE = " << output_path <<"/problem.tsp \nOUTPUT_TOUR_FILE = " << output_path << "/result.txt \nPRECISION = 10\n RUNS = " << runs << "\n TRACE_LEVEL = 0 \n";
  param_file.close();
};

std::tuple<CGAL::Point_set_3<Point>, float, std::vector<int>> LKHWrapper::result_to_pointset(CGAL::Point_set_3<Point> points, std::string output_path)
{
  std::ifstream result_file(output_path + "/result.txt");

  CGAL::Point_set_3<Point> path;
  std::vector<int> order;
  std::string line;
  float cost = -std::numeric_limits<float>::infinity();
  bool tour_section = false;
  Point p0(0.0, 0.0, 0.0);
  while (std::getline(result_file, line)) {
    if (line.find("Length") != std::string::npos) 
    {
      cost = std::stof(line.substr(line.find('=') + 1))/1000000; //using scale 1000000 to include the decimals in LKH so need to divide through 1000000 here
    }
    else if (line.find("TOUR_SECTION") != std::string::npos) 
    {
      tour_section = true;
    }
    else if (tour_section)
    {
      if (line.find("-1") != std::string::npos)
      {
        tour_section = false;
      }
      else
      {
        int p = std::stoi(line) - 1;
        if(p < static_cast<int>(points.size()))
        {
          path.insert(points.point(p));
          order.push_back(p);
        }
        else
        {
          path.insert(p0);
          order.push_back(-1);
        }
      }
    }
  }
  std::tie(path, order) = shift_point_set(path, p0, order);

  result_file.close();
  return std::make_tuple(path, cost, order);
};

std::tuple<CGAL::Point_set_3<Point>, std::vector<int>> LKHWrapper::shift_point_set(CGAL::Point_set_3<Point> point_set, Point target, std::vector<int>& order)
{
  auto it = point_set.begin();
  for (; it != point_set.end(); ++it) {
      if (point_set.point(*it) == Point(0, 0, 0)) {
          break;
      }
  }

  if (it != point_set.end()) {
      std::rotate(point_set.begin(), it+1, point_set.end()); 
      std::rotate(order.begin(), order.begin() + (it - point_set.begin() + 1), order.end());
  }

  if (!point_set.empty()) {
      point_set.remove(point_set.end()-1);
  }

  return make_tuple(point_set, order);
}

