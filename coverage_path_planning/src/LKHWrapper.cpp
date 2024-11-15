#include "coverage_path_planning/LKHWrapper.h"

std::pair<CGAL::Point_set_3<Point>, float> LKHWrapper::execute(CGAL::Point_set_3<Point> points, std::vector<std::vector<double>> dist, std::string lkh_path, std::string output_path, int runs, bool is_inter_path)
{
  if(points.size() == 1)
  {
    CGAL::Point_set_3<Point> path;
    path.insert(points.point(0));
    return std::make_pair(path, 0);
  }
  else if(points.size() == 2)
  {
    CGAL::Point_set_3<Point> path;
    path.insert(points.point(0));
    path.insert(points.point(1));
    float cost = dist[0][1];
    return std::make_pair(path, cost);
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
  if (!problem_file.is_open()) {
    std::cerr << "Failed to open file: " << strerror(errno) << std::endl;
  }
	problem_file << "NAME: TSP\nTYPE: TSP\nDIMENSION: " << dist.size() << "\nEDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: FULL_MATRIX\n SCALE: 100000\nEDGE_WEIGHT_SECTION\n";
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

std::pair<CGAL::Point_set_3<Point>, float> LKHWrapper::result_to_pointset(CGAL::Point_set_3<Point> points, std::string output_path)
{
  std::ifstream result_file(output_path + "/result.txt");

  CGAL::Point_set_3<Point> path;
  std::string line;
  float cost = -std::numeric_limits<float>::infinity();
  bool tour_section = false;
  Point p0(0.0, 0.0, 0.0);
  while (std::getline(result_file, line)) {
    if (line.find("Length") != std::string::npos) 
    {
      cost = std::stof(line.substr(line.find('=') + 1))/100000; //using scale 100000 to include the decimals in LKH so need to divide through 100000 here
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
        }
        else
        {
          path.insert(p0);
        }
      }
    }
  }
  path = shift_point_set(path, p0);

  result_file.close();
  return std::make_pair(path, cost);
};

CGAL::Point_set_3<Point> LKHWrapper::shift_point_set(CGAL::Point_set_3<Point> point_set, Point target)
{
  auto it = point_set.begin();
  for (; it != point_set.end(); ++it) {
      if (point_set.point(*it) == Point(0, 0, 0)) {
          break;
      }
  }

  if (it != point_set.end()) {
      std::rotate(point_set.begin(), it+1, point_set.end()); 
  }

  if (!point_set.empty()) {
      point_set.remove(point_set.end()-1);
  }

  return point_set;
}

