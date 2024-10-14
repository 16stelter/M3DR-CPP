#include "coverage_path_planning/LKHWrapper.h"

CGAL::Point_set_3<Point> LKHWrapper::execute(CGAL::Point_set_3<Point> points, std::vector<std::vector<double>> dist, std::string lkh_path, std::string output_path)
{
  write_problem(dist, output_path);
  write_params(output_path);
  int result = system((lkh_path + " " + output_path + "/params.par").c_str());
  return result_to_pointset(points, output_path);
};

void LKHWrapper::write_problem(std::vector<std::vector<double>> dist, std::string output_path)
{
  std::cerr << (output_path + "/problem.tsp");
	std::ofstream problem_file(output_path + "/problem.tsp");
  if (!problem_file.is_open()) {
    std::cerr << "Failed to open file: " << strerror(errno) << std::endl;
  }
	problem_file << "NAME: TSP\nTYPE: TSP\nDIMENSION: " << dist.size() << "\nEDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
	for (size_t i = 0; i < dist.size(); i++) {
	  for (size_t j = 0; j < dist[i].size(); j++) {
		problem_file << dist[i][j] << " ";
	  }
	  problem_file << "\n";
	}
  problem_file << "EOF";
  problem_file.close();
}

void LKHWrapper::write_params(std::string output_path)
{
  std::ofstream param_file(output_path + "/params.par");
  param_file << "PROBLEM_FILE = " << output_path <<"/problem.tsp \nOUTPUT_TOUR_FILE = " << output_path << "/result.txt \n";
  param_file.close();
}

CGAL::Point_set_3<Point> LKHWrapper::result_to_pointset(CGAL::Point_set_3<Point> points, std::string output_path)
{
  std::ifstream result_file(output_path + "/result.txt");

  CGAL::Point_set_3<Point> path;
  std::string line;
  bool tour_section = false;
  while (std::getline(result_file, line)) {
    if (line.find("Length") != std::string::npos) 
    {
      std::cout << line << "\n";
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
        break;
      }
      int p = std::stoi(line);
      path.insert(points.point(p));
    }
  }
  result_file.close();
  return path;
}

