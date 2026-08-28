#ifndef LKHWRAPPER_H
#define LKHWRAPPER_H

#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <iostream>
#include <iomanip>
#include <cstdlib>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                          Point;

class LKHWrapper
{
private:
    void write_problem(std::vector<std::vector<double>> dist, std::string output_path, bool is_inter_path);
    void write_params(std::string output_path, int runs);
    std::tuple<CGAL::Point_set_3<Point>, float, std::vector<int>> result_to_pointset(CGAL::Point_set_3<Point> points, std::string output_path);
    std::tuple<CGAL::Point_set_3<Point>, std::vector<int>> shift_point_set(CGAL::Point_set_3<Point> point_set, Point target, std::vector<int>& order);

public:
    std::tuple<CGAL::Point_set_3<Point>, float, std::vector<int>> execute(CGAL::Point_set_3<Point> points, std::vector<std::vector<double>> dist, std::string lkh_path="", std::string output_path="", int runs = 10, bool is_inter_path=false);
};

#endif //LKHWRAPPER_H

