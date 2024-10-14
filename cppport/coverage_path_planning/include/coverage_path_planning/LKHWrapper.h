#ifndef LKHWRAPPER_H
#define LKHWRAPPER_H

#include <CGAL/Point_set_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <iostream>
#include <cstdlib>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                          Point;

class LKHWrapper
{
private:
    void write_problem(std::vector<std::vector<double>> dist, std::string output_path);
    void write_params(std::string output_path);
    CGAL::Point_set_3<Point> result_to_pointset(CGAL::Point_set_3<Point> points, std::string output_path);

public:
    CGAL::Point_set_3<Point> execute(CGAL::Point_set_3<Point> points, std::vector<std::vector<double>> dist, std::string lkh_path="", std::string output_path="");
};

#endif //LKHWRAPPER_H

