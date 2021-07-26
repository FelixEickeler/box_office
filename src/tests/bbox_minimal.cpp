//
// Created by felix on 21.07.21.
//

#include "typedefs.h"
#include "mvbb_algorithm.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <Evaluator.h>
#include <CGAL/Point_set_3/IO/OFF.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;

int main(int argc, char* args[]) {
    std::string src = "/home/boxy/Clion/src/tests/data/minimal_object_example.off";

    std::vector<K::Point_3> points;
    if (!CGAL::IO::read_OFF(src, std::back_inserter(points))) {
        std::cout << "Testfile not found."  << src << "\n";
    }

    std::array<K::Point_3 , 8> obb_points;
    CGAL::oriented_bounding_box(points, obb_points);
    for(auto& p : obb_points){
        std::cout << p << "\n";
    }
//    std::filesystem::path ebbox("./example_bbox.off");
//    if (!CGAL::IO::write_OFF(ebbox, obb_points, CGAL::parameters::stream_precision(6))) {
//        std::cout << "oh no3\n";
//    }

}