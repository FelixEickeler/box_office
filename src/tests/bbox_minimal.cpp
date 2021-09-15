//
// Created by felix on 21.07.21.
//

//#include "typedefs.h"
//#include "mvbb_algorithms.h"
//#include <CGAL/Point_set_3/IO/OFF.h>
//
//
//typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
//
//int main(int argc, char* args[]) {
//    std::string src = "../../../src/tests/data/bunny00.off";
//
//    std::vector<K::Point_3> points;
//    if (!CGAL::IO::read_OFF(src, std::back_inserter(points))) {
//        std::cout << "Testfile not found."  << src << "\n";
//    }
//
//    std::array<K::Point_3 , 8> obb_points;
//    CGAL::oriented_bounding_box(points, obb_points);
//    for(auto& p : obb_points){
//        std::cout << p << "\n";
//    }
////    std::filesystem::path ebbox("./example_bbox.off");
////    if (!CGAL::IO::write_OFF(ebbox, obb_points, CGAL::parameters::stream_precision(6))) {
////        std::cout << "oh no3\n";
////    }
//
//}