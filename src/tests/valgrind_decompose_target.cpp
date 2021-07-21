//
// Created by felix on 17.07.2021.
//
#include "typedefs.h"
#include "mvbb_algorithm.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <Evaluator.h>

//int main(int argc, char* args[]){
//    mvbb::Algo_MVBB<pointcloud_xyzc> mvbb;
//    std::filesystem::path source_pig("../../tests/data/pig.off");
//    CGAL::Surface_mesh<Point> sm;
//    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(source_pig, sm) || sm.is_empty()) {
//        std::cout  << "Testfile not found.";
//    }
//    pointcloud_xyzc points;
//    for (const auto &v : vertices(sm)) {
//        points.push_back(std::make_tuple(sm.point(v), 0));
//    }
//
//    uint32_t levels = 6;
//    auto start = std::chrono::high_resolution_clock::now();
//    auto tree = mvbb::decompose3D(points, &mvbb, levels);
//    auto stop    = std::chrono::high_resolution_clock::now();
//    auto duration =  std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//
//    uint32_t pointsum = 0;
//    uint32_t nr_finalized_nodes = 0;
//    for(int k =0; k < tree.depth(); ++k){
//        uint32_t sk = 0;
//        for(auto& node : tree.getNodes(k)) {
//            if(node.final){
//                std::filesystem::path cell_path_name(
//                        "./pig_tree_lvl" + std::to_string(k) + "_" + std::to_string(sk) + ".off");
//                pointsum += node.points.size();
//
//                CGAL::Surface_mesh<Point> obb_sm;
//                auto obb_points = node.bounding_box.get_vertices();
//                auto box = CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
//                                                 obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);
//
//                if (!CGAL::IO::write_OFF(cell_path_name, obb_sm,
//                                         CGAL::parameters::stream_precision(6))) {
//                    std::cout  << "tree was not written, not sure why, maybe check path: "
//                               << std::filesystem::current_path();
//                }
//                ++sk;
//                ++nr_finalized_nodes;
//            }
//        }
//    }
//    std::cout << "The point cloud was partitioned in: " << nr_finalized_nodes << " bboxes \t  points : " << pointsum << " / " << points.size() << "(original)\n";
//    return 0;
//}

int main(int argc, char* args[]){
    auto path = "../../../python/tests/data/sample2_full.pcs";
    MvbbEvaluator eval;
    eval.set_pointcloud_path(path);
    eval.load_points();

 }