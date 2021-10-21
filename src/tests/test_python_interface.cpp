//Python
// Created by felix on 08.10.21.
//
#include "gtest/gtest.h"
#include "mvbb_algorithms.h"
#include "TargetSetting.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

//TODO This ist more a functional test
TEST (Python , Decompose3D_Pig_CorrectNumberOffSubBoxes) {
    mvbb::CGAL_MVBB<pointcloud_xyzc> mvbb;
    std::filesystem::path source_pig("./files/pig.off");
    CGAL::Surface_mesh<Point> sm;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(source_pig, sm) || sm.is_empty()) {
        ASSERT_TRUE(false) << "Testfile not found.";
    }
    pointcloud_xyzc points;
    for (const auto &v : vertices(sm)) {
        points.push_back(std::make_tuple(sm.point(v), 0));
    }

    uint32_t levels = 3;
    TwoSplitStrategy two_split_strategy;
    TargetSetting output_setting(levels,0.99, 10, 1000, "/home/boxy/dev/debug/planes");
    auto tree = mvbb::decompose3D(points, &mvbb, output_setting, two_split_strategy);
    for(int k =0; k <= levels; ++k){
        uint32_t sk = 0;
        for(auto& node : tree.getNodes(k)) {
            std::filesystem::path cell_path_name( "./pig_lvl" + std::to_string(k) + "_" + std::to_string(sk) + ".off");
            CGAL::Surface_mesh<Point> obb_sm;
            auto obb_points = node.bounding_box.get_vertices();
            auto box = CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                                             obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);

            if (!CGAL::IO::write_OFF(cell_path_name, obb_sm, CGAL::parameters::stream_precision(6))) {
                ASSERT_TRUE(false) << "the tree was not written, not sure why, maybe check path: "
                                   << std::filesystem::current_path();
            }
            ++sk;
        }
    }
}

