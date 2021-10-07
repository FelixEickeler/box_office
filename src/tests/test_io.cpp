//
// Created by felix on 20.07.2021.
//
#include <BoxScene.h>
#include "gtest/gtest.h"


TEST (Input , LoadPoints_XYZC_NoThrow) {
    auto paths = {"./files/bunny_classy_head.txt"};
    for(auto&path : paths){
        BoxScene eval;
        eval.set_pointcloud_path(path);
        eval.load_points();
        ASSERT_NO_THROW();
    }

}

// TODO: Write test
//TEST (io_testing , load_classes) {
//    auto paths = {"./files/bunny_classy_head.ol", "./files/bunny_classy_head.txt"};
//    for(auto&path : paths){
//        BoxScene eval;
//        eval.set_pointcloud_path(path);
//        eval.load_points();
//    }
//}

TEST (Input , CreateScene_XYZC_CorrectPointNumber) {
    auto eval = create_scene("./files/bunny_classy_head.txt",
                             "./files/bunny_classy_head.ol");
    ASSERT_EQ(eval.get_pointcloud().size(), 37706);
}

TEST (Input , Decompose3D_Bunny_NoThrow) {
    auto eval = create_scene("./files/bunny_classy_head.txt",
                             "./files/bunny_classy_head.ol");
   auto entity = eval.get_object(1);
   mvbb::CGAL_MVBB<boxy::pointcloud_xyzc> algo;
   auto epoints = entity.get_points();
   mvbb::decompose3D(epoints, &algo, mvbb::Target_Setting(1, 0.99));
   ASSERT_NO_THROW();
}



