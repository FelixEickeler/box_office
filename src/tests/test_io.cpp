//
// Created by felix on 20.07.2021.
//
#include <BoxScene.h>
#include "gtest/gtest.h"
#include "TargetSetting.h"


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
   TwoSplitStrategy split_strategy;
   auto target_setting = TargetSetting(2, 0.99,10,1000,"/home/boxy/dev/debug/bunny_test");
   spdlog::set_level(spdlog::level::trace);

//   target_setting.output_grids=true;
//   target_setting.output_cuts=true;
//   target_setting.output_boxes=true;
   mvbb::decompose3D(epoints, &algo, target_setting, split_strategy);
   ASSERT_NO_THROW();
}

TEST (Input /*test suite name*/, TakeFirstN_1to10_CorrectWords /*test name*/) {
    auto test_string = "The red-bellied black snake (Pseudechis-porphyriacus) is a species of elapid snake native to Australia";
    std::array<std::string, 14> result = {
            "The", "red-bellied", "black", "snake", "(Pseudechis-porphyriacus)", "is", "a", "species", "of", "elapid",
            "snake", "native", "to", "Australia"
    };
    {
        using namespace helpers;
        auto zero = take_first_n<0>(test_string, ' ');
        ASSERT_EQ(zero.size(), 0);

        auto first = take_first_n<1>(test_string, ' ');
        ASSERT_EQ(first[0], "The");

        auto fourteen = take_first_n<14>(test_string, ' ');
        for (auto word = 0; word < fourteen.size(); ++word) {
            ASSERT_EQ(fourteen[word], result[word]);
        }
    }
}


