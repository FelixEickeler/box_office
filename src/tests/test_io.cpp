//
// Created by felix on 20.07.2021.
//
#include <Evaluator.h>
#include "gtest/gtest.h"


TEST (io_testing /*test suite name*/, load_points/*test name*/) {
    auto paths = {"../../../python/tests/data/head_test.pcs", "../../../python/tests/data/sample2_full.pcs"};
    for(auto&path : paths){
        MvbbEvaluator eval;
        eval.set_pointcloud_path(path);
        eval.load_points();
    }

}

TEST (io_testing /*test suite name*/, load_classes/*test name*/) {
    auto paths = {"../../../python/tests/data/head_test.ol", "../../../python/tests/data/sample2_full.pcs"};
    for(auto&path : paths){
        MvbbEvaluator eval;
        eval.set_pointcloud_path(path);
        eval.load_points();
    }
}

TEST (io_testing /*test suite name*/, create_mvbb/*test name*/) {
    auto eval = create_mvbbevaluator("../../../python/tests/data/head_test.pcs",
                                     "../../../python/tests/data/head_test.ol");
    ASSERT_EQ(eval.get_pointcloud().size(), 155);
}

TEST (io_testing /*test suite name*/, decompose_2/*test name*/) {
    auto eval = create_mvbbevaluator("../../../python/tests/data/sample2_full.pcs",
                                     "../../../python/tests/data/sample2_full.ol");
   auto entity = eval.get_object(1);
   mvbb::CGAL_MVBB<boxy::pointcloud_xyzc> algo;
   auto epoints = entity.get_points();
   mvbb::decompose3D(epoints, &algo, 1, 0.99);
}



