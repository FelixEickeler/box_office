//
// Created by felix on 20.07.2021.
//
#include <Evaluator.h>
#include "gtest/gtest.h"


TEST (io_testing /*test suite name*/, load_points/*test name*/) {
    MvbbEvaluator eval;
    eval.set_pointcloud_path("../../../python/tests/data/head_test.pcs");
    eval.load_points();

}
