//
// Created by felix on 07.07.2021.
//
#include "gtest/gtest.h"
#include "typedefs.h"


TEST (typdef_testing /*test suite name*/, Cgal2Eigen_conversion /*test name*/) {
    boxy::Point point{1,2,3};
    auto eigen_vector = boxy::cgal_to_eigen(point);
    ASSERT_EQ(eigen_vector.x(), point.x());
    ASSERT_EQ(eigen_vector.x(), point.x());
    ASSERT_EQ(eigen_vector.x(), point.x());

}

TEST (typdef_testing /*test suite name*/, Eigen2Cgal_conversion /*test name*/) {
    boxy::Vector3f eigen_vector{1,2,3};
    auto cgal_vector = boxy::eigen_to_cgal(eigen_vector);
    ASSERT_EQ(eigen_vector.x(), eigen_vector.x());
    ASSERT_EQ(eigen_vector.x(), eigen_vector.x());
    ASSERT_EQ(eigen_vector.x(), eigen_vector.x());
}