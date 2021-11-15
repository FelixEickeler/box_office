//
// Created by felix on 08.10.21.
//
#include "gtest/gtest.h"
#include "typedefs.h"
#include "data.h"

TEST (BBox , GetPlane_CubePoints_AllPlanesDefinedCorrectly) {
    auto cube_points = Get_CubePoints();
    boxy::BBox bbox(cube_points);

    auto A = boxy::Plane_3(cube_points[0], cube_points[1], cube_points[5]);
    auto B = boxy::Plane_3(cube_points[3], cube_points[2], cube_points[1]);
    auto C = boxy::Plane_3(cube_points[5], cube_points[4], cube_points[3]);
    auto D = boxy::Plane_3(cube_points[4], cube_points[7], cube_points[2]);
    auto E = boxy::Plane_3(cube_points[4], cube_points[5], cube_points[6]);
    auto F = boxy::Plane_3(cube_points[2], cube_points[7], cube_points[6]);

    ASSERT_EQ(bbox.get_plane(boxy::BoxFaces::A), A);
    ASSERT_EQ(bbox.get_plane(boxy::BoxFaces::B), B);
    ASSERT_EQ(bbox.get_plane(boxy::BoxFaces::C), C);
    ASSERT_EQ(bbox.get_plane(boxy::BoxFaces::D), D);
    ASSERT_EQ(bbox.get_plane(boxy::BoxFaces::E), E);
    ASSERT_EQ(bbox.get_plane(boxy::BoxFaces::F), F);
}

TEST (BBox , MinMax_CubePoints_CorrectXYZAlignedBoundaries) {
    auto cube_points = Get_CubePoints();
    boxy::BBox bbox(cube_points);

    auto min_max = bbox.min_max();
    ASSERT_EQ(min_max[0], 0);
    ASSERT_EQ(min_max[1], 0);
    ASSERT_EQ(min_max[2], 0);
    ASSERT_EQ(min_max[3], 1);
    ASSERT_EQ(min_max[4], 1);
    ASSERT_EQ(min_max[5], 1);
}