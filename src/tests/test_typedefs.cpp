//
// Created by felix on 07.07.2021.
//
#include "gtest/gtest.h"
#include "typedefs.h"
#include <array>

TEST (typdef_testing /*test suite name*/, Cgal2Eigen_Point3_Vector3f /*test name*/) {
    boxy::Point point{1,2,3};
    auto eigen_vector = boxy::cgal_to_eigen(point);
    ASSERT_EQ(eigen_vector.x(), point.x());
    ASSERT_EQ(eigen_vector.x(), point.x());
    ASSERT_EQ(eigen_vector.x(), point.x());
}

TEST (typdef_testing /*test suite name*/, Cgal2Eigen_Vector3_Vector3f /*test name*/) {
    boxy::Vector_3 cgal_vector{1,2,3};
    auto eigen_vector = boxy::cgal_to_eigen(cgal_vector);
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
}

TEST (typdef_testing /*test suite name*/, Eigen2Cgal_Vector3f_Vector3 /*test name*/) {
    boxy::Vector3f eigen_vector{1,2,3};
    auto cgal_vector = boxy::eigen_to_cgal(eigen_vector);
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
}

std::tuple<std::array<boxy::Point, 9>, std::array<boxy::Vector2f,9>> Cube_Diag_Testpoints() {
    std::array<boxy::Point, 9> test_points = {{
          {0, 0, 0}, {1, 1, 0}, {2, 2, 0},
          {0, 0, 1}, {1, 1, 1}, {2, 2, 1},
          {0, 0, 2}, {1, 1, 2}, {2, 2, 2}
    }};
    std::array<boxy::Vector2f, 9> result_points = {{
          {2, 2 * sqrt(2)}, {2, sqrt(2)}, {2, 0},
          {1, 2 * sqrt(2)}, {1, sqrt(2)}, {1, 0},
          {0, 2 * sqrt(2)}, {0, sqrt(2)}, {0, 0},
    }};
    return {test_points, result_points};
}


TEST (typdef_testing /*test suite name*/, CooridnateSystem2D_Project2Plane /*test name*/) {
    // Suqare cut edges
    using namespace boxy;
    CoordinateSystem2D system{
        Vector3f{2,2,2}, Vector3f{0,0,-1}, Vector3f{-1,-1,0}.normalized()
    };
    //on diag plane
    auto [test_points, result_points] = Cube_Diag_Testpoints();

    for(auto i=0; i < test_points.size(); ++i){
        auto current_testpoint = system.project_onto_plane(test_points[i]);
        ASSERT_TRUE(current_testpoint.isApprox(result_points[i])) << "Diagnostics Point:" << i << "\t (" \
        << result_points[i].transpose() << ") != (" << current_testpoint.transpose() << ")";
    }
}

TEST (typdef_testing /*test suite name*/, CooridnateSystem2D_Project2Global/*test name*/) {
    // Suqare cut edges
    using namespace boxy;
    CoordinateSystem2D system{
            Vector3f{2,2,2}, Vector3f{0,0,-1}, Vector3f{-1,-1,0}.normalized()
    };
    //on diag plane
    auto [result_points, test_points] = Cube_Diag_Testpoints();

    for(auto i=0; i < test_points.size(); ++i){
        auto current_testpoint = system.project_to_global(test_points[i]);
        ASSERT_TRUE(current_testpoint == result_points[i]) << "Diagnostics Point:" << i << "\t (" \
        << result_points[i] << ") != (" << current_testpoint << ")";
    }
}

std::array<boxy::Point, 8> Get_CubePoints(){
    std::array<boxy::Point, 8> cube_points = {{
              {0,0,0}, {1,0,0}, {1,1,0},
              {0,1,0}, {0,1,1}, {0,0,1},
              {1,0,1}, {1,1,1},
      }};
    return cube_points;
}

TEST (typdef_testing /*test suite name*/, BBOX_get_plane_allFaces/*test name*/) {
    boxy::BBox bbox;
    auto cube_points = Get_CubePoints();
    bbox.vertices = cube_points;

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

TEST (typdef_testing /*test suite name*/, BBOX_minMax/*test name*/) {
    boxy::BBox bbox;
    auto cube_points = Get_CubePoints();
    bbox.vertices = cube_points;

    auto min_max = bbox.min_max();
    ASSERT_EQ(min_max[0], 0);
    ASSERT_EQ(min_max[1], 0);
    ASSERT_EQ(min_max[2], 0);
    ASSERT_EQ(min_max[3], 1);
    ASSERT_EQ(min_max[4], 1);
    ASSERT_EQ(min_max[5], 1);
}