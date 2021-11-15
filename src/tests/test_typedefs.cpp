//
// Created by felix on 07.07.2021.
//
#include "gtest/gtest.h"
#include "typedefs.h"
#include "data.h"

TEST (TypeDefinitions_CastingAndConversion , Cgal2Eigen_Point3_Vector3f ) {
    boxy::Point point{1,2,3};
    auto eigen_vector = boxy::cgal_to_eigen(point);
    ASSERT_EQ(eigen_vector.x(), point.x());
    ASSERT_EQ(eigen_vector.x(), point.x());
    ASSERT_EQ(eigen_vector.x(), point.x());
}

TEST (TypeDefinitions_CastingAndConversion , Cgal2Eigen_Vector3_Vector3f ) {
    boxy::Vector_3 cgal_vector{1,2,3};
    auto eigen_vector = boxy::cgal_to_eigen(cgal_vector);
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
}

TEST (TypeDefinitions_CastingAndConversion , Eigen2Cgal_Vector3f_Vector3 ) {
    boxy::Vector3f eigen_vector{1,2,3};
    auto cgal_vector = boxy::eigen_to_cgal(eigen_vector);
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
    ASSERT_EQ(eigen_vector.x(), cgal_vector.x());
}


TEST (CooridnateSystem2D , ProjectOntoPlane_TestPoints_CorrectPlaneCordinates ) {
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

TEST (CooridnateSystem2D , CooridnateSystem2D_Project2Global_CorrectPointCordinatesInSpace) {
    // Suqare cut edges
    using namespace boxy;
    CoordinateSystem2D system{
            Vector3f{2,2,2}, Vector3f{0,0,-1}, Vector3f{-1,-1,0}.normalized()
    };
    //on diag plane
    auto [result_points, test_points] = Cube_Diag_Testpoints();

    for(auto i=0; i < test_points.size(); ++i){
        auto current_testpoint = system.project_to_global(test_points[i]);
        ASSERT_TRUE((current_testpoint-result_points[i]).squared_length() < 1e-10 ) << "Diagnostics Point:" << i << "\t (" \
        << result_points[i] << ") != (" << current_testpoint << ")";
    }
}

TEST (TypeDefinitions_Ranges, VectorView_CubePoints_SameAdress) {
    boxy::pointcloud_xyzc pc;
    for(auto p : Get_CubePoints()){
        pc.push_back(std::make_tuple(p,0));
    }
    boxy::VectorView cr(pc.cbegin(), pc.cend());
    ASSERT_EQ(std::distance(cr.begin(), pc.cbegin()), 0);
    ASSERT_EQ(std::distance(cr.end(), pc.cend()), 0);
}

