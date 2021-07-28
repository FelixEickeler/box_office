//
// Created by felix on 18.06.2021.
//
#define GTEST_COUT std::cerr << "[          ] [ INFO ]"
#ifndef TEST_BIND_ALGORITHM_H
#define TEST_BIND_ALGORITHM_H
#include "typedefs.h"
#include "helpers.h"
#include "rasterizer.h"
#include <functional>
#include <algorithm>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/bounding_box.h>
#include <iterator>
#include <fstream>
#include <CGAL/IO/write_off_points.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Origin.h>
#include <filesystem>
#include "FitAndSplitHierarchy.h"

using namespace boxy;
const bool direction_ex = false;
const bool direction_ey = true;


namespace mvbb {

    template<class TPointCloudType>
    class Algo_Base {
        public:
            virtual ~Algo_Base() {};
            using iterator = typename TPointCloudType::iterator;

            virtual BBox fit_bounding_box(
                    TPointCloudType &points3D) const = 0;  // This method is not implemented in the base class, making it a pure virtual method. Subclasses must implement it
    };

    template<class TPointCloudType>
    class CGAL_MVBB : public Algo_Base<TPointCloudType> {
        public:
            ~CGAL_MVBB() = default;

            BBox fit_bounding_box(TPointCloudType &points3D) const;
    };

    template<class TPointCloudType>
    BBox CGAL_MVBB<TPointCloudType>::fit_bounding_box(TPointCloudType &points3D) const {
        // TODO figure out how to generate pass the view to the oriented_bounding_box
        boxy::VectorView input(points3D.begin(), points3D.end());
        std::vector<Point> clean_points;
        for(auto& p : points3D){
            clean_points.push_back(std::get<0>(p));
        }
        std::array<Point, 8> vertices;
        if (points3D.size() > 10) {
            try {
                CGAL::oriented_bounding_box(clean_points, vertices);//, CGAL::parameters::point_map(boxy::Point_map())); //CGAL::parameters::use_convex_hull(true).
            }
            catch (std::exception &e) {
                std::cout << e.what() << '\n';
            }
        }
        return BBox(vertices);
    }
//
//    template<class TPointCloudType>
//    class Algo_PCA : Algo_Base<TPointCloudType>{
//       BBox fit_bounding_box(TPointCloudType& points3D);
//    };
//
//    template<class TPointCloudType>
//    BBox Algo_PCA<TPointCloudType>::fit_bounding_box(TPointCloudType &points3D) {
//        // TODO: write test for pca fit
//        BBox tmp;
//        CGAL::Simple_cartesian<float>::Iso_cuboid_3 c3 = CGAL::bounding_box(points3D->begin(), points3D->end());
//        for(auto i=0; i < 8; ++i){
//            tmp.vertices[i] = c3.vertex(i);
//        }
//        return tmp;
//    }



//    template<class TPointCloudType>
    using TPointCloudType = pointcloud_xyzc;
    FitAndSplitHierarchy<TPointCloudType> decompose3D(TPointCloudType &points3D, Algo_Base<TPointCloudType> *bbox_algorithm,
                                                      uint32_t kappa, float gain_threshold = 0.99);

}
#endif