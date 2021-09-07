//
// Created by felix on 17.07.2021.
//

#ifndef BOXOFFICE_RASTERIZER_H
#define BOXOFFICE_RASTERIZER_H

//#include "mvbb_algorithm.h"
#include "typedefs.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

//const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", "\n");
using namespace boxy;
namespace mvbb {

    template<size_t TRaster>
    class Rasterizer {
        MatrixXu grid = MatrixXu::Zero(TRaster, TRaster);
//        Eigen::Matrix<uint32_t ,TRaster,TRaster> grid = Eigen::Matrix<uint32_t, TRaster,TRaster>::Zero();//(TRaster, TRaster);

        std::array<float, 4> min_max;
        Vector2f scale;
        Vector2f shift;

    public:
        explicit Rasterizer(std::array<float, 4> _min_max) : min_max{_min_max},
                                                             scale((_min_max[2] - _min_max[0]) / (TRaster - 1),
                                                                   (_min_max[3] - _min_max[1]) / (TRaster - 1)),
                                                             shift(_min_max[0], _min_max[1]) {}

        float step_x() {
            return scale[0];
        }

        float step_y() {
            return scale[1];
        }

        [[nodiscard]] size_t raster() const {
            return TRaster;
        }

        [[nodiscard]] std::tuple<float, float> step_sizes() const {
            return {scale[0], scale[1]};
        }

        [[nodiscard]] const auto &get_grid() const {
            return grid;
        }

        [[nodiscard]] const auto &get(uint32_t x, uint32_t y) {
            return grid(y, x);
        }

        Point2D from_grid(const Point2D &grid_point) {
            return grid_point.cwiseProduct(scale) + shift;
        }

        Point2D from_grid(const Eigen::Matrix<uint32_t, 2, 1> &grid_point) {
            return grid_point.cast<double>().cwiseProduct(scale) + shift;
        }

        /// Inserts point in the grid, it uses integer casting to deterimin the position. However the grid ('inserted coorindate) is shifted by 0.5 to compensate.
        /// \param point2D
        void insert(const Point2D &point2D) {
//                auto x_coord = static_cast<uint32_t>((point2D.x() - min_max[0]) / step_x());
//                auto y_coord = static_cast<uint32_t>((point2D.y() - min_max[1]) / step_y());
//                grid(y_coord, x_coord) += 1;
            const Vector2f compensate(0.5, 0.5);
            Eigen::Matrix<uint32_t, 2, 1> point_grid = ((point2D - shift).cwiseQuotient(scale) +
                                                        compensate).cast<uint32_t>();
            grid(point_grid.y(), point_grid.x()) += 1;
        }

//        bool save_grid(const std::filesystem::path &path) {
//            if (!exists(path.parent_path())) {
//                throw std::runtime_error("Parent path does not exist");
//            }
//            std::ofstream file(path);
//            file << "min_x: " << min_max[0] << "min_y: " << min_max[1] << "max_x: " << min_max[2] << "max_y: "
//                 << min_max[3] << "\n";
//            file << "step_x: " << step_x() << "step_y: " << step_y() << "\n\n";
//            file << grid.format(CSVFormat);
//            return true;
//        }

//        bool save_plane() {
//            // }
////                if (!CGAL::IO::write_OFF(cell_path_name, obb_sm,
////                                         CGAL::parameters::stream_precision(6))) {
////                    ASSERT_TRUE(false) << "tree0 was not written, not sure why, maybe check path: "
////                                       << std::filesystem::current_path();
////                }
//            return true;
//        }

        /// This method returns a vector of spans represneting the occupoied space of the grid.
        /// It will be either provide these as column : direciton_x(false)  or row : direction_y(true) represntations
        /// \tparam TDirection :  true for row; false for column
        /// \return Eigen::Vector2i<TRaster : v(n,0) start of the span and v(n,1) end of it
        template<bool TDirection>
        auto get_first_and_last_slot() {
            MatrixXu _span = MatrixXu::Zero(2, TRaster);

            for (auto i = 0; i < TRaster; ++i) {
                bool first_set = false;
                for (auto j = 0; j < TRaster; ++j) {
                    if constexpr(TDirection) {
                        if (grid(i, j) != 0) {
                            if (!first_set) {
                                _span(0, i) = j;
                                first_set = true;
                            }
                            _span(1, i) = j + 1;
                        }
                    } else {
                        if (grid(j, i) != 0) {
                            if (!first_set) {
                                _span(0, i) = j;
                                first_set = true;
                            }
                            _span(1, i) = j + 1;
                        }
                    }
                }
            }
            return _span;
        }

        float area() const {
            return (min_max[2] - min_max[0]) * (min_max[3] - min_max[1]);
        }


        /// This method is calculating the splits on the grid of the rasterizer. It is not transformed in an overall coordinate system !
        /// \tparam TDirection :  true for row; false for column
        /// \return BestGridSplit : rasterized position, no global space !
        template<bool TDirection>
        auto best_split_rasterized() {
            auto slot_sizes = get_first_and_last_slot<TDirection>();
            MatrixXu zero_stopper = (slot_sizes.row(1).array() == 0).template cast<uint32_t>() * UINT32_MAX;
            zero_stopper += slot_sizes.row(0);
            Eigen::Matrix<uint32_t, 2, 2> bbox_boundaries = Eigen::Matrix<uint32_t, 2, 2>::Zero();

            auto min_area = UINT32_MAX;
            uint32_t idx_min_area = 0;

            // cut between 0 and 1 of grid, last row does not after
            for (auto idx = 0; idx < slot_sizes.cols() - 1; ++idx) {
                uint32_t wbbox2 = TRaster - idx - 1;
                if (bbox_boundaries(0, 0) > slot_sizes(0, idx)) bbox_boundaries(0, 0) = slot_sizes(0, idx);
                if (bbox_boundaries(1, 0) < slot_sizes(1, idx)) bbox_boundaries(1, 0) = slot_sizes(1, idx);

                // only evaluate non zero:zero columns, for minimum => zero_stopper!
                bbox_boundaries(0, 1) = zero_stopper.topRightCorner(1, wbbox2).minCoeff();
                bbox_boundaries(1, 1) = slot_sizes.bottomRightCorner(1, wbbox2).maxCoeff();

                auto area = (idx + 1) * (bbox_boundaries(1, 0) - bbox_boundaries(0, 0)) +
                            wbbox2 * (bbox_boundaries(1, 1) - bbox_boundaries(0, 1));

                if (area < min_area) {
                    min_area = area;
                    idx_min_area = idx;
                }
            }
            return BestGridSplit{min_area, idx_min_area};
        }

        // direction 0 => first orientation, direction true(1) => second orientation:
        // This is a little hacky but is my first constexpr !
        // Ah btw this is not transformed in the a real coordinate system (e.g. cutting plane)
        template<bool TDirection>
        BestSplit best_split(float cell_depth = 1) {
            auto bgs = best_split_rasterized<TDirection>();
            float cell_area = step_x() * step_y();
            Vector2f line{0, TRaster - 1};

            Eigen::Matrix<uint32_t, 2, 1> begin_cut_grid;
            Eigen::Matrix<uint32_t, 2, 1> end_cut_grid;
            if constexpr (TDirection) { // true being direction_ey
                begin_cut_grid << 0, bgs.index;
                end_cut_grid << TRaster - 1, bgs.index;
            } else {
                begin_cut_grid << bgs.index, 0;
                end_cut_grid << bgs.index, TRaster - 1;
            }
            auto begin_cut = from_grid(begin_cut_grid);
            auto end_cut = from_grid(end_cut_grid);
            // TODO another quickfix
            auto origin = from_grid(Point2D(0, 0));
            return BestSplit{bgs.area * cell_area, begin_cut, end_cut, origin};
        }
    };

    std::array<float, 4> minmax2D(const BBox& bbox, const CoordinateSystem2D& coordinate_system2D);

}

#endif //BOXOFFICE_RASTERIZER_H
