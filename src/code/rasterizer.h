//
// Created by felix on 17.07.2021.
//

#ifndef BOXOFFICE_RASTERIZER_H
#define BOXOFFICE_RASTERIZER_H

#include "typedefs.h"
#include "rasterizer_definitions.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <functional>
#include "SplitStrategies.h"

using namespace boxy;
namespace mvbb {
    struct SpaceProxy{
        std::function<float()> step_x;
        std::function<float()> step_y;
        std::function<std::tuple<float, float>()> step_sizes;
    };

    class Discretization {
        private:
            XY_Grid _grid;
            SplitStrategy& _split_strategy;
            MinMax2f _min_max;
            Vector2f _scale;
            Vector2f _shift;

        public:
            static Discretization create_discretization(SplitStrategy& split_strategy, MinMax2f _min_max, long raster=512);
            static Discretization create_discretization(SplitStrategy& split_strategy, BBox& bbox, const CoordinateSystem2D& coordinate_system2D, long raster=512);
            explicit Discretization(SplitStrategy& splitStrategy, MinMax2f min_max, long raster);
            [[nodiscard]] float step_y() const;
            [[nodiscard]] float step_x() const;
            [[nodiscard]] std::tuple<float, float> step_sizes() const;
            [[nodiscard]] ProjectedSplits best_splits() const;
            [[nodiscard]] Point2D grid2space(const Point2D &grid_point) const;
            [[nodiscard]] Point2D grid2space(const Vector2i &grid_point) const;
            [[nodiscard]] ProjectedLine grid2space(const GridLine &grid_line) const;
            [[nodiscard]] std::vector<mvbb::ProjectedLine> grid2space(const std::vector<GridLine> &grid_lines) const;
            [[nodiscard]] float area() const;

            XY_Grid& Grid();
            [[nodiscard]] SpaceProxy Space() const;

            /// Inserts point in the grid, it uses integer casting to determine the position.
            /// However the grid (inserted coordinate) is shifted by 0.5 to compensate. <= not to sure
            /// \param point2D
            void insert(const Point2D &point2D);
    };
    #include "rasterizer.ipp"
}



#endif //BOXOFFICE_RASTERIZER_H
//      bool save_plane() {
//          // }
////              if (!CGAL::IO::write_OFF(cell_path_name, obb_sm,
////                                       CGAL::parameters::stream_precision(6))) {
////                  ASSERT_TRUE(false) << "tree0 was not written, not sure why, maybe check path: "
////                                     << std::filesystem::current_path();
////              }
//          return true;


/// This method is calculating the splits on the grid of the rasterizer. It is not transformed in an overall coordinate system !
/// \tparam TDirection :  true for row; false for column
/// \return BestGridSplit : rasterized position, no global space !
//            template<bool TDirection>
//            auto best_split();
//                bool save(const std::filesystem::path &path) {
//                    if (!exists(path.parent_path())) {
//                        throw std::runtime_error("Parent path does not exist");
//                    }
//                    std::ofstream file(path);
//                    file << "min_x: " << parent._min_max[0] << "min_y: " << parent._min_max[1] << "max_x: " << parent._min_max[2] << "max_y: "
//                         << parent._min_max[3] << "\n";
//                    file << "step_x: " << step_x() << "step_y: " << step_y() << "\n\n";
//                    file << parent._data.format(CSVFormat);
//                    return true;
//                }


//    class Space{
//        private:
//            std::reference_wrapper<Rasterizer>  parent;
//
//        public:
//            explicit Space(Rasterizer& r): parent(r) {}
//
//            Point2D from_grid(const Point2D &grid_point);
//
//            Point2D from_grid(const Eigen::Matrix<uint32_t, 2, 1> &grid_point);
//
//            [[nodiscard]] float area() const;
//
//
//            template<bool TDirection>
//            BestSplit best_split(float cell_depth = 1);
//    };
