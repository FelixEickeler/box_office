//
// Created by felix on 17.07.2021.
//

#ifndef BOXOFFICE_RASTERIZER_H
#define BOXOFFICE_RASTERIZER_H

#include "typedefs.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>


// Rasterizer constructs generate a discretization
// a dicretizaton has a grid
// a discretizatrion can convert grid information to normal information
// all calculus is done on grid after converstion

//const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t", "\n");
using namespace boxy;
namespace mvbb {
    std::array<float, 4> minmax2D(const BBox& bbox, const CoordinateSystem2D& coordinate_system2D);
    class Rasterizer;

    struct Grid{
        MatrixXu data;
        explicit Grid(long raster) : data(MatrixXu::Zero(raster, raster)){}
        [[nodiscard]] size_t raster() const;
        template<bool TDirection>

        /// This method returns a vector of spans representing the occupied space of the grid.
        /// It will be either provide these as column : direciton_x(false)  or row : direction_y(true) representations
        /// \tparam TDirection :  true for row; false for column
        /// \return Eigen::Vector2i<TRaster : v(n,0) start of the span and v(n,1) end of it
        auto get_first_and_last_slot();






            /// This method is calculating the splits on the grid of the rasterizer. It is not transformed in an overall coordinate system !
            /// \tparam TDirection :  true for row; false for column
            /// \return BestGridSplit : rasterized position, no global space !
            template<bool TDirection>
            auto best_split();
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
    };

    class Space{
        private:
            std::reference_wrapper<Rasterizer>  parent;

        public:
            explicit Space(Rasterizer& r): parent(r) {}

            Point2D from_grid(const Point2D &grid_point);

            Point2D from_grid(const Eigen::Matrix<uint32_t, 2, 1> &grid_point);

            friend void swap(Space& first, Space& second);


            [[nodiscard]] float area() const;

            /// direction 0 => first orientation, direction true(1) => second orientation:
            /// This is a little hacky but is my first constexpr !
            /// Ah btw this is not transformed in the a real coordinate system (e.g. cutting plane)
            template<bool TDirection>
            BestSplit best_split(float cell_depth = 1);
    };

    class Rasterizer {
        private:
//            Eigen::Matrix<uint32_t , 512, 512> _data = Eigen::Matrix<uint32_t, 512,512>::Zero();//(TRaster, TRaster);

            long _raster;
            std::array<float, 4> _min_max;
            Vector2f _scale;
            Vector2f _shift;
            friend class Grid;
            friend class Space;

    public:
        Grid grid;
        Space space;

        static Rasterizer create_grid(std::array<float, 4> _min_max, long raster=512);

        static Rasterizer create_grid(BBox& bbox, const CoordinateSystem2D& coordinate_system2D, long raster=512);

        explicit Rasterizer(std::array<float, 4> min_max, long raster);

        Rasterizer( const Rasterizer &obj);

        Rasterizer& operator=(Rasterizer& other);

        friend void swap(Rasterizer& first, Rasterizer& second);

        [[nodiscard]] const MatrixXu &data() const;

        [[nodiscard]] uint &get(uint32_t x, uint32_t y);

        // TODO Move to common
        [[nodiscard]] float step_x() const;

        [[nodiscard]] float step_y() const;

        [[nodiscard]] std::tuple<float, float> step_sizes() const;

        /// Inserts point in the grid, it uses integer casting to determine the position. However the grid (inserted coordinate) is shifted by 0.5 to compensate.
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