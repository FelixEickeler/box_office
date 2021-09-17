//
// Created by felix on 17.07.2021.
//

#include "rasterizer.h"
#include <algorithm>

std::array<float, 4> mvbb::minmax2D(const BBox &bbox, const CoordinateSystem2D &coordinate_system2D) {
    std::array<float, 4> min_max2D{1.0e10, -1.0e-10,1.0e-10,-1.0e-10};
    for(auto& v : bbox.get_vertices()){
        auto p2d = coordinate_system2D.project_onto_plane(v);
        auto x = p2d.x(); auto y = p2d.y();
        if(x < min_max2D[0]){
            min_max2D[0] = x;
        }
        else if(x > min_max2D[2]){
            min_max2D[2] = x;
        }
        if(y < min_max2D[1]){
            min_max2D[1] = y;
        }
        else if(y > min_max2D[3]){
            min_max2D[3] = y;
        }
    }
    return min_max2D;
}

size_t mvbb::Grid::raster() const {
    return data.innerSize();
}

//float mvbb::Grid::step_x() const {
//    return float(this->parent._scale[0]);
//}
//
//float mvbb::Grid::step_y() const {
//    return float(this->parent._scale[1]);
//}
//
//std::tuple<float, float> mvbb::Grid::step_sizes() const {
//    return {this->parent._scale[0], this->parent._scale[1]};
//}

Point2D mvbb::Space::from_grid(const Point2D &grid_point) {
    return grid_point.cwiseProduct(parent._scale) + parent._shift;
}

Point2D mvbb::Space::from_grid(const Eigen::Matrix<uint32_t, 2, 1> &grid_point) {
    return grid_point.cast<double>().cwiseProduct(parent._scale) + parent._shift;
}

float mvbb::Space::area() const {
    return (parent._min_max[2] - parent._min_max[0]) * (parent._min_max[3] - parent._min_max[1]);
}

mvbb::Rasterizer mvbb::Rasterizer::create_grid(std::array<float, 4> _min_max, long raster) {
    auto rasterizer =  Rasterizer(_min_max, raster);
    return rasterizer;
}

mvbb::Rasterizer mvbb::Rasterizer::create_grid(BBox &bbox, const CoordinateSystem2D &coordinate_system2D, long raster) {
    auto _min_max = minmax2D(bbox, coordinate_system2D);
    auto rasterizer =  Rasterizer(_min_max, raster);
    return rasterizer;
}

void mvbb::Rasterizer::insert(const Point2D &point2D) {
    const Vector2f compensate(0.5, 0.5);
    Eigen::Matrix<uint32_t, 2, 1> point_grid = ((point2D - _shift).cwiseQuotient(_scale) + compensate).cast<uint32_t>();
    grid.data()(point_grid.y(), point_grid.x()) += 1;
}

mvbb::Rasterizer::Rasterizer(const mvbb::Rasterizer &obj) : _data(obj._data),
                _raster(obj._raster),
                _min_max{obj._min_max},
                _scale(obj._scale),
                _shift(obj._shift),
                grid(*this), space(*this) {}

mvbb::Rasterizer::Rasterizer(std::array<float, 4> min_max, long raster) :  _data(MatrixXu::Zero(raster, raster)),
                                                                           _raster(raster),
                                                                           _min_max{min_max},
                                                                           _scale((_min_max[2] - _min_max[0]) / float(raster - 1),
                                                                                  (_min_max[3] - _min_max[1]) / float(raster - 1)),
                                                                           _shift(min_max[0], min_max[1]),
                                                                           grid(*this), space(*this) {}

mvbb::Rasterizer &mvbb::Rasterizer::operator=(mvbb::Rasterizer &other) {
    if (this != &other) {
        swap(*this, other);
    }
    return *this;
}

void mvbb::swap(Rasterizer &first, Rasterizer &second) {
        using std::swap;
        swap(first._data, first._data);
        swap(first._raster, first._raster);
        swap(first._min_max, first._min_max);
        swap(first._scale, second._scale);
        swap(first._shift, second._shift);
        swap(first.grid, second.grid);
        swap(first.space, second.space);
}





//
