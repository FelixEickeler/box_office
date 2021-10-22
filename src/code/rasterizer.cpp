//
// Created by felix on 17.07.2021.
//

#include "rasterizer.h"



Point2D mvbb::Discretization::grid2space(const Point2D &grid_point) const{
    return grid_point.cwiseProduct(_scale) + _shift;
}

Point2D mvbb::Discretization::grid2space(const Eigen::Matrix<uint32_t, 2, 1> &grid_point) const{
    return grid_point.cast<double>().cwiseProduct(_scale) + _shift;
}

mvbb::ProjectedLine mvbb::Discretization::grid2space(const GridLine &grid_line) const {
    return ProjectedLine{grid2space(grid_line.start), grid2space(grid_line.end)};
}

std::vector<mvbb::ProjectedLine> mvbb::Discretization::grid2space(const std::vector<GridLine> &grid_lines) const{
    std::vector<mvbb::ProjectedLine> projected_lines;
    projected_lines.reserve(grid_lines.size());
    std::transform(grid_lines.begin(), grid_lines.end(), std::back_inserter(projected_lines),
                   [this](const auto& gl)->mvbb::ProjectedLine{ return grid2space(gl);});
    return projected_lines;
}


float mvbb::Discretization::area() const {
    return (_min_max[2] - _min_max[0]) * (_min_max[3] - _min_max[1]);
}

mvbb::Discretization mvbb::Discretization::create_discretization(SplitStrategy& split_strategy, MinMax2f _min_max, long raster){
    auto rasterizer =  Discretization(split_strategy, _min_max, raster);
    return rasterizer;
}

mvbb::Discretization mvbb::Discretization::create_discretization(SplitStrategy& split_strategy, BBox &bbox, const CoordinateSystem2D &coordinate_system2D, long raster){
    auto _min_max = minmax2D(bbox, coordinate_system2D);
    auto rasterizer =  Discretization(split_strategy, _min_max, raster);
    return rasterizer;
}

void mvbb::Discretization::insert(const Point2D &point2D) {
   auto point_grid = ((point2D - _shift).cwiseQuotient(_scale)).cast<uint32_t>();
   auto t = point_grid.x();
   auto t2 = point_grid.y();
    _grid.data(point_grid.y(), point_grid.x()) += 1;
}


mvbb::Discretization::Discretization(SplitStrategy& splitStrategy, MinMax2f min_max, long raster) :  _grid(raster), _split_strategy(splitStrategy),
                                                                           _min_max{extend_boundaries(min_max)},
                                                                           _scale((_min_max[2] - _min_max[0]) / float(raster),
                                                                                  (_min_max[3] - _min_max[1]) / float(raster)),
                                                                           _shift(min_max[0], min_max[1]) {}

float mvbb::Discretization::step_x() const {
    return _scale[0];
}

float mvbb::Discretization::step_y() const {
    return _scale[1];
}

std::tuple<float, float> mvbb::Discretization::step_sizes() const {
    return {_scale[0], _scale[1]};
}

mvbb::XY_Grid& mvbb::Discretization::Grid(){
    return _grid;
}

mvbb::SpaceProxy mvbb::Discretization::Space() const {
    return SpaceProxy{std::bind(&Discretization::step_x, this),
                      std::bind(&Discretization::step_y, this),
                      std::bind(&Discretization::step_sizes, this)};
}

mvbb::ProjectedSplits mvbb::Discretization::best_splits()  const{
    ProjectedSplits projected_splits;
    // calculate all the splits with the given strategy
    auto best_grid_splits = _split_strategy.calculate_best_splits(_grid);
    double cell_area = step_x() * step_y();
    Vector2f origin = grid2space(Point2D(0, 0));

   for(const auto& orientation : GridOrientationEnumerator){
       auto os = best_grid_splits[orientation];
       projected_splits[orientation].emplace_back(
                   grid2space(os.cuts),
                   origin,
                   os.area * cell_area,
                   os.orientation);

   }
   return projected_splits;
}
