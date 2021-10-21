//
// Created by felix on 19.09.2021.
//
#include "rasterizer_definitions.h"
#include <exception>
#include <functional>

mvbb::MinMax2f mvbb::minmax2D(const BBox &bbox, const CoordinateSystem2D &coordinate_system2D) {
    mvbb::MinMax2f min_max2D{1.0e10, -1.0e-10,1.0e-10,-1.0e-10};
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

mvbb::MinMax2f &mvbb::extend_boundaries(mvbb::MinMax2f &minmax) {
    minmax[2] = std::nextafter(minmax[2], +std::numeric_limits<float>::infinity());
    minmax[3] = std::nextafter(minmax[3], +std::numeric_limits<float>::infinity());
    return minmax;
}

std::vector<mvbb::ProjectedSplit>::const_iterator mvbb::ProjectedSplits::best_x() const{
    return std::min_element(x.begin(), x.end(), min_area);
}

std::vector<mvbb::ProjectedSplit>::const_iterator mvbb::ProjectedSplits::best_y() const {
    return std::min_element(y.begin(), y.end(), min_area);
}

std::vector<mvbb::ProjectedSplit>::const_iterator mvbb::ProjectedSplits::best_xy() const{
    auto min_x = best_x();
    auto min_y = best_y();
    return min_area(*min_x, *min_y) ? min_x : min_y;
}

mvbb::BoxSplits::Return_Best_Split mvbb::BoxSplits::superior_split() {
    // Which is the best split
    if(_box_splits.empty()) {
        throw std::logic_error("Something went wrong, the splits are empty");
    }
    Return_Best_Split bsr{_box_splits.front().x.front(), BoxFaces::A};
    //technically this is not correct, as A could also be D, B could be E, ...
    std::array<BoxFaces, 3> face_enum = {BoxFaces::A, BoxFaces::B, BoxFaces::C};
    for (auto face: face_enum) {
        auto split_on_face = _box_splits[face];
        auto best_x = split_on_face.best_x();
        auto best_y = split_on_face.best_y();
        auto best_both = min_area(*best_x, *best_y) ? best_x : best_y;

        if (bsr.best_split.area > best_both->area) {
            bsr.best_split = *best_both;
            bsr.on_face = face;
        }
    }
    return bsr;
}

float &mvbb::MinMax2f::operator[](int _acc) {
    switch (_acc) {
        case 0:
            return min_x;
        case 1:
            return min_y;
        case 2:
            return max_x;
        case 3:
            return max_y;
        default:
            throw std::out_of_range(
                    "MinMax2f only hase 4 possible values: 0 => min_x, 1 => min_y, 2 => max_x, 3 => max_y");
    }
}

float mvbb::MinMax2f::operator[](int _acc) const {
    switch (_acc) {
        case 0:
            return min_x;
        case 1:
            return min_y;
        case 2:
            return max_x;
        case 3:
            return max_y;
        default:
            throw std::out_of_range(
                    "MinMax2f only hase 4 possible values: 0 => min_x, 1 => min_y, 2 => max_x, 3 => max_y");
    }
}

mvbb::MinMax2f::MinMax2f(float min_x_, float min_y_, float max_x_, float max_y_) : min_x(min_x_), min_y(min_y_), max_x(max_x_),
                                                                                   max_y(max_y_) {}

mvbb::MinMax2f::MinMax2f(std::array<float, 4> arr_minmax) : min_x(arr_minmax[0]), min_y(arr_minmax[1]), max_x(arr_minmax[2]),
                                                 max_y(arr_minmax[3]) {}

const mvbb::ProjectedSplit::CutListSorted
mvbb::ProjectedSplit::project_cuts_and_create_planes(const CoordinateSystem2D &back_projection,
                                                     std::filesystem::path debug_output_planes_path) {

    CutListSorted cutting_planes;
    std::transform(cuts.begin(), cuts.end(), std::back_inserter(cutting_planes),
                   [back_projection, this](const auto& cut){
                        auto p2 = back_projection.project_to_global(cut.end);
                        auto o1 = back_projection.project_to_global(cut.start);
                        auto p1 = back_projection.project_to_global(origin);
                        // generate plane normal orthogonal
                        auto orthogal_vector = CGAL::cross_product(p1 - o1, p2 - o1);
                        // create a plane that cuts the discretized plane orthogonal
                        return Plane_3(o1, p2, o1 + orthogal_vector);
                    });
    return cutting_planes;
}
