//
// Created by felix on 19.09.2021.
//

#ifndef BOXOFFICE_RASTERIZER_DEFINITIONS_H
#define BOXOFFICE_RASTERIZER_DEFINITIONS_H
#include "typedefs.h"
//#include "mvbb_algorithms.h"
#include <exception>

using namespace boxy;
namespace mvbb{
    struct MinMax2f {
        float min_x;
        float min_y;
        float max_x;
        float max_y;
        MinMax2f(float min_x_, float min_y_, float max_x_, float max_y_);
        MinMax2f(std::array<float,4>);
        float &operator[](int _acc);
        float operator[](int _acc) const;
    };

    MinMax2f& extend_boundaries(MinMax2f& minmax);

    MinMax2f minmax2D(const BBox& bbox, const CoordinateSystem2D& coordinate_system2D);
    enum GridOrientation : bool { X = true, Y = false};
    constexpr std::array<GridOrientation, 2> GridOrientationEnumerator = {GridOrientation::X, GridOrientation::Y};

    struct GridLine {
        Vector2i start;
        Vector2i end;
    };

    struct GridSplit{
        std::vector<GridLine> cuts;
        int area;
        std::vector<int> index;
        GridOrientation orientation;
    };

    struct GridSplits {
        GridSplit x;
        GridSplit y;

        GridSplit& operator[](GridOrientation orientation){
            if (orientation == GridOrientation::X)
                return x;
            return y;
        }
    };

    struct ProjectedLine {
        Vector2f start;
        Vector2f end;
    };

    struct ProjectedSplit{
        std::vector<ProjectedLine> cuts;
        Vector2f origin;
        double area;
        GridOrientation orientation;
        ProjectedSplit(ProjectedLine _line, Vector2f _origin, double _area, GridOrientation _orientation) : cuts({_line}), origin(_origin), area(_area), orientation(_orientation){};
        ProjectedSplit(std::vector<ProjectedLine> _lines, Vector2f _origin, double _area, GridOrientation _orientation) : cuts(_lines), origin(_origin), area(_area), orientation(_orientation){};
        ProjectedSplit() : cuts({}), origin(), area() {};

        using CutListSorted = std::vector<Plane_3>;
        const CutListSorted project_cuts_and_create_planes(const CoordinateSystem2D& back_projection, std::filesystem::path debug_output_planes="");
    };

    const auto min_area  = [](const auto &a, const auto &b) {return a.area < b.area;};

    struct ProjectedSplits{
        std::vector<ProjectedSplit> x;
        std::vector<ProjectedSplit> y;

        std::vector<ProjectedSplit>& operator[](GridOrientation orientation){
            if (orientation == GridOrientation::X)
                return x;
            return y;
        }

        [[nodiscard]] std::vector<ProjectedSplit>::const_iterator best_x() const;

        [[nodiscard]] std::vector<ProjectedSplit>::const_iterator best_y() const;

        [[nodiscard]] std::vector<ProjectedSplit>::const_iterator best_xy() const;
    };

    struct BoxSplits{
        std::array<ProjectedSplits, 3> _box_splits;

        // operator function returns reference (&) to type
        // Overloading [] operator to access elements in array style
        ProjectedSplits& operator[](BoxFaces face){
            return _box_splits[to_underlying(face)];
        }

        struct Return_Best_Split{
            ProjectedSplit best_split;
            BoxFaces on_face;
        } ;

        Return_Best_Split superior_split();
    };

    //void output_planes(const mvbb::TargetSetting &target_settings, int depth, int node_counter, ProjectedSplit &best_split,
    //              const BoxFaces &selected_face, unknown coordinate_system2D);
}


#endif //BOXOFFICE_RASTERIZER_DEFINITIONS_H
