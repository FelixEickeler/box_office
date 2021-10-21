//
// Created by felix on 20.10.21.
//

#ifndef BOXOFFICE_RESULT_WRITER_H
#define BOXOFFICE_RESULT_WRITER_H
//#include <utility>
#include "typedefs.h"
#include "rasterizer_definitions.h"
#include "CGAL/Surface_mesh.h"
#include "rasterizer.h"
#include "FitAndSplitHierarchy.h"


using namespace boxy;
std::tuple<CGAL::Surface_mesh<Point>, std::array<Point,4>> Plane2Mesh(Point o1, Point p1, Point p2, bool swap_normal_direction=false);


namespace ResultType {
    struct Plane {};
    struct Grid{};
    struct BoundingBox{};
    struct Vector{};
    struct PointCloud{};
}



class ResultWriter{

    public:
        ResultWriter(const Path base_output_path){};
        virtual void set_path_addendum(Path& addendum_){};
        virtual bool write(ResultType::Plane plane, const mvbb::ProjectedSplit& best_split,
                           const CoordinateSystem2D& coordinate_system2D){return true;};
        virtual bool write(ResultType::Grid Grid, Discretization& discretization,
                           CoordinateSystem2D coordinate_system2D) const{return true;};

//        template<class TPointCloudType>
//        bool write(FitAndSplitNode<TPointCloudType> node) const {return true;}
        // Non-Templated
        virtual bool write(ResultType::BoundingBox box, BBox bounding_box, bool is_final){return true;};

        virtual bool write(ResultType::BoundingBox box, BBox bounding_box){return true;};
//        virtual ~ResultWriter(){}
};

class ResultWriterFs: public ResultWriter{
    private:
        Path base_path;
        Path addendum;
        static bool _write_bbox(BBox& bounding_box, Path path) ;

    public:
        ResultWriterFs(Path base_output_path);

        void setup();

        void set_path_addendum(Path& addendum_) override;

        bool write(ResultType::Plane plane, const mvbb::ProjectedSplit& best_split,
                   const CoordinateSystem2D& coordinate_system2D) override;

        bool write(ResultType::Grid grid, Discretization& discretization,
                   CoordinateSystem2D coordinate_system2D) const override;

        bool write(ResultType::BoundingBox box, BBox bounding_box) override;
        bool write(ResultType::BoundingBox box, BBox bounding_box, bool is_final) override;

//        ~ResultWriterFs() override {};
};

std::unique_ptr<ResultWriter> make_result_writer(const Path& base_output_path);


//void output_planes(const TargetSetting &target_settings, int depth, int node_counter, ProjectedSplit &best_split,
//                   const BoxFaces &selected_face, CoordinateSystem2D coordinate_system2D);

#endif //BOXOFFICE_RESULT_WRITER_H