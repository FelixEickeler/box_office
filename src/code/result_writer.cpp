//
// Created by felix on 20.10.21.
//
#include "result_writer.h"
#include <spdlog/spdlog.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/Surface_mesh.h>


using namespace mvbb;

ResultWriterFs::ResultWriterFs(const Path base_output_path) : ResultWriter(Path("")), base_path(base_output_path) {
    setup();
}

void ResultWriterFs::setup() {
    create_directories(base_path);
}

bool ResultWriterFs::write(ResultType::Plane plane, const ProjectedSplit &best_split, const CoordinateSystem2D &coordinate_system2D) {
    auto cut_counter = 0;
    for (auto &cp: best_split.cuts) {
        auto p2 = coordinate_system2D.project_to_global(cp.end);
        auto o1 = coordinate_system2D.project_to_global(cp.start);
        auto p1 = coordinate_system2D.project_to_global(best_split.origin);
        auto[plane_mesh, plane_system] = Plane2Mesh(o1, p1, p2, best_split.orientation == Y);

        // write final plane to .off
        auto plane_path = base_path / "cutting_planes" /  (this->addendum.string() + "cut=_" + (std::to_string(cut_counter++) +".off"));
        create_directories(plane_path.parent_path());
        if (!CGAL::IO::write_OFF(plane_path, plane_mesh, CGAL::parameters::stream_precision(6))) {
            std::cout << "The number of vertices is: " << plane_mesh.vertices().size() << "\n";
            throw std::runtime_error("not found");
        }
    }
    return false;
}

bool ResultWriterFs::write(ResultType::Grid, Discretization &discretization,
                           CoordinateSystem2D coordinate_system2D) const {
    std::vector<Point> after_transform; // debug
    auto grid =  discretization.Grid();
    auto dim = grid.raster();
    for(auto i=0; i < dim; ++i){
        for(auto j=0; j < dim; ++j){
            if(grid.data(i,j) > 0) {
                Point2D p2d_grid;
                p2d_grid << i, j;
                auto p2d = discretization.grid2space(p2d_grid);
                auto gp = coordinate_system2D.project_to_global(p2d);
                after_transform.push_back(gp);
            }
        }
    }
    std::filesystem::path grid_path(base_path / "grids" / fmt::format("grid_{}.xyz", addendum.c_str()));
    create_directories(grid_path.parent_path());
    if (!CGAL::IO::write_points(grid_path, after_transform, CGAL::parameters::stream_precision(6))) {
        spdlog::critical("The grid from node: {} \twith {}, could not be written to the given path:\t {}",
                         addendum.c_str(), after_transform.size(), grid_path.c_str());
        return false;
    }
    return true;
}

void ResultWriterFs::set_path_addendum(Path& addendum_) {
    this->addendum = addendum_;
}

std::tuple<CGAL::Surface_mesh<Point>, std::array<Point, 4>> Plane2Mesh(Point o1, Point p1, Point p2, bool swap_normal_direction){
    auto e0 = o1;
    auto e1 = p1 ;
    auto e2 = p2;
    auto _e3 = CGAL::cross_product(e1- e0,  e2- e0);
    _e3= (_e3 / CGAL::sqrt(_e3.squared_length()));
    auto e3 = e0 + _e3;
    std::array<Point, 4> plane_system{e0, e1, e2, e3};

    if(swap_normal_direction){
        _e3 *= -1;
    }
    //box thickness
    auto eth = (p1 - e0) * 0.001;
    std::array<Point , 8> obb_points;
    obb_points[0] = p2  - eth;
    obb_points[1] = o1  - eth;
    obb_points[2] = o1 - _e3 - eth;
    obb_points[3] = p2 - _e3 - eth;

    obb_points[4] = p2 - _e3 + eth;
    obb_points[5] = p2  + eth;
    obb_points[6] = o1  + eth;
    obb_points[7] = o1 - _e3 + eth;

    CGAL::Surface_mesh<Point> plane_mesh;
    auto box = CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                                     obb_points[4], obb_points[5], obb_points[6], obb_points[7],
                                     plane_mesh);
    return {plane_mesh, plane_system};

}

std::unique_ptr<ResultWriter> make_result_writer(const Path &base_output_path) {
    if(base_output_path.empty()) return std::make_unique<ResultWriter>(base_output_path);
    return std::make_unique<ResultWriterFs>(base_output_path);
}

bool ResultWriterFs::write(ResultType::BoundingBox box, BBox bounding_box, bool is_final) {
    auto fin = is_final ? "final_" : "open_";
    std::filesystem::path box_path(base_path / "bounding_boxes" / fmt::format("box_{}_{}.off", fin, addendum.c_str()));
    return _write_bbox(bounding_box, box_path);
}

bool ResultWriterFs::write(ResultType::BoundingBox box, BBox bounding_box) {
    std::filesystem::path box_path(base_path / "bounding_boxes" / fmt::format("box_{}.off", addendum.c_str()));
    return _write_bbox(bounding_box, box_path);
}

bool ResultWriterFs::_write_bbox(BBox &bounding_box, Path path) {
    create_directories(path.parent_path());
    CGAL::Surface_mesh<Point> obb_sm;
    auto vertices = bounding_box.get_vertices();
    CGAL::make_hexahedron(vertices[0], vertices[1], vertices[2], vertices[3],
                          vertices[4], vertices[5], vertices[6], vertices[7], obb_sm);
    std::ofstream(path) << obb_sm;
    return true;
}
