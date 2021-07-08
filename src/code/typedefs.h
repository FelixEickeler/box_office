//
// Created by felix on 09.06.2021.
//

#ifndef TEST_BIND_TYPEDEFS_H
#define TEST_BIND_TYPEDEFS_H
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <pybind11/numpy.h>
#include <tuple>
#include <filesystem>
#include <CGAL/property_map.h>
#include <CGAL/Plane_3.h>
#include <Eigen/Dense>

namespace boxy{
    typedef CGAL::Simple_cartesian<float> Kernel;
    typedef Kernel::Point_3 Point;
    typedef std::tuple<Point, uint32_t> XYZC;
    typedef CGAL::Nth_of_tuple_property_map<0, XYZC> Point_map;
    typedef CGAL::Nth_of_tuple_property_map<1, XYZC> Class_map;
    typedef std::vector<XYZC> pointcloud_xyzc;
    using Plane_3 = Kernel::Plane_3;
    using Point2D = Eigen::Vector2f; //Kernel::Point_2;
    using Vector_3 = Kernel::Vector_3;
    using Vector3f = Eigen::Vector3f;
    using Vector2f = Eigen::Vector2f;
    using Grid = Eigen::Matrix2Xf;

    enum BoxFaces{
        A=0,
        B=1,
        C=2,
        D=3,
        E=4,
        F=5
    };


    //TODO cartesian_begin  should be an iterator pointing to the first float
    Vector3f cgal_to_eigen(Point point){
        Vector3f tmp{point.x(), point.y(), point.z()};
        return tmp;
    }

    Vector3f cgal_to_eigen(Kernel::Vector_3 point){
        Vector3f tmp{point.x(), point.y(), point.z()};
        return tmp;
    }

    Point eigen_to_cgal(Vector3f point){
        Point* cgal= reinterpret_cast<Point*>(const_cast<float*>(point.data()));
        return *cgal;
    }

    struct CoordinateSystem2D{
        Vector3f e_0;
        Vector3f e_x;
        Vector3f e_y;

        [[nodiscard]] Point2D project_onto_plane(const Point& point3D) const{
            //TODO fix this
//            Vector3f p3D(point3D);
//            Vector2f p2D(p3D*e_x, p3D*e_y);
//            return p2D;
              Point2D p2D;
              return p2D;
        }

        [[nodiscard]] Point project_to_global(const Point2D& point2D) const{
            Vector2f p2D(point2D);
            auto p3d(e_x * p2D.x() + e_y * p2D.y() + e_0);
            return Point(p3d.x(), p3d.y(), p3d.z());
        }
    };

    struct CoordinateTriple_YOX{
        std::tuple<Point, Point, Point> YOX;
        Point X(){ return std::get<0>(YOX); }
        Point O(){ return std::get<1>(YOX); }
        Point Y(){ return std::get<2>(YOX); }
        CoordinateTriple_YOX(Point Y, Point O, Point X) : YOX(Y,O,X){}
    };

    class BBox{

        [[nodiscard]] CoordinateTriple_YOX GetPoints(BoxFaces face) const{
            // Orientation should be counter clockwise, Points are ordered with the origin as second.
            switch (face) {
                case BoxFaces::A: return {vertices[5], vertices[0], vertices[1]};
                case BoxFaces::B: return {vertices[1], vertices[0], vertices[3]};
                case BoxFaces::C: return {vertices[3], vertices[0], vertices[5]};
                case BoxFaces::D: return {vertices[2], vertices[3], vertices[4]};
                case BoxFaces::E: return {vertices[4], vertices[5], vertices[6]};
                case BoxFaces::F: return {vertices[6], vertices[1], vertices[2]};
            }
        }

        public:
                std::array<Point , 8> vertices;
                Plane_3 get_plane(BoxFaces face) const{
                    return std::make_from_tuple<Plane_3>(GetPoints(face).YOX);
                }

                CoordinateSystem2D get_plane_coordinates2D(BoxFaces face) const{
                    auto triplet = GetPoints(face);
                    auto e_x = cgal_to_eigen(triplet.X() - triplet.O());
                    auto e_y = cgal_to_eigen(triplet.Y() - triplet.O());
                    auto e_0 = cgal_to_eigen(triplet.O());
                    e_x.normalize();
                    e_y.normalize();
                    return {e_0, e_x, e_y};
                }

                std::array<float, 6> min_max(){
                    std::array<float, 6> min_max{};
                    for(auto& v : vertices){
                        if(v.x() < min_max[0]) min_max[0] = v.x();
                        if(v.y() < min_max[1]) min_max[1] = v.y();
                        if(v.z() < min_max[2]) min_max[2] = v.z();
                        
                        if(v.x() > min_max[3]) min_max[3] = v.x();
                        if(v.y() > min_max[4]) min_max[4] = v.y();
                        if(v.z() > min_max[5]) min_max[5] = v.z();
                    }
                    return min_max;
                }
    };

    namespace py = pybind11;
    using np_array = py::array_t<float, py::array::c_style> ;
    using objectlist = std::unordered_map<int, std::string>;
    using Path = std::filesystem::path;

    template<typename T>
    struct crange{
        typename T::const_iterator begin;
        typename T::const_iterator end;
    };

    struct BestGridSplit{
        uint32_t area;
        uint32_t index;
    };

    struct BestSplit{
        double area;
        Vector2f coordinate;
    };
}

#endif //TEST_BIND_TYPEDEFS_H
