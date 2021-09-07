//
// Created by felix on 09.06.2021.
//

#ifndef TEST_BIND_TYPEDEFS_H
#define TEST_BIND_TYPEDEFS_H
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <tuple>
#include <filesystem>
#include <CGAL/property_map.h>
#include <CGAL/Plane_3.h>
#include <Eigen/Dense>
#include <stdexcept>
#include <iterator>

namespace boxy{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef Kernel::Point_3 Point;
    typedef std::tuple<Point, uint32_t> XYZC;
    typedef CGAL::Nth_of_tuple_property_map<0, XYZC> Point_map;
    typedef CGAL::Nth_of_tuple_property_map<1, XYZC> Class_map;
    typedef std::vector<XYZC> pointcloud_xyzc;
    using Plane_3 = Kernel::Plane_3;
    using Point2D = Eigen::Vector2d; //Kernel::Point_2;
    using Vector_3 = Kernel::Vector_3;
    using Vector3f = Eigen::Vector3d;
    using Vector2f = Eigen::Vector2d;
    using Grid = Eigen::Matrix2Xf;
    using MatrixXu = Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorXu = Eigen::Matrix<uint32_t , Eigen::Dynamic, 1>;

    enum BoxFaces{
        A=0,
        B=1,
        C=2,
        D=3,
        E=4,
        F=5
    };


    //TODO cartesian_begin  should be an iterator pointing to the first float
    inline Vector3f cgal_to_eigen(Point point){
        Vector3f tmp{point.x(), point.y(), point.z()};
        return tmp;
    }

    inline Vector3f cgal_to_eigen(Kernel::Vector_3 point){
        Vector3f tmp{point.x(), point.y(), point.z()};
        return tmp;
    }

    inline Point eigen_to_cgal(Vector3f point){
        Point* cgal= reinterpret_cast<Point*>(const_cast<double*>(point.data()));
        return *cgal;
    }

    struct CoordinateSystem2D{
        Vector3f e_0;
        Vector3f e_x;
        Vector3f e_y;

        CoordinateSystem2D(Vector3f O, Vector3f X, Vector3f Y) : e_0{O}, e_x(X/X.norm()), e_y(Y/Y.norm()){}

        [[nodiscard]] Point2D project_onto_plane(const Point& point3D) const{
              Vector3f p3D(cgal_to_eigen(point3D));
              p3D = p3D - e_0;
              Vector2f p2D(e_x.dot(p3D), e_y.dot(p3D));
              return p2D;;
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
                default:
                    throw std::runtime_error("Case {name} not found.");
            }
        }
            std::array<Point , 8> vertices{};

        public:
            BBox(Point p1,Point p2,Point p3,Point p4,Point p5,Point p6,Point p7,Point p8) :
                    vertices({p1, p2, p3, p4, p5 , p6, p7, p8}) {}

            BBox(std::array<Point, 8> arr) : vertices(arr) {}

            BBox() = default;

            [[nodiscard]] std::array<Point, 8> get_vertices() const{
                return vertices;
            }

            [[nodiscard]] Plane_3 get_plane(BoxFaces face) const{
                return std::make_from_tuple<Plane_3>(GetPoints(face).YOX);
            }

            [[nodiscard]] CoordinateSystem2D get_plane_coordinates2D(BoxFaces face) const{
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

            float get_depth(BoxFaces face) {
                switch (face) {
                    case BoxFaces::A:
                    case BoxFaces::D:
                        return CGAL::sqrt<float>((vertices[5]- vertices[0]).squared_length());
                    case BoxFaces::B:
                    case BoxFaces::E:
                        return CGAL::sqrt<float>((vertices[3]- vertices[0]).squared_length());
                    case BoxFaces::C:
                    case BoxFaces::F:
                        return CGAL::sqrt<float>((vertices[1]- vertices[0]).squared_length());
                    default:
                        throw std::runtime_error("This face does not exist");
                }
            }

            [[nodiscard]] float volume() const{
                return abs(cross_product((vertices[1] - vertices[0]),(vertices[3] - vertices[0])) * (vertices[5] - vertices[0]));
            }

            template<class TPointCloud>
            TPointCloud intersect(TPointCloud& points) const{
                TPointCloud tmp;
                std::array<Plane_3, 8> sides =  {get_plane(BoxFaces::A),    get_plane(BoxFaces::B), get_plane(BoxFaces::C),
                                                 get_plane(BoxFaces::D),    get_plane(BoxFaces::E), get_plane(BoxFaces::F)};

                for(auto& point : points){
                    for(auto& plane : sides){
                        if(plane.has_on_positive_side(std::get<0>(point))){
                            goto skip_point;
                        }
                    }
                    tmp.push_back(point);
                    skip_point:;
                }
                return tmp;
            }
    };

    // TODO Extract this to a Pybind Typedef i guess ?

//    using objectlist = std::unordered_map<int, std::string>;
    using Path = std::filesystem::path;

    template<typename TIterator>
    class VectorView{
//        using constit = typename T::const_iterator;
        TIterator _begin;
        TIterator _end;

        public:
            VectorView(TIterator begin, TIterator end)  : _begin(begin), _end(end){}
//            VectorView()= default;
        TIterator begin() {return _begin; }
        TIterator end() {return _end;}
//        using const_iterator = typename std::iterator_traits<TIterator>::const_iterator;
        TIterator begin() const { return _begin; }
        TIterator end()   const { return _end; }
        TIterator cbegin() const { return _begin; }
        TIterator cend()   const { return _end; }

        typename std::iterator_traits<TIterator>::reference operator[](std::size_t index) { return _begin[index]; }
        [[nodiscard]] size_t size() const{
            return std::distance(_begin, _end);
        }
    };

    struct BestGridSplit{
        uint32_t area;
        uint32_t index;
    };

    struct BestSplit{
        double area;
        Vector2f begin_cut;
        Vector2f end_cut;
        Vector2f origin;
    };
}

#endif //TEST_BIND_TYPEDEFS_H
