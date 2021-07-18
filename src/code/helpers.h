//
// Created by felix on 09.06.2021.
//

#ifndef TEST_BIND_HELPER_H
#define TEST_BIND_HELPER_H
#include "typedefs.h"
#include <fstream>
#include <string>
#include <fstream>
#include <experimental/string>
#include <CGAL/Surface_mesh.h>
#include "trim.h"
#include <CGAL/IO/write_off_points.h>

using namespace boxy;

namespace  helpers {

    // TODO fix to iterators to avoid copy before, also make these in a pybind file !
    np_array xyzc_2_numpy(const pointcloud_xyzc &pointcloud);

    np_array xyzc_2_numpy(VectorView<pointcloud_xyzc::iterator> objr);

    np_array bbox_2_numpy(BBox bbox);



//    auto split(std::string const &str, const char delim);

    template<int Tn>
    auto take_first_n(std::string const &str, const char delim) {
        std::array<std::string, Tn> first_n;
        std::string tmp;
        std::stringstream ss(str);
        for (std::string &tmp : first_n) {
            if (!std::getline(ss, tmp, delim)) break;
            trim(tmp);
        }
        return first_n;
    }

    template<int TColumns>
    auto read_file(const Path &path) {
        std::vector<std::array<std::string, TColumns>> entries;
        std::ifstream ins(path);
        if (ins) {
            std::string line;
            while (getline(ins, line)) {
                entries.emplace_back(take_first_n<TColumns>(line, ' '));
            }
            ins.close();
        } else
            std::cout << "Could not open: " + path.generic_string() << std::endl;
        return entries;
    }

    bool xyzc_compare(const XYZC& p1, const XYZC& p2);

    bool xyzc_objecttype_compare(const XYZC& xyzc, const XYZC& lookup);
}
#endif