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
#include <spdlog/spdlog.h>


std::tuple<CGAL::Surface_mesh<boxy::Point>, std::array<boxy::Point,4>> Plane2Mesh(boxy::Point o1, boxy::Point p1, boxy::Point p2, bool swap_normal_direction=false);

using namespace boxy;

namespace  helpers {

    template <typename T>
    struct range_t{
        T a, b;
        range_t(T x, T y) : a(x), b(y) {}
        T begin(){
            return a;
        }
        T end(){
            return b;
        }
    };

    template <typename T>
    range_t<T> range(T a, T b)    {
        return range_t<T>(a, b);
    }

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
        }
        else {
            spdlog::error("Could not open: {}", std::filesystem::weakly_canonical(path).generic_string());
        }

        spdlog::debug("Reading File in lines => {} fround", entries.size());
        return entries;
    }

    bool xyzc_compare(const XYZC& p1, const XYZC& p2);

    bool xyzc_objecttype_compare(const XYZC& xyzc, const XYZC& lookup);
}
#endif