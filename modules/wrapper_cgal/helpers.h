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
#include "trim.h"
using namespace boxy;

namespace  helpers {

    auto xyzc_2_numpy(const pointcloud_xyzc &pointcloud) {

        //create numpy memory space
        auto shape = std::array<ssize_t, 2>{static_cast<ssize_t>(pointcloud.size()), 4};
        np_array ext_pointcloud(shape);

        // copy data to numpy array, Observation: <50m point cloud its not even worth to think about par_unseq
        auto ext_begin = ext_pointcloud.mutable_data();
        std::for_each(pointcloud.cbegin(), pointcloud.cend(),
                      [&point_cloud = std::as_const(pointcloud), &ext_begin = std::as_const(ext_begin)](
                              auto const &cgal_point) {
                          int idx = (&cgal_point - &point_cloud[0]) * 4;
                          ext_begin[idx + 0] = std::get<0>(cgal_point).x();
                          ext_begin[idx + 1] = std::get<0>(cgal_point).y();
                          ext_begin[idx + 2] = std::get<0>(cgal_point).z();
                          ext_begin[idx + 3] = float(std::get<1>(cgal_point));
                      });
        return ext_pointcloud;
    }

    auto split(std::string const &str, const char delim) {
        std::vector<std::string> parts;
        std::string part;
        std::stringstream ss(str);
        while (std::getline(ss, part, delim)) {
            parts.push_back(part);
        }
        return parts;
    }

    template<int Tn>
    auto take_first_n(std::string const &str, const char delim) {
        std::array<std::string, Tn> first_n;
        std::string tmp;
        std::stringstream ss(str);
        for (std::string &tmp : first_n) {
            if (!std::getline(ss, tmp, delim)) break;
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
}

#endif //TEST_BIND_HELPER_H
//    auto start = std::chrono::high_resolution_clock::now();
//    auto finish = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> elapsed = finish - start;
//    std::cout << "Elapsed time: " << elapsed.count() << " s\n";