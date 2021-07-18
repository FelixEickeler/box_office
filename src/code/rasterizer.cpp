//
// Created by felix on 17.07.2021.
//

#include "rasterizer.h"

std::array<float, 4> mvbb::minmax2D(const BBox &bbox, const CoordinateSystem2D &coordinate_system2D) {
    std::array<float, 4> min_max2D{1.0e10, -1.0e-10,1.0e-10,-1.0e-10};
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

