//
// Created by felix on 17.09.21.
//

#ifndef BOXOFFICE_SPLITSTRATEGIES_H
#define BOXOFFICE_SPLITSTRATEGIES_H
namespace mvbb

struct xy_split{
    BestGridSplit x;
    BestGridSplit y;
};

class SplitStrategy{
    public:
        virtual xy_split calculate_best_splits(const Grid& grid) = 0


};

class TwoSplitStragety : SplitStrategy{

};

}
#endif //BOXOFFICE_SPLITSTRATEGIES_H
