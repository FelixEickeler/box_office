//
// Created by felix on 09.07.2021.
//

#include "gtest/gtest.h"
#include "helpers.h"
#include <array>


TEST (helper_testing /*test suite name*/, take_first_n_1to10 /*test name*/) {
    auto test_string = "The red-bellied black snake (Pseudechis-porphyriacus) is a species of elapid snake native to Australia";
    std::array<std::string, 14> result = {
            "The", "red-bellied", "black", "snake", "(Pseudechis-porphyriacus)", "is", "a", "species", "of", "elapid",
            "snake", "native", "to", "Australia"
    };
    {
        using namespace helpers;
        auto zero = take_first_n<0>(test_string, ' ');
        ASSERT_EQ(zero.size(), 0);

        auto first = take_first_n<1>(test_string, ' ');
        ASSERT_EQ(first[0], "The");

        auto fourteen = take_first_n<14>(test_string, ' ');
        for (auto word = 0; word < fourteen.size(); ++word) {
            ASSERT_EQ(fourteen[word], result[word]);
        }
    }
}
