#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jpsl/jpsl.hpp"
#include "jpsl/encodings.hpp"

using namespace std;
using namespace JPSL;

TEST(test_standardize, test_standardize) {

  for (Dir d : NEIGHBORS_3D) {
    auto cchange = standardize_dir(d);

    // Expect d to be mapped to default direction
    if (d.order() == 1) {
      EXPECT_EQ(cchange(d), Dir(-1,0,0));
    }

    if (d.order() == 2) {
      EXPECT_EQ(cchange(d), Dir(-1,-1,0));
    }

    if (d.order() == 3) {
      EXPECT_EQ(cchange(d), Dir(-1,-1,-1));
    }

    // Expect mapping to be its own inverse
    for (Dir d : NEIGHBORS_3D) {
      EXPECT_EQ(d, cchange(cchange(d)));
    }

  }

}