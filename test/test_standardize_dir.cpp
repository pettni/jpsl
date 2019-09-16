#include <iostream>
#include <set>
#include "gtest/gtest.h"

#include "jps/jps.hpp"

using namespace std;
using namespace JPS;

TEST(TestSuite, test_standardize) {

	for (Dir d : NEIGHBORS_3D) {
		cout << d << endl;
		auto cchange = standardize_dir(d);

		// Expect d to be mapped to default direction
		if (d.order() == 1) 
			EXPECT_EQ(cchange(d), Dir(-1,0,0));

		if (d.order() == 2) 
			EXPECT_EQ(cchange(d), Dir(-1,-1,0));

		if (d.order() == 3) 
			EXPECT_EQ(cchange(d), Dir(-1,-1,-1));

		// Expect mapping to be its own inverse
		for (Dir d : NEIGHBORS_3D) 
			EXPECT_EQ(d, cchange(cchange(d)));

	}

}