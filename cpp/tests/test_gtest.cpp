#include "gtest/gtest.h"

TEST(TestGtest, Equality) {
    ASSERT_EQ(1, 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
