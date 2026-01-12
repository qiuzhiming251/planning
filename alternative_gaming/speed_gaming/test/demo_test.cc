#include <gtest/gtest.h>

// // 定义一个测试用例
// TEST(TestCaseName, TestName)
// {
//     EXPECT_EQ(2 + 2, 4);
// }

// 对于独立测试程序，添加以下定义
GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}