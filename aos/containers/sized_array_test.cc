#include "aos/containers/sized_array.h"

#include "gtest/gtest.h"

namespace aos {
namespace testing {

// Tests the various ways of accessing elements.
TEST(SizedArrayTest, ElementAccess) {
  SizedArray<int, 5> a;
  a.push_back(9);
  a.push_back(7);
  a.push_back(1);
  EXPECT_EQ(9, a[0]);
  EXPECT_EQ(7, a[1]);
  EXPECT_EQ(1, a[2]);
  EXPECT_EQ(9, a.data()[0]);
  EXPECT_EQ(7, a.data()[1]);
  EXPECT_EQ(1, a.data()[2]);
  EXPECT_EQ(9, a.at(0));
  EXPECT_EQ(7, a.at(1));
  EXPECT_EQ(1, a.at(2));
  EXPECT_EQ(9, a.front());
  EXPECT_EQ(1, a.back());

  a.pop_back();
  EXPECT_EQ(9, a.front());
  EXPECT_EQ(7, a.back());
}

// Tests the accessors that don't access data.
TEST(SizedArrayTest, Accessors) {
  SizedArray<int, 5> a;
  EXPECT_TRUE(a.empty());
  EXPECT_FALSE(a.full());
  EXPECT_EQ(0u, a.size());
  EXPECT_EQ(5u, a.max_size());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_FALSE(a.full());
  EXPECT_EQ(1u, a.size());
  EXPECT_EQ(5u, a.max_size());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_FALSE(a.full());
  EXPECT_EQ(2u, a.size());
  EXPECT_EQ(5u, a.max_size());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_FALSE(a.full());
  EXPECT_EQ(3u, a.size());
  EXPECT_EQ(5u, a.max_size());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_FALSE(a.full());
  EXPECT_EQ(4u, a.size());
  EXPECT_EQ(5u, a.max_size());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_TRUE(a.full());
  EXPECT_EQ(5u, a.size());
  EXPECT_EQ(5u, a.max_size());
}

// Tests the various kinds of iterator.
TEST(SizedArrayTest, Iterators) {
  SizedArray<int, 5> a;
  EXPECT_EQ(a.begin(), a.end());
  EXPECT_EQ(a.cbegin(), a.cend());
  EXPECT_EQ(a.rbegin(), a.rend());
  EXPECT_EQ(a.crbegin(), a.crend());
  a.push_back(9);
  a.push_back(7);
  a.push_back(1);

  {
    auto iterator = a.begin();
    ASSERT_NE(iterator, a.end());
    EXPECT_EQ(9, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.end());
    EXPECT_EQ(7, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.end());
    EXPECT_EQ(1, *iterator);
    ++iterator;
    EXPECT_EQ(iterator, a.end());
  }

  {
    auto iterator = a.cbegin();
    ASSERT_NE(iterator, a.cend());
    EXPECT_EQ(9, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.cend());
    EXPECT_EQ(7, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.cend());
    EXPECT_EQ(1, *iterator);
    ++iterator;
    EXPECT_EQ(iterator, a.cend());
  }

  {
    auto iterator = a.rbegin();
    ASSERT_NE(iterator, a.rend());
    EXPECT_EQ(1, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.rend());
    EXPECT_EQ(7, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.rend());
    EXPECT_EQ(9, *iterator);
    ++iterator;
    EXPECT_EQ(iterator, a.rend());
  }

  {
    auto iterator = a.crbegin();
    ASSERT_NE(iterator, a.crend());
    EXPECT_EQ(1, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.crend());
    EXPECT_EQ(7, *iterator);
    ++iterator;
    ASSERT_NE(iterator, a.crend());
    EXPECT_EQ(9, *iterator);
    ++iterator;
    EXPECT_EQ(iterator, a.crend());
  }
}

// Tests various ways of filling up and emptying.
TEST(SizedArrayTest, FillEmpty) {
  SizedArray<int, 2> a;
  EXPECT_TRUE(a.empty());
  EXPECT_FALSE(a.full());
  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_FALSE(a.full());
  a.push_back(7);
  EXPECT_FALSE(a.empty());
  EXPECT_TRUE(a.full());

  a.clear();
  EXPECT_TRUE(a.empty());
  EXPECT_FALSE(a.full());
  a.push_back(1);
  EXPECT_EQ(1, a.back());
}

}  // namespace testing
}  // namespace aos
