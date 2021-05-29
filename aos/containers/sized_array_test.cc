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
  EXPECT_NE(a.size(), a.capacity());
  EXPECT_EQ(0u, a.size());
  EXPECT_EQ(5u, a.capacity());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  EXPECT_EQ(1u, a.size());
  EXPECT_EQ(5u, a.capacity());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  EXPECT_EQ(2u, a.size());
  EXPECT_EQ(5u, a.capacity());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  EXPECT_EQ(3u, a.size());
  EXPECT_EQ(5u, a.capacity());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  EXPECT_EQ(4u, a.size());
  EXPECT_EQ(5u, a.capacity());

  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_EQ(a.size(), a.capacity());
  EXPECT_EQ(5u, a.size());
  EXPECT_EQ(5u, a.capacity());
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
  // The absl InlinedVector is overly clever and realizes that, for certain
  // vectors, we can increase the static size of the vector for free...
  EXPECT_EQ(4, a.capacity());
  EXPECT_TRUE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  a.push_back(9);
  EXPECT_FALSE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  a.push_back(9);
  EXPECT_FALSE(a.empty());
  a.push_back(7);
  EXPECT_FALSE(a.empty());
  a.push_back(1);
  EXPECT_EQ(a.size(), a.capacity());

  a.clear();
  EXPECT_TRUE(a.empty());
  EXPECT_NE(a.size(), a.capacity());
  a.push_back(1);
  EXPECT_EQ(1, a.back());
}

TEST(SizedArrayTest, OverflowTest) {
  SizedArray<int, 4> a;
  EXPECT_EQ(a.capacity(), 4u);
  EXPECT_TRUE(a.empty());

  const int *const pre_front = a.data();
  a.assign({1, 2, 3, 4});

  EXPECT_EQ(a.capacity(), 4u);
  // Verify that we didn't reallocate
  EXPECT_EQ(pre_front, a.data());

  EXPECT_DEATH(a.emplace_back(5), "Aborted at");
}

// Tests inserting at various positions in the array.
TEST(SizedArrayTest, Inserting) {
  SizedArray<int, 5> a;
  a.insert(a.begin(), 2);
  EXPECT_EQ(a.at(0), 2);
  EXPECT_EQ(a.size(), 1);

  a.emplace_back(3);
  EXPECT_EQ(a.at(0), 2);
  EXPECT_EQ(a.at(1), 3);
  EXPECT_EQ(a.size(), 2);

  a.insert(a.begin(), 0);
  EXPECT_EQ(a.at(0), 0);
  EXPECT_EQ(a.at(1), 2);
  EXPECT_EQ(a.at(2), 3);
  EXPECT_EQ(a.size(), 3);

  a.insert(a.begin() + 1, 1);
  EXPECT_EQ(a.at(0), 0);
  EXPECT_EQ(a.at(1), 1);
  EXPECT_EQ(a.at(2), 2);
  EXPECT_EQ(a.at(3), 3);
  EXPECT_EQ(a.size(), 4);

  a.insert(a.begin() + 1, 0);
  EXPECT_EQ(a.at(0), 0);
  EXPECT_EQ(a.at(1), 0);
  EXPECT_EQ(a.at(2), 1);
  EXPECT_EQ(a.at(3), 2);
  EXPECT_EQ(a.at(4), 3);
  EXPECT_EQ(a.size(), 5);
}

// Tests erasing things from the array
TEST(SizedArrayTest, Erasing) {
  SizedArray<int, 5> a;
  a.push_back(8);
  a.push_back(9);
  a.push_back(7);
  a.push_back(1);
  a.push_back(5);
  EXPECT_EQ(a.at(0), 8);
  EXPECT_EQ(a.at(1), 9);
  EXPECT_EQ(a.at(2), 7);
  EXPECT_EQ(a.at(3), 1);
  EXPECT_EQ(a.at(4), 5);
  EXPECT_EQ(a.size(), 5);

  a.erase(a.begin() + 1, a.begin() + 3);
  EXPECT_EQ(a.at(0), 8);
  EXPECT_EQ(a.at(1), 1);
  EXPECT_EQ(a.at(2), 5);
  EXPECT_EQ(a.size(), 3);

  a.erase(a.begin());
  EXPECT_EQ(a.at(0), 1);
  EXPECT_EQ(a.at(1), 5);
  EXPECT_EQ(a.size(), 2);

  a.erase(a.end() - 1);
  EXPECT_EQ(a.at(0), 1);
  EXPECT_EQ(a.size(), 1);
}

}  // namespace testing
}  // namespace aos
