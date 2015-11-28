#include "aos/common/util/linked_list.h"

#include <vector>

#include "gtest/gtest.h"

namespace aos {
namespace util {
namespace testing {

class LinkedListTest : public ::testing::Test {
 public:
  virtual ~LinkedListTest() {
    while (list.head() != nullptr) {
      RemoveElement(list.head());
    }
  }

  struct Member {
    Member(int i) : i(i) {}

    int i;
    Member *next = nullptr;
  };
  LinkedList<Member> list;

  Member *AddElement(int i) {
    Member *member = new Member(i);
    list.Add(member);
    return member;
  }

  void RemoveElement(Member *member) {
    list.Remove(member);
    delete member;
  }

  Member *GetMember(int i) const {
    return list.Find([i](const Member *member) { return member->i == i; });
  }

  bool HasMember(int i) const { return GetMember(i) != nullptr; }

  ::std::vector<int> GetMembers() {
    ::std::vector<int> r;
    list.Each([&r](Member *member) { r.push_back(member->i); });
    return r;
  }
};

// Tests that adding and removing elements works correctly.
TEST_F(LinkedListTest, Basic) {
  EXPECT_TRUE(list.Empty());
  AddElement(971);
  EXPECT_FALSE(list.Empty());
  AddElement(254);
  EXPECT_FALSE(list.Empty());
  AddElement(1678);
  EXPECT_FALSE(list.Empty());

  EXPECT_EQ((::std::vector<int>{1678, 254, 971}), GetMembers());

  EXPECT_EQ(1678, list.head()->i);
  RemoveElement(list.head());
  EXPECT_EQ(254, list.head()->i);
  EXPECT_FALSE(list.Empty());
  RemoveElement(list.head());
  EXPECT_EQ(971, list.head()->i);
  EXPECT_FALSE(list.Empty());
  RemoveElement(list.head());
  EXPECT_TRUE(list.Empty());
}

TEST_F(LinkedListTest, Each) {
  ::std::vector<int> found;
  auto add_to_found = [&found](Member *member) {
    found.push_back(member->i);
  };

  AddElement(971);
  found.clear();
  list.Each(add_to_found);
  EXPECT_EQ((::std::vector<int>{971}), found);

  AddElement(254);
  found.clear();
  list.Each(add_to_found);
  EXPECT_EQ((::std::vector<int>{254, 971}), found);

  AddElement(1678);
  found.clear();
  list.Each(add_to_found);
  EXPECT_EQ((::std::vector<int>{1678, 254, 971}), found);
}

TEST_F(LinkedListTest, Find) {
  auto find_254 = [](const Member *member) { return member->i == 254; };

  AddElement(971);
  EXPECT_EQ(nullptr, list.Find(find_254));
  Member *member = AddElement(254);
  EXPECT_EQ(member, list.Find(find_254));
  AddElement(1678);
  EXPECT_EQ(member, list.Find(find_254));
}

// Removing an element from the middle of the list used to break it.
TEST_F(LinkedListTest, RemoveFromMiddle) {
  AddElement(971);
  auto in_middle = AddElement(254);
  AddElement(1678);
  RemoveElement(in_middle);

  EXPECT_EQ((::std::vector<int>{1678, 971}), GetMembers());
}

}  // namespace testing
}  // namespace util
}  // namespace aos
