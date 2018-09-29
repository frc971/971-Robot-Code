#ifndef AOS_GTEST_PROD_H_
#define AOS_GTEST_PROD_H_

// These macros replace gtest's FRIEND_TEST if the test is in a different
// namespace than the code that needs to make it a friend.
// Example:
//  foo.h:
//   namespace bla {
//   namespace testing {
//
//   FORWARD_DECLARE_TEST_CASE(FooTest, Bar);
//
//   }  // namespace testing
//
//   class Foo {
//     FRIEND_TEST_NAMESPACE(FooTest, Bar, testing);
//   };
//
//   }  // namespace bla
//  foo_test.cc:
//   namespace bla {
//   namespace testing {
//
//   TEST(FooTest, Bar) {
//     access private members of Foo
//   }
//
//   }  // namespace testing
//   }  // namespace bla
#define FORWARD_DECLARE_TEST_CASE(test_case_name, test_name) \
    class test_case_name##_##test_name##_Test;
#define FRIEND_TEST_NAMESPACE(test_case_name, test_name, namespace_name) \
    friend class namespace_name::test_case_name##_##test_name##_Test

// Copied from googletest's gtest_prod.h. See that file for documentation.
#define FRIEND_TEST(test_case_name, test_name) \
  friend class test_case_name##_##test_name##_Test

#endif  // AOS_GTEST_PROD_H_
