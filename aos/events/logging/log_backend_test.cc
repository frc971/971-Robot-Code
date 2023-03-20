#include "aos/events/logging/log_backend.h"

#include <filesystem>

#include "aos/testing/tmpdir.h"
#include "gtest/gtest.h"

namespace aos::logger::testing {
TEST(LogBackendTest, CreateSimpleFile) {
  const std::string logevent = aos::testing::TestTmpDir() + "/logevent/";
  FileBackend backend(logevent);
  auto file = backend.RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = write(file->fd(), "test", 4);
  EXPECT_GT(result, 0);
  EXPECT_EQ(file->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
}

TEST(LogBackendTest, CreateRenamableFile) {
  const std::string logevent = aos::testing::TestTmpDir() + "/logevent/";
  RenamableFileBackend backend(logevent);
  auto file = backend.RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = write(file->fd(), "testtest", 8);
  EXPECT_GT(result, 0);
  EXPECT_EQ(file->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
}

TEST(LogBackendTest, UseTempRenamableFile) {
  const std::string logevent = aos::testing::TestTmpDir() + "/logevent/";
  RenamableFileBackend backend(logevent);
  backend.EnableTempFiles();
  auto file = backend.RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = write(file->fd(), "testtest", 8);
  EXPECT_GT(result, 0);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log.tmp"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
}

TEST(LogBackendTest, RenameBaseAfterWrite) {
  const std::string logevent = aos::testing::TestTmpDir() + "/logevent/";
  RenamableFileBackend backend(logevent);
  auto file = backend.RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = write(file->fd(), "testtest", 8);
  EXPECT_GT(result, 0);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));

  std::string renamed = aos::testing::TestTmpDir() + "/renamed/";
  backend.RenameLogBase(renamed);

  EXPECT_FALSE(std::filesystem::exists(logevent + "test.log"));
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));
}

TEST(LogBackendTest, UseTestAndRenameBaseAfterWrite) {
  const std::string logevent = aos::testing::TestTmpDir() + "/logevent/";
  RenamableFileBackend backend(logevent);
  backend.EnableTempFiles();
  auto file = backend.RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = write(file->fd(), "testtest", 8);
  EXPECT_GT(result, 0);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log.tmp"));

  std::string renamed = aos::testing::TestTmpDir() + "/renamed/";
  backend.RenameLogBase(renamed);

  EXPECT_FALSE(std::filesystem::exists(logevent + "test.log.tmp"));
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log.tmp"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));
}

}  // namespace aos::logger::testing