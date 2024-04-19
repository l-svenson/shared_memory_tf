#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do this in one cpp file

#include <catch2/catch.hpp>
#include <shared_memory_tf/tf_manager.hpp>

#define SHM_SEGMENT_NAME "shared_memory_tf"
using namespace boost::interprocess;

bool transform_is(std::optional<Eigen::Isometry3d> result, Eigen::Isometry3d expected_value)
{
  if (result)
  {
    if (result.value().matrix() == expected_value.matrix())
    {
      return true;
    }
    else
    {
      std::cout << "False transformation was found!" << std::endl;
      std::cout << "Got: \n" << result.value().matrix() << std::endl;
      std::cout << "Expected: \n" << expected_value.matrix() << std::endl;

      return false;
    }
  }
  else
  {
    std::cout << "No transformation was found!" << std::endl;
    return false;
  }
}

TEST_CASE("Simple translation test_case")
{
  shared_memory_object::remove(SHM_SEGMENT_NAME);
  SharedMemoryTFManager tf_manager;
  tf_manager.addTransform(1, "grandparent", "parent", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "parent", "ego", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "ego", "child", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "child", "grandchild", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "parent", "sibling", {0, 1, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "sibling", "nephew", {0, 0, 1}, {1, 0, 0, 0});

  REQUIRE(transform_is(tf_manager.getHTM(1, "grandchild", "parent"), Eigen::Isometry3d(Eigen::Translation3d(0, 0, 3))));
  REQUIRE(transform_is(tf_manager.getHTM(1, "parent", "grandchild"), Eigen::Isometry3d(Eigen::Translation3d(0, 0, -3))));
  REQUIRE(transform_is(tf_manager.getHTM(1, "nephew", "ego"), Eigen::Isometry3d(Eigen::Translation3d(0, 1, 1))));
  REQUIRE(transform_is(tf_manager.getHTM(1, "nephew", "child"), Eigen::Isometry3d(Eigen::Translation3d(0, 1, 0))));
  REQUIRE(transform_is(tf_manager.getHTM(1, "nephew", "grandparent"), Eigen::Isometry3d(Eigen::Translation3d(0, 1, 3))));
}

TEST_CASE("translation interpolation test_case")
{
  shared_memory_object::remove(SHM_SEGMENT_NAME);
  SharedMemoryTFManager tf_manager;
  tf_manager.addTransform(1, "grandparent", "parent", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(3, "grandparent", "parent", {0, 0, 2}, {1, 0, 0, 0});

  REQUIRE(transform_is(tf_manager.getHTM(2, "parent", "grandparent"), Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1.5))));
}
