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

TEST_CASE("overwrite transformation")
{
  shared_memory_object::remove(SHM_SEGMENT_NAME);
  SharedMemoryTFManager tf_manager;

  tf_manager.addTransform(1, "map", "base_link", {5, 2, 0}, {1.0, 0.0, 0.0, 0.0});
  const Eigen::Isometry3d base_link_H_map_new = tf_manager.getHTM(1, "base_link", "map").value();
  std::cout << "base_link_H_map_new=" << std::endl << base_link_H_map_new.matrix() << std::endl;
  REQUIRE(transform_is(tf_manager.getHTM(1, "base_link", "map"), Eigen::Isometry3d(Eigen::Translation3d(5, 2, 0))));

  // tf_manager.addTransform(1, "map", "base_link", {10, 5, 0}, {1.0, 0.0, 0.0, 0.0});
  // const Eigen::Isometry3d base_link_H_map_old = tf_manager.getHTM(1, "base_link", "map").value();
  // std::cout << "base_link_H_map_old=" << std::endl << base_link_H_map_old.matrix() << std::endl;
  // REQUIRE(transform_is(tf_manager.getHTM(1, "base_link", "map"), Eigen::Isometry3d(Eigen::Translation3d(10, 5, 0))));
}

TEST_CASE("build HTM")
{
  shared_memory_object::remove(SHM_SEGMENT_NAME);
  SharedMemoryTFManager tf_manager;
  tf_manager.addTransform(1, "map", "base_link", {5, 2, 0}, {1.0, 0.0, 0.0, 0.0});
  tf_manager.addTransform(1, "base_link", "sensor", {1, 0, 0}, {1.0, 0.0, 0.0, 0.0});
  tf_manager.addTransform(1, "sensor", "camera", {0, 0, 0}, {-0.5, 0.5, -0.5, 0.5});

  // Translation -> HTM
  const Eigen::Isometry3d base_link_H_map = tf_manager.getHTM(1, "base_link", "map").value();
  REQUIRE(transform_is(tf_manager.getHTM(1, "base_link", "map"), Eigen::Isometry3d(Eigen::Translation3d(5, 2, 0))));

  // Rotation -> HTM
  const Eigen::Isometry3d sensor_H_camera = tf_manager.getHTM(1, "sensor", "camera").value();
  REQUIRE(transform_is(tf_manager.getHTM(1, "sensor", "camera"), Eigen::Isometry3d(Eigen::Quaterniond(-0.5, 0.5, -0.5, 0.5))));
  std::cout << "sensor_H_camera:" << std::endl;
  std::cout << sensor_H_camera.matrix() << std::endl;

  // // Translation + Rotation -> HTM
  // const Eigen::Isometry3d map_H_camera = tf_manager.getHTM(1, "map", "camera").value();
  // REQUIRE(transform_is(tf_manager.getHTM(1, "map", "camera"),
  //                      Eigen::Isometry3d(Eigen::Quaterniond(-0.5, 0.5, -0.5, 0.5) * Eigen::Translation3d(6, 2, 0))));
  // std::cout << "map_H_camera:" << std::endl;
  // std::cout << map_H_camera.matrix() << std::endl;
}

TEST_CASE("full transformation chain")
{
  shared_memory_object::remove(SHM_SEGMENT_NAME);
  SharedMemoryTFManager tf_manager;
  tf_manager.addTransform(1, "map", "base_link", {5, 2, 0}, {0.707107, 0, 0, 0.707107});
  tf_manager.addTransform(1, "base_link", "sensor", {1, 0, 0}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "sensor", "camera", {0, 0, 0}, {-0.5, 0.5, -0.5, 0.5});

  const Eigen::Vector3d map_A{5, 5, 0};
  const Eigen::Vector3d map_B{6, 4, 1};

  const Eigen::Isometry3d base_link_H_map = tf_manager.getHTM(1, "base_link", "map").value();
  const Eigen::Isometry3d sensor_H_map = tf_manager.getHTM(1, "sensor", "map").value();
  const Eigen::Isometry3d camera_H_map = tf_manager.getHTM(1, "camera", "map").value();

  const Eigen::Isometry3d map_H_base_link = tf_manager.getHTM(1, "map", "base_link").value();

  std::cout << base_link_H_map.matrix() << std::endl;

  REQUIRE(((base_link_H_map * map_H_base_link).matrix() - Eigen::Isometry3d::Identity().matrix()).norm() < 1e-10);

  // Transform all points into base_link frame
  const Eigen::Vector3d base_link_A = base_link_H_map * map_A;
  const Eigen::Vector3d base_link_B = base_link_H_map * map_B;

  // Transform all points into sensor frame
  const Eigen::Vector3d sensor_A = sensor_H_map * map_A;
  const Eigen::Vector3d sensor_B = sensor_H_map * map_B;

  // Transform all points into camera frame
  const Eigen::Vector3d camera_A = camera_H_map * map_A;
  const Eigen::Vector3d camera_B = camera_H_map * map_B;

  REQUIRE(base_link_A == Eigen::Vector3d(3, 0, 0));
  // REQUIRE(base_link_A == Eigen::Vector3d(3, 0, 0));

  // REQUIRE(transform_is(tf_manager.getHTM(2, "grandparent", "parent"), Eigen::Isometry3d(Eigen::Translation3d(0,
  // 0, 1.5))));
}
