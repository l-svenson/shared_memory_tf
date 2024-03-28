#pragma once

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <ctime>
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace boost::interprocess;

// Typedefs
typedef managed_shared_memory::segment_manager segment_manager_t;
typedef allocator<void, segment_manager_t> void_allocator;
typedef allocator<char, managed_shared_memory::segment_manager> CharAllocator;
typedef basic_string<char, std::char_traits<char>, CharAllocator> ShmString;

struct Vector3
{
  double t_x{0.0};
  double t_y{0.0};
  double t_z{0.0};

  Eigen::Map<const Eigen::Vector3d> eigenVector() const
  {
    return Eigen::Map<const Eigen::Vector3d>(&t_x);
  }
};

struct Quaternion
{
  double q_x{0.0};
  double q_y{0.0};
  double q_z{0.0};
  double q_w{1.0};

  Eigen::Map<const Eigen::Quaterniond> eigenQuaternion() const
  {
    return Eigen::Map<const Eigen::Quaterniond>(&q_x);
  }
};

struct Transformation
{
  uint64_t stamp_nanosec;

  Vector3 translation;
  Quaternion rotation;
};

struct TransformationBuffer
{
  TransformationBuffer(const char* parent_frame_id, const void_allocator& void_alloc)
      : parent_frame_id(parent_frame_id, void_alloc)
  {
  }

  static constexpr std::size_t LENGTH = 10;
  Transformation transformations[LENGTH];
  std::size_t current_index{0};

  const ShmString parent_frame_id;

  void addTransformation(Transformation trafo)
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);

    std::cout << "  current_index=" << current_index << std::endl;

    transformations[current_index] = trafo;

    current_index++;

    if (current_index >= LENGTH)
    {
      current_index = 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

private:
  boost::interprocess::interprocess_mutex mutex_;
};

// Define the streaming operator for Transformation
std::ostream& operator<<(std::ostream& os, const Transformation& Transformation)
{
  os << "stamp=" << Transformation.stamp_nanosec << ", translation=(" << Transformation.translation.eigenVector().transpose()
     << "), rotation=(" << Transformation.rotation.eigenQuaternion() << ")";
  return os;
}
