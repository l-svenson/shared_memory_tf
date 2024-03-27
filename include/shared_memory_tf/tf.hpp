#pragma once

#include <boost/interprocess/managed_shared_memory.hpp>
#include <ctime>
#include <iostream>
#include <thread>

struct Vector3
{
  double t_x{0.0};
  double t_y{0.0};
  double t_z{0.0};

  double* data()
  {
    return &t_x;
  }
};

struct Quaternion
{
  double q_x{0.0};
  double q_y{0.0};
  double q_z{0.0};
  double q_w{1.0};

  double* data()
  {
    return &q_x;
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
  static constexpr std::size_t LENGTH = 10;
  Transformation transformations[LENGTH];
  std::size_t current_index{0};

  void addTransformation(Transformation trafo)
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);

    std::cout << "current_index=" << current_index << std::endl;

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
  os << "stamp=" << Transformation.stamp_nanosec << ", translation=(" << Transformation.translation.t_x << ", "
     << Transformation.translation.t_y << ", " << Transformation.translation.t_z << "), rotation=(" << Transformation.rotation.q_x
     << ", " << Transformation.rotation.q_y << ", " << Transformation.rotation.q_z << ", " << Transformation.rotation.q_w << ")";
  return os;
}
