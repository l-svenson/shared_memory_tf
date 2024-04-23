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
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Eigen::Map<const Eigen::Vector3d> eigenVector() const
  {
    return Eigen::Map<const Eigen::Vector3d>(&x);
  }
};

struct Quaternion
{
  double q_x{0.0};
  double q_y{0.0};
  double q_z{0.0};
  double q_w{1.0};

  void normalize()
  {
    Eigen::Quaterniond normalized_quat = this->eigenQuaternion().normalized();
    this->q_x = normalized_quat.x();
    this->q_y = normalized_quat.y();
    this->q_z = normalized_quat.z();
    this->q_w = normalized_quat.w();
  }

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

  Eigen::Isometry3d eigenTransformation() const
  {
    return Eigen::Isometry3d(rotation.eigenQuaternion().conjugate()) *
           Eigen::Translation3d(translation.x, translation.y, translation.z);
  }
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

    // TODO: Insert the new trafo at the correct position (in case of out-of-sequence added transformations)
    // TODO: Overwrite in case of adding a new transformation with an existing timestamp (update!)

    transformations[current_index] = trafo;

    current_index++;

    if (current_index >= LENGTH)
    {
      current_index = 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::optional<Eigen::Isometry3d> getTransformation(uint64_t stamp) const
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);

    std::optional<Transformation> prev_trafo;
    std::optional<Transformation> next_trafo;

    for (std::size_t i = 0; i < LENGTH; i++)
    {
      const Transformation& trafo = transformations[i];
      if (trafo.stamp_nanosec > 0)
      {
        if (trafo.stamp_nanosec == stamp)
        {
          return trafo.eigenTransformation();
        }
        else if (trafo.stamp_nanosec < stamp)
        {
          prev_trafo = trafo;
          std::cout << "prev_trafo @ index " << i << std::endl;
        }
        else if (trafo.stamp_nanosec > stamp)
        {
          next_trafo = trafo;
          std::cout << "next_trafo @ index " << i << std::endl;
        }
      }

      // If the requested stamp lies between two existing transformation, interpolate!
      if (prev_trafo && next_trafo)
      {
        // Scaling factor between the previous and the next transformation
        // t = 0.0 => use 100% prev_trafo
        // t = 1.0 => use 100% next_trafo
        const double t =
            double(stamp - prev_trafo->stamp_nanosec) / double(next_trafo->stamp_nanosec - prev_trafo->stamp_nanosec);
        const double factor_prev = 1.0 - t;
        const double factor_next = t;

        const Eigen::Vector3d interpolated_translation(factor_prev * prev_trafo->translation.eigenVector() +
                                                       factor_next * next_trafo->translation.eigenVector());

        const Eigen::Quaterniond interpolated_rotation(
            prev_trafo->rotation.eigenQuaternion().slerp(factor_prev, next_trafo->rotation.eigenQuaternion()));

        return Eigen::Isometry3d(interpolated_rotation.conjugate()) * Eigen::Translation3d(interpolated_translation);
      }
    }

    // Fallback: Not found
    return {};
  }

private:
  mutable boost::interprocess::interprocess_mutex mutex_;
};

// Define the streaming operator for Transformation
std::ostream& operator<<(std::ostream& os, const Transformation& trafo)
{
  os << "stamp=" << trafo.stamp_nanosec << ", translation=(" << trafo.translation.eigenVector().transpose() << "), rotation=("
     << trafo.rotation.eigenQuaternion() << ")";
  return os;
}
