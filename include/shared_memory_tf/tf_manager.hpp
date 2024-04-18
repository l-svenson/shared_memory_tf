#pragma once

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <ctime>
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tf.hpp"

using namespace boost::interprocess;

#define SHM_SEGMENT_NAME "shared_memory_tf"

class SharedMemoryTFManager
{
public:
  void addTransform(const uint64_t stamp_nanosec,
                    const std::string& parent_frame_id,
                    const std::string& frame_id,
                    const Eigen::Vector3d& translation,
                    const Eigen::Quaterniond& rotation)
  {
    std::pair<TransformationBuffer*, std::size_t> p = managed_shm.find<TransformationBuffer>(frame_id.c_str());
    if (!p.first)
    {
      std::cout << "Could not find '" << frame_id << "' yet... I will create it now!" << std::endl;
      managed_shm.construct<TransformationBuffer>(frame_id.c_str())(parent_frame_id.c_str(), alloc_inst);
    }

    p = managed_shm.find<TransformationBuffer>(frame_id.c_str());
    if ((p.first->parent_frame_id.c_str() != parent_frame_id.c_str()))
    {
      std::cout << "error! Parent frame id not matching!" << std::endl;
      std::terminate();
    }
    Transformation trafo;

    trafo.stamp_nanosec = stamp_nanosec;
    trafo.translation.x = translation.x();
    trafo.translation.y = translation.y();
    trafo.translation.z = translation.z();
    trafo.rotation.q_x = rotation.x();
    trafo.rotation.q_y = rotation.y();
    trafo.rotation.q_z = rotation.z();
    trafo.rotation.q_w = rotation.w();
    trafo.rotation.normalize();
    p.first->addTransformation(trafo);
  };

  std::optional<Eigen::Isometry3d>
  getTransform(const uint64_t stamp_nanosec, const std::string& parent_frame_id, const std::string& frame_id)
  {
    // Find Transform Chain
    std::vector<std::string> frame_ancestors = getAncestors(frame_id);
    std::vector<std::string> parent_frame_ancestors = getAncestors(parent_frame_id);
    std::string mutual_ancestor;

    for (const auto& frame_ancestor : frame_ancestors)
    {
      for (const auto& parent_frame_ancestor : parent_frame_ancestors)
      {
        if (frame_ancestor == parent_frame_ancestor)
        { // Found mutual ancestor
          mutual_ancestor = frame_ancestor;
        }
      }
    }
    if (mutual_ancestor == "")
    {
      // Tranformation not found!
      return {};
    }

    // Get Tranformations from buffers
    std::vector<std::string> trafo_chain;
    Eigen::Isometry3d trafo = Eigen::Isometry3d::Identity();
    for (const auto& frame_ancestor : frame_ancestors)
    {
      if (frame_ancestor == mutual_ancestor)
      {
        break;
      }
      trafo_chain.push_back(frame_ancestor);
      auto cur_trafo = managed_shm.find<TransformationBuffer>(frame_ancestor.c_str()).first->getTransformation(stamp_nanosec);
      if (cur_trafo)
      {
        trafo = trafo * cur_trafo.value();
      }
      else
      {
        // Tranformation not found for this timestamp
        return {};
      }
    }
    trafo_chain.push_back(mutual_ancestor);
    for (auto it = parent_frame_ancestors.rbegin(); it != parent_frame_ancestors.rend(); it++)
    {
      const auto& parent_frame_ancestor = *it;
      if (parent_frame_ancestor == mutual_ancestor)
      {
        break;
      }
      trafo_chain.push_back(parent_frame_ancestor);
      auto cur_trafo =
          managed_shm.find<TransformationBuffer>(parent_frame_ancestor.c_str()).first->getTransformation(stamp_nanosec);
      if (cur_trafo)
      {
        trafo = trafo * cur_trafo.value().inverse();
      }
      else
      { // Tranformation not found for this timestamp
        return {};
      }
    }

    std::cout << "trafo chain is: " << std::endl;
    for (const auto& elem : trafo_chain)
    {
      std::cout << elem << std::endl;
    }

    return trafo;
  };

private:
  managed_shared_memory managed_shm{open_or_create, SHM_SEGMENT_NAME, 10240};
  void_allocator alloc_inst{managed_shm.get_segment_manager()};

  std::vector<std::string> getAncestors(const std::string& frame_id)
  {
    std::vector<std::string> ancestor_list;
    std::pair<TransformationBuffer*, std::size_t> p = managed_shm.find<TransformationBuffer>(frame_id.c_str());
    if (p.first)
    {
      ancestor_list = getAncestors(p.first->parent_frame_id.c_str());
    }
    ancestor_list.push_back(frame_id);
    return ancestor_list;
  }
};
