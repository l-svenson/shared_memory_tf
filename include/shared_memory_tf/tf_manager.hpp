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
      p = managed_shm.find<TransformationBuffer>(frame_id.c_str());
    }
    else if ((std::string(p.first->parent_frame_id.c_str()) != parent_frame_id))
    {
      std::cout << "error! Parent frame id not matching! parent_frame_id is: " << p.first->parent_frame_id.c_str() << " but "
                << parent_frame_id.c_str() << " was expected!" << std::endl;
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
        {
          // Found mutual ancestor
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
    Eigen::Isometry3d trafo = Eigen::Isometry3d::Identity();

    for (auto it = parent_frame_ancestors.rbegin(); it != parent_frame_ancestors.rend(); it++)
    {
      const auto& parent_frame_ancestor = *it;
      if (parent_frame_ancestor == mutual_ancestor)
      {
        break;
      }
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
    bool passed_mutual = false;
    for (auto& frame_ancestor : frame_ancestors)
    {
      if (!passed_mutual)
      {
        passed_mutual = (frame_ancestor == mutual_ancestor);
        continue;
      }
      auto cur_trafo = managed_shm.find<TransformationBuffer>(frame_ancestor.c_str()).first->getTransformation(stamp_nanosec);
      if (cur_trafo.has_value())
      {
        trafo = trafo * cur_trafo.value();
      }
      else
      {
        // Tranformation not found for this timestamp
        return {};
      }
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
