#include <shared_memory_tf/tf.hpp>

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

volatile sig_atomic_t stop;

void inthand(int signum)
{
  stop = 1;
}

using namespace boost::interprocess;

#define SHM_SEGMENT_NAME "shared_memory_tf"
#define FRAME_ID "base_link"
#define PARENT_FRAME_ID "map"

int main(int argc, char** argv)
{
  std::srand(std::time(nullptr));

  if (argc > 1)
  {
    shared_memory_object::remove(SHM_SEGMENT_NAME);
  }

  managed_shared_memory managed_shm{open_or_create, SHM_SEGMENT_NAME, 10240};
  std::cout << "Created managed_shared_memory object." << std::endl;

  signal(SIGINT, inthand);

  while (!stop)
  {
    for (auto it = managed_shm.named_begin(); it != managed_shm.named_end(); ++it)
    {
      std::string_view name(it->name(), it->name_length());
      std::cout << "frame_id `" << name << "`:" << std::endl;

      std::pair<TransformationBuffer*, std::size_t> p = managed_shm.find<TransformationBuffer>(it->name());
      if (p.first)
      {
        std::cout << "  parent: `" << p.first->parent_frame_id << "`" << std::endl;

        Transformation trafo = p.first->transformations[p.first->current_index];

        trafo.stamp_nanosec = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        trafo.translation.x += std::rand() / ((RAND_MAX + 1U) / 10);
        trafo.translation.y += std::rand() / ((RAND_MAX + 1U) / 10);
        trafo.translation.z += std::rand() / ((RAND_MAX + 1U) / 10);

        trafo.rotation.q_x += 1;

        Transformation* array = p.first->transformations;

        for (std::size_t i = 0; i < TransformationBuffer::LENGTH; i++)
        {
          std::cout << "  element [" << i << "]=" << array[i] << std::endl;
        }

        const auto req_stamp =
            (std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(200)).time_since_epoch().count();
        std::optional<Eigen::Isometry3d> H = p.first->getTransformation(req_stamp);

        std::cout << "stamp=" << req_stamp << std::endl;

        if (H)
        {
          std::cout << "H = " << std::endl << H->matrix() << std::endl;
        }
        else
        {
          std::cout << "Could not interpolate to the given stamp...." << std::endl;
        }
      }
    }

    std::cout << "=========================" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
