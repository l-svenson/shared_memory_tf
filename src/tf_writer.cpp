#include <shared_memory_tf/tf.hpp>

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

volatile sig_atomic_t stop;

void inthand(int /*signum*/)
{
  stop = 1;
}

using namespace boost::interprocess;

#define SHM_SEGMENT_NAME "shared_memory_tf"
#define FRAME_ID "base_link"
#define PARENT_FRAME_ID "map"

int main(int argc, char** /*argv*/)
{
  std::srand(std::time(nullptr));

  if (argc > 1)
  {
    shared_memory_object::remove(SHM_SEGMENT_NAME);
  }

  managed_shared_memory managed_shm{open_or_create, SHM_SEGMENT_NAME, 10240};
  std::cout << "Created managed_shared_memory object." << std::endl;

  // An allocator convertible to any allocator<T, segment_manager_t> type
  void_allocator alloc_inst(managed_shm.get_segment_manager());

  std::pair<TransformationBuffer*, std::size_t> p_ = managed_shm.find<TransformationBuffer>(FRAME_ID);
  if (p_.first)
  {
    std::cout << "I found '" << FRAME_ID << "' already!" << std::endl;
  }
  else
  {
    std::cout << "Could not find '" << FRAME_ID << "' yet... I will create it now!" << std::endl;
    managed_shm.construct<TransformationBuffer>(FRAME_ID)("foo", alloc_inst);
  }

  std::cout << "# objects = " << managed_shm.get_segment_manager()->get_num_named_objects() << std::endl;

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
        trafo.rotation.q_y += 2;
        trafo.rotation.q_z += 3;
        trafo.rotation.q_w += 4;

        trafo.rotation.normalize();

        p.first->addTransformation(trafo);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Transformation* array = p.first->transformations;

        // for (std::size_t i = 0; i < TransformationBuffer::LENGTH; i++)
        // {
        //   std::cout << "  element [" << i << "]=" << array[i] << std::endl;
        // }
      }
    }

    std::cout << "=========================" << std::endl;
  }
}
