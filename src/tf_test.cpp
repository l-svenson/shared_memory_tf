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

#define object_name "foo"

int main(int argc, char** argv)
{
  std::srand(std::time(nullptr));

  if (argc > 1)
  {
    shared_memory_object::remove("Boost");
  }

  managed_shared_memory managed_shm{open_or_create, "shared_memory_tf", 10240};
  std::cout << "Created managed_shared_memory object." << std::endl;

  std::pair<TransformationBuffer*, std::size_t> p_ = managed_shm.find<TransformationBuffer>(object_name);
  if (p_.first)
  {
    std::cout << "I found '" << object_name << "' already!" << std::endl;
  }
  else
  {
    std::cout << "Could not find '" << object_name << "' yet... I will create it now!" << std::endl;
    managed_shm.construct<TransformationBuffer>(object_name)();
  }

  std::cout << "# objects = " << managed_shm.get_segment_manager()->get_num_named_objects() << std::endl;

  signal(SIGINT, inthand);

  while (!stop)
  {
    for (auto it = managed_shm.named_begin(); it != managed_shm.named_end(); ++it)
    {
      std::string_view name(it->name(), it->name_length());
      std::cout << "Item `" << name << "`:" << std::endl;

      std::pair<TransformationBuffer*, std::size_t> p = managed_shm.find<TransformationBuffer>(it->name());
      if (p.first)
      {
        Transformation trafo = p.first->transformations[p.first->current_index];

        trafo.stamp_nanosec = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        trafo.translation.t_x += std::rand() / ((RAND_MAX + 1U) / 10);
        trafo.translation.t_y += std::rand() / ((RAND_MAX + 1U) / 10);
        trafo.translation.t_z += std::rand() / ((RAND_MAX + 1U) / 10);

        trafo.rotation.q_x += 1;

        p.first->addTransformation(trafo);

        Transformation* array = p.first->transformations;

        for (std::size_t i = 0; i < TransformationBuffer::LENGTH; i++)
        {
          std::cout << "  element [" << i << "]=" << array[i] << std::endl;
        }
      }
    }

    std::cout << "=========================" << std::endl;
  }
}
