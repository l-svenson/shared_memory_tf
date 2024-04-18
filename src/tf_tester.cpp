#include <shared_memory_tf/tf.hpp>
#include <shared_memory_tf/tf_manager.hpp>

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

volatile sig_atomic_t stop;

void inthand(int /*signum*/)
{
  stop = 1;
}
#define SHM_SEGMENT_NAME "shared_memory_tf"
using namespace boost::interprocess;

int main(int /*argc*/, char** /*argv*/)

{

  shared_memory_object::remove(SHM_SEGMENT_NAME);
  SharedMemoryTFManager tf_manager;

  tf_manager.addTransform(1, "world", "map", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "map", "odom", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "odom", "base_link_1", {1, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "base_link_1", "sensor1", {0, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "odom", "base_link_2", {-1, 0, 1}, {1, 0, 0, 0});
  tf_manager.addTransform(1, "base_link_2", "sensor2", {0, 0, 1}, {1, 0, 0, 0});

  std::string from_frame = "base_link_1";
  std::string to_frame = "sensor2";

  auto tf1 = tf_manager.getTransform(1, from_frame, to_frame);

  if (tf1)
  {
    std::cout << "Found Tf from " << from_frame << " to " << to_frame << " is: \n" << tf1.value().matrix() << std::endl;
  }
  else
  {
    std::cout << " Could not find TF from " << from_frame << " to " << to_frame << std::endl;
  }

  auto tf2 = tf_manager.getTransform(1, to_frame, from_frame);

  if (tf1)
  {
    std::cout << "Found Tf from " << to_frame << " to " << from_frame << " is: \n" << tf2.value().matrix() << std::endl;
  }
  else
  {
    std::cout << " Could not find TF from " << to_frame << " to " << from_frame << std::endl;
  }
}
