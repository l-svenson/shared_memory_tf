#include <shared_memory_tf/tf_manager.hpp>

using namespace boost::interprocess;

int main(int /*argc*/, char** /*argv*/)
{
  shared_memory_object::remove(SHM_SEGMENT_NAME);
  managed_shared_memory(open_or_create, SHM_SEGMENT_NAME, 10240);
  std::cout << "Successfully reset shared memory tf buffer! --> Exiting normally." << std::endl;
}
