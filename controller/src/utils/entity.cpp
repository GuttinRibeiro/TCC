#include "entity.hpp"

Entity::Entity(int frequency) {
  // Create a separate thread executing entity
  std::thread{std::bind(&Entity::execute, this, std::placeholders::_1), frequency}.detach();
}

[[noreturn]] void Entity::execute(int frequency) {
  // Calculate the time window for a given frequency in Hz
  const auto timeWindow = std::chrono::milliseconds(1000/frequency);

  // Call configure to allocate objects and initialize internal variables
  configure();

  // Infinite loop, which justifies the decision of declaring this function as [[noreturn]]
  while (true) {
    auto start = std::chrono::steady_clock::now();
    run();
    auto elapsed = std::chrono::steady_clock::now() - start;
    auto timeToWait = timeWindow - elapsed;
    if(timeToWait > std::chrono::milliseconds::zero()) {
      std::this_thread::sleep_for(timeToWait);
    } else {
      std::cout << "[" +name()+ "] Required frequency is too high!\n";
    }
  }
}
