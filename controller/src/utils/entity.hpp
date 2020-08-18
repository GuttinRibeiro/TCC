#ifndef ENTITY_HPP
#define ENTITY_HPP

#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <iostream>

class Entity {
private:
  int _frequency;
  [[noreturn]] void execute(int frequency);
private:
  virtual void run() = 0;
  virtual void configure() = 0;
  virtual std::string name() = 0;
public:
  Entity(int frequency);
  void start();
};

#endif // ENTITY_HPP
