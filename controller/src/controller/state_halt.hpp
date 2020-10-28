#ifndef STATE_HALT_HPP
#define STATE_HALT_HPP

#include "state.hpp"
#include "../utils/states.hpp"
#include <string>

class State_Halt : public State {
public:
  State_Halt(Controller *ctr);
  ~State_Halt();
  int nextState();
  void runState();
};

#endif // STATE_HALT_HPP
