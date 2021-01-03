#ifndef STATE_HALT_HPP
#define STATE_HALT_HPP

#include "state.hpp"
#include "../utils/states.hpp"

class State_Halt : public State {
public:
  State_Halt(BehavioralNode *bh);
  ~State_Halt();
  int nextState();
  void runState();
};

#endif // STATE_HALT_HPP
