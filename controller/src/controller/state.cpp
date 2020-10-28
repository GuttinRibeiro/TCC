#include "state.hpp"

State::State(Controller *ctr, int frequency) : Entity(frequency){
  if(ctr != nullptr) {
    _ctr = ctr;
  }
}

void State::goNext() {
  _ctr->nextState(this->nextState());
}
