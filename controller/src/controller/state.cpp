#include "state.hpp"

State::State(Controller *ctr){
  if(ctr != nullptr) {
    _ctr = ctr;
  }
}

void State::goNext() {
  _ctr->nextState(this->nextState());
}

State::~State() {
  _ctr = nullptr;
}
