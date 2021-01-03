#include "state.hpp"

State::State(BehavioralNode *bh){
  if(bh != nullptr) {
    _bh = bh;
  }
}

void State::goNext() {
  _bh->nextState(this->nextState());
}

State::~State() {
  _bh = nullptr;
}
