#include "state_halt.hpp"

State_Halt::State_Halt(BehavioralNode *bh) : State(bh){

}

void State_Halt::runState() {
  bhAccess()->goTo(bhAccess()->infoBus()->myPosition());
}

int State_Halt::nextState() {
  return States::HALT;
}

State_Halt::~State_Halt() {

}
