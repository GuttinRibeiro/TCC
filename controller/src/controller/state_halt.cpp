#include "state_halt.hpp"

State_Halt::State_Halt(Controller *ctr) : State(ctr){

}

void State_Halt::runState() {
  ctrAccess()->goTo(ctrAccess()->infoBus()->myPosition());
}

int State_Halt::nextState() {
  return States::HALT;
}

State_Halt::~State_Halt() {

}
