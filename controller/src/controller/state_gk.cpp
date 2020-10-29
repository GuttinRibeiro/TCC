#include "state_gk.hpp"

State_GK::State_GK(Controller *ctr) : State (ctr){

}

int State_GK::nextState() {
  return States::ATK;
}

void State_GK::runState() {
  ctrAccess()->goToLookTo(ctrAccess()->infoBus()->ourGoal(), ctrAccess()->infoBus()->ballPosition());
}

State_GK::~State_GK() {

}
