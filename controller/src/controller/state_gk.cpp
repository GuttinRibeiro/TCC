#include "state_gk.hpp"

State_GK::State_GK(BehavioralNode *bh) : State (bh){

}

int State_GK::nextState() {
  return States::ATK;
}

void State_GK::runState() {
  bhAccess()->goToLookTo(bhAccess()->infoBus()->ourGoal(), bhAccess()->infoBus()->theirGoal());
}

State_GK::~State_GK() {

}
