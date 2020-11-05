#include "state_halt.hpp"

State_Halt::State_Halt(BehavioralNode *bh) : State(bh){

}

void State_Halt::runState() {
//  bhAccess()->goTo(bhAccess()->infoBus()->myPosition());
  std::cout << "State halt running!\n";
  Vector dest = bhAccess()->infoBus()->theirGoal();
  bhAccess()->goTo(dest);
  std::cout << "" << dest.x() << ", " << dest.y() << "\n";
}

int State_Halt::nextState() {
  return States::HALT;
}

State_Halt::~State_Halt() {

}
