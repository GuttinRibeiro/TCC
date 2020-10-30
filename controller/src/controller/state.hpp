#ifndef STATE_HPP
#define STATE_HPP

#include "behavioralnode.hpp"
#include <string>

class State {
private:
  BehavioralNode *_bh;

protected:
  BehavioralNode * bhAccess() const {return _bh;}
public:
  virtual int nextState() = 0;
  State(BehavioralNode *bh);
  virtual ~State();
  void goNext();
  virtual void runState() = 0;
};

#endif // STATE_HPP
