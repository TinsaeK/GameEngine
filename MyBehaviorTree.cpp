#include "MyBehaviorTree.h"


bool MySelectorNode::run()
{
  //TODO: implement this

    bool a =false;
    for (MyTaskNode* b : children) {
        a = b->run();
        if (a == true) break;
    }
  return a;
}

bool MySequenceNode::run()
{

  //TODO: implement this
    bool a = true;
    for (MyTaskNode* b : children) {
        a = b->run();
        if (a == false) break;
    }
  return a;
}

//Note: You are free to create more types of nodes if needed