/** \file TwoRooms.cc
    Implements a two room gridworld domain, with possible action delays or
    multiple goals (with partial observability).
    \author Todd Hester
*/


#include "../include/corridor.h"

Corridor::Corridor(Random &rand, bool rewardType):
  grid(create_default_map()),
  goal(coord_t(1.,1.)),
  negReward(rewardType),
  rng(rand),
  doorway(coord_t(2.,5.)),
  s(2),
  ns(s[0]),
  ew(s[1])
{
  reset();
}


Corridor::~Corridor() { delete grid; }

const std::vector<float> &Environment::sensation() const {
  //cout << "At state " << s[0] << ", " << s[1] << endl;

  return s;
}

float Corridor::apply(int action) {

  //cout << "Taking action " << static_cast<room_action_t>(action) << endl;

  int actUsed = action;

  if (actUsed > -1){

    switch(effect) {
    case NORTH:
      if (!grid->wall(static_cast<unsigned>(ns),
                      static_cast<unsigned>(ew),
                      effect))
        {
          ++ns;
        }
      return reward();
    case SOUTH:
      if (!grid->wall(static_cast<unsigned>(ns),
                      static_cast<unsigned>(ew),
                      effect))
        {
          --ns;
        }
      return reward();
    case EAST:
      if (!grid->wall(static_cast<unsigned>(ns),
                      static_cast<unsigned>(ew),
                      effect))
        {
          ++ew;

      return reward();
    case WEST:
      if (!grid->wall(static_cast<unsigned>(ns),
                      static_cast<unsigned>(ew),
                      effect))
        {
          --ew;
        }
      return reward();
    }

    std::cerr << "Unreachable point reached in TwoRooms::apply!!!\n";
  }

  return 0;
}

// return the reward for this move
float Corridor::reward() {
  /*
  if (coord_t(ns,ew) == goal2)
    cout << "At goal 2, " << useGoal2 << endl;
  if (coord_t(ns,ew) == goal)
    cout << "At goal 1, " << !useGoal2 << endl;
  */

  if (negReward){
    // normally -1 and 0 on goal
    if (terminal())
      return 0;
    else
      return -1;

  }else{
    // or we could do 0 and 1 on goal
    if (terminal())
      return 1;
    else
      return 0;
  }
}



bool Corridor::terminal() const {
  // current position equal to goal??
  return coord_t(ns, ew) == goal;
}

// start at the beginning
void Corridor::reset() {
  ns = rng.uniformDiscrete(0, 2);
  ew = 0;

  // a history of no_acts
  actHistory.clear();
}



int Corridor::getNumActions(){
  return 4;
}


const Gridworld *Corridor::create_default_map() {
  int width = 7;
  int height = 7;
  std::vector<std::vector<bool> > nsv(width, std::vector<bool>(height-1,false));
  std::vector<std::vector<bool> > ewv(height, std::vector<bool>(width-1,false));

  // put the wall between the two rooms
  for (int j = 0; j < width; j++){
    // skip doorway
    if (j == 0) {
      nsv[3][j] = true;
    } else if (j == width) {
        nsv[1][j] = true;
        nsv[5][j] = true;
    } else {
      nsv[1][j] = true;
      nsv[3][j] = true;
      nsv[5][j] = true;
  }

  // add a doorway
  doorway = coord_t(2, 5);
  return new Gridworld(height, width, nsv, ewv);
}

void Corridor::getMinMaxFeatures(std::vector<float> *minFeat,
                                 std::vector<float> *maxFeat){

  minFeat->resize(s.size(), 0.0);
  maxFeat->resize(s.size(), 10.0);

  (*maxFeat)[0] = 5.0;

}

void Corridor::getMinMaxReward(float *minR,
                              float *maxR){
  if (negReward){
    *minR = -1.0;
    *maxR = 0.0;
  }else{
    *minR = 0.0;
    *maxR = 1.0;
  }
}


std::vector<experience> Environment::getSeedings() {

  // return seedings
  std::vector<experience> seeds;

  //if (true)
  // return seeds;
  // REMOVE THIS TO USE SEEDINGS

  // single seed of terminal state
  useGoal2 = false;
  actHistory.clear();
  actHistory.assign(actDelay, SOUTH);
  seeds.push_back(getExp(2,1,SOUTH));

  // possible seed of 2nd goal
  if (multiGoal){
    useGoal2 = true;
    actHistory.clear();
    actHistory.assign(actDelay, NORTH);
    seeds.push_back(getExp(3,1,NORTH));
  }

  // single seed of doorway
  actHistory.clear();
  actHistory.assign(actDelay, WEST);
  seeds.push_back(getExp(2,6,WEST));

  reset();

  return seeds;

}

experience Corridor::getExp(float s0, float s1, int a){

  experience e;

  e.s.resize(2, 0.0);
  e.next.resize(2, 0.0);

  ns = s0;
  ew = s1;

  e.act = a;
  e.s = sensation();
  e.reward = apply(e.act);

  e.terminal = terminal();
  e.next = sensation();

  /*
  cout << "Seed from " << e.s[0] << "," << e.s[1] << " a: " << e.act
       << " r: " << e.reward << " term: " << e.terminal << endl;
  */

  reset();

  return e;
}
