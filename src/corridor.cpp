#include "../include/corridor.h"

Corridor::Corridor(Random &rand, bool rewardType):
  grid(create_default_map()),
  goal(coord_t(7.,7.)),
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

const std::vector<float> &Corridor::sensation() const {
  //cout << "At state " << s[0] << ", " << s[1] << endl;

  return s;
}

float Corridor::apply(int action) {

  int actUsed = action;

  if (actUsed > -1){
    const room_action_t effect = static_cast<room_action_t>(actUsed);
    switch(effect) {
    case FORWARD:
      if (current_dir == NORTH) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew),
                        current_dir)) {
    	  ++ns;
        }
        return reward();
      } else if (current_dir == WEST) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew),
	  				    current_dir)) {
        	++ew;
        }
        return reward();
      } else if (current_dir == SOUTH) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew),
					    current_dir)) {
    	  --ns;
        }
        return reward();
      } else if (current_dir == EAST) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew),
					    current_dir)) {
    	  ++ew;
        }
        return reward();
      }
    case TURNLEFT:
      if (current_dir == EAST)
    	  current_dir = NORTH;
      else
    	  current_dir = static_cast<robot_direction>(static_cast<int>(current_dir) + 1);
      return reward();
    }

    std::cerr << "Unreachable point reached in Corridor::apply!!!\n";
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
  return 2;
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
  }
  // add a doorway
  doorway = coord_t(2, 5);
  return new Gridworld(height, width, nsv, ewv);
}

void Corridor::getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat) {
  minFeat->resize(s.size(), 0.0);
  maxFeat->resize(s.size(), 10.0);

  (*maxFeat)[0] = 5.0;
}

void Corridor::getMinMaxReward(float *minR, float *maxR) {
  if (negReward) {
    *minR = -1.0;
    *maxR = 0.0;
  } else {
    *minR = 0.0;
    *maxR = 1.0;
  }
}


std::vector<experience> Corridor::getSeedings() {

  // return seedings
  std::vector<experience> seeds;

  //if (true)
  // return seeds;
  // REMOVE THIS TO USE SEEDINGS

  // single seed of terminal state
  actHistory.clear();
  seeds.push_back(getExp(6,6,TURNLEFT));

  // single seed of doorway
  actHistory.clear();
  seeds.push_back(getExp(2,1,FORWARD));

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

  reset();

  return e;
}
