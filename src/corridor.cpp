/**
 * @author    Aldrin I. Racelis
 * @copyright Aldrin Racelis (c)2017
 * @file      corridor.cpp
 * @brief     Class Implementation of the Corridor Environment
 */

#include "../include/corridor.h"

/**
 * @brief Standard Constructor
 * @param rand Random Number generator
 * @param rewardType Create -1 per step and 0 on termination (vs 0 and 1)
 */
Corridor::Corridor(Random &rand, bool rewardType):
  grid(create_default_map()),
  goal(coord_t(6., 5.)),
  negReward(rewardType),
  rng(rand),
  doorway(coord_t(2., 5.)),
  s(2),
  ns(s[0]),
  ew(s[1]),
  current_dir(EAST),
  numactions(3) {
  reset();
}


Corridor::~Corridor() { delete grid; }

/**
 * @brief Returns the state
 */
const std::vector<float> &Corridor::sensation() const {
  cout << "At state " << s[0] << ", " << s[1] << endl;

  return s;
}

/**
 * @brief Applies the action to change the state of the agent depending on the
 * direction the agent is facing.
 * @param action taken
 */
float Corridor::apply(int action) {
  ROS_INFO("Taking action %d ", static_cast<room_action_t>(action));
  ROS_INFO("Current Direction %d ", static_cast<robot_direction>(current_dir));

  int actUsed = action;

  if (actUsed > -1) {
    const room_action_t effect = static_cast<room_action_t>(actUsed);
    switch (effect) {
    case FORWARD:
      if (current_dir == NORTH) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew),
                        0)) {
          ++ns;
        }
        return reward();
      } else if (current_dir == WEST) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew), 3)) {
          --ew;
        }
        return reward();
      } else if (current_dir == SOUTH) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew), 1)) {
          --ns;
        }
        return reward();
      } else if (current_dir == EAST) {
        if (!grid->wall(static_cast<unsigned>(ns),
                        static_cast<unsigned>(ew), 2)) {
          ++ew;
        }
        return reward();
      }
    case TURNLEFT:
      if (current_dir == EAST)
        current_dir = NORTH;
      else
        current_dir = static_cast<robot_direction>(static_cast<int>
          (current_dir) + 1);
      return reward();
    case TURNRIGHT:
      if (current_dir == NORTH)
        current_dir = EAST;
      else
        current_dir = static_cast<robot_direction>(static_cast<int>
          (current_dir) - 1);
      return reward();
    }

    std::cerr << "Unreachable point reached in Corridor::apply!!!\n";
  }

  return 0;
}

/**
 * @brief return the reward for this move
 */
float Corridor::reward() {
  if (negReward) {
    // normally -1 and 0 on goal
    if (terminal())
      return 0;
    else
      return -1;

  } else {
    // or we could do 0 and 1 on goal
    if (terminal())
      return 1;
    else
      return 0;
  }
}

/**
 * @brief Determines if the goal was reached
 */
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


/**
 * @brief Returns the number of actions available
 */
int Corridor::getNumActions() {
  return numactions;
}


/**
 * @brief Creates the map for the corridor
 */
const Gridworld *Corridor::create_default_map() {
  int width = 7;
  int height = 7;
  std::vector<std::vector<bool> > nsv(width, std::vector<bool>(height-1,
        false));
  std::vector<std::vector<bool> > ewv(height, std::vector<bool>(width-1,
        false));

  // put the wall between the two rooms
  for (int j = 0; j < width; j++) {
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

/**
 * @brief Determines the min and max range
 */
void Corridor::getMinMaxFeatures(std::vector<float> *minFeat,
       std::vector<float> *maxFeat) {
  minFeat->resize(s.size(), 0.0);
  maxFeat->resize(s.size(), 10.0);

  (*maxFeat)[0] = 5.0;
}

/**
 * @brief Determines the min and max reward values
 * @param the minimum reward
 * @param the maximum reward
 */
void Corridor::getMinMaxReward(float *minR, float *maxR) {
  if (negReward) {
    *minR = -1.0;
    *maxR = 0.0;
  } else {
    *minR = 0.0;
    *maxR = 1.0;
  }
}

/**
 * @brief Seeds the experience vector
 */
std::vector<experience> Corridor::getSeedings() {
  // return seedings
  std::vector<experience> seeds;

  // if (true)
  // return seeds;
  // REMOVE THIS TO USE SEEDINGS

  actHistory.clear();
  seeds.push_back(getExp(6, 6, TURNLEFT));
  actHistory.clear();
  seeds.push_back(getExp(4, 2, TURNRIGHT));
  actHistory.clear();
  seeds.push_back(getExp(2, 1, FORWARD));

  reset();

  return seeds;
}

/**
 * @brief Creates an experience tuple for the given state-action.
 * @param state of the vertical position
 * @param state of the horizontal position
 * @param action
 */
experience Corridor::getExp(float s0, float s1, int a) {
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

  cout << "Seed from " << e.s[0] << "," << e.s[1] << " a: " << e.act

         << " r: " << e.reward << " term: " << e.terminal << endl;

  reset();

  return e;
}
