/**
 * @author    Aldrin I. Racelis
 * @copyright Aldrin Racelis (c)2017
 * @file      corridor.h
 * @brief     Header for the corridor Class
 */

#ifndef SRC_ROBOT_PATH_PLANNING_INCLUDE_CORRIDOR_H_
#define SRC_ROBOT_PATH_PLANNING_INCLUDE_CORRIDOR_H_

#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <set>
#include <deque>
#include <vector>
#include <utility>
#include "../include/rl_env/gridworld.hh"

class Corridor: public Environment {
 public:
  Corridor(Random &rand, bool rewardType);
  virtual ~Corridor();
  virtual const std::vector<float> &sensation() const;
  virtual float apply(int action);
  virtual bool terminal() const;
  virtual void reset();
  virtual int getNumActions();
  virtual void getMinMaxFeatures(std::vector<float> *minFeat,
                                   std::vector<float> *maxFeat);
  virtual void getMinMaxReward(float* minR, float* maxR);
  experience getExp(float s0, float s1, int a);
  virtual std::vector<experience> getSeedings();

 protected:
  typedef std::pair<float, float> coord_t;
  enum room_action_t {FORWARD, TURNLEFT, TURNRIGHT};
  enum robot_direction {NORTH, WEST, SOUTH, EAST};
  robot_direction current_dir;

 private:
  const Gridworld *const grid;
  coord_t goal;
  std::deque<int> actHistory;
  int numactions;
  const bool negReward;
  Random &rng;
  coord_t doorway;
  std::vector<float> s;
  float &ns;
  float &ew;
  const Gridworld *create_default_map();
  float reward();
};


#endif /* SRC_ROBOT_PATH_PLANNING_INCLUDE_CORRIDOR_H_ */
