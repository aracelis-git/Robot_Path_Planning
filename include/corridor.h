#ifndef INCLUDE_CORRIDOR_H_
#define INCLUDE_CORRIDOR_H_


#include <set>
#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include "gridworld.hh"

#include <deque>

class Corridor: public Environment {
public:

  /** Standard Constructor
      \param rand Random Number generator
      \param rewardType Create -1 per step and 0 on termination (vs 0 and 1)
  */
  Corridor(Random &rand, bool rewardType);

  virtual ~Corridor();

  virtual const std::vector<float> &sensation() const;
  virtual float apply(int action);

  virtual bool terminal() const;
  virtual void reset();

  virtual int getNumActions();
  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
  virtual void getMinMaxReward(float* minR, float* maxR);

  /** Create an experience tuple for the given state-action. */
  experience getExp(float s0, float s1, int a);

  virtual std::vector<experience> getSeedings();

protected:
  typedef std::pair<float,float> coord_t;
  enum room_action_t {NORTH, SOUTH, EAST, WEST};

private:
  const Gridworld *const grid;
  coord_t goal;
  std::deque<int> actHistory;

  const bool negReward;

  Random &rng;

  coord_t doorway;

  std::vector<float> s;

  float &ns;
  float &ew;

  /** Create default two room gridworld */
  const Gridworld *create_default_map();


  /** Return the correct reward based on the current state. */
  float reward();

};


#endif /* INCLUDE_CORRIDOR_H_ */
