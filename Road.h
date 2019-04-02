#ifndef _ROAD_H
#define _ROAD_H

#include <stdio.h>
#include <cmath>
#include "Node.h"
using namespace std;

class Road
{
public:
  Road(int m_id1, int m_id2, int m_l1, int m_l2, int m_t, int m_gt) 
  {
		flow = 0;
    id1 = m_id1;
    id2 = m_id2;
    lane1 = m_l1;
    lane2 = m_l2;
    light_time = m_t;
    green_time = m_gt;
  }
  ~Road() {}

  void cal_length_slope(float x1, float x2, float y1, float y2) 
  {
    float delta_x = x2 - x1;
    float delta_y = y2 - y1;
// consider the axis on the canvas for details
    slope = atan2(delta_y, delta_x);
    slope = slope * 180 / 3.14159265;
    if (slope < 0)
      slope = slope + 360;
    length = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  }

  void judge_jam()
  {
    if (1) jamOrNot = 1;
    else jamOrNot = 0;    // to be developed
  }

  void cal_ave_velocity()
  {
    // to be developed
  }

  int id1;
  int id2;
  int lane1;              // the number of lanes on the way id1 --> id2
  int lane2;              // the number of lanes on the way id2 --> id1
  float length;           // the real length (euclidean distance of node[id1] & node[id2])
  float slope;            // the slope on the screen (UI)
  int light_time;         // the whole period of traffic light (green+red, yellow is not implemented)
  int green_time;         // the green light time of the traffic light
  int flow;               // the vehicle numbers on the road
  int jamOrNot;           // whether there is a jam on the road
  float ave_velocity;     // the average velocity of the vehicles on the road present
};

#endif
