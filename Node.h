#ifndef _NODE_H
#define _NODE_H

#include <stdio.h>
#include <cmath>

class Node
{
public:
  Node(int m_id, float m_x, float m_y) 
  {
    id = m_id;
    axis_real_x = m_x;
    axis_real_y = m_y;
    link = new int [4];
    dist = new float [4];
    for (int i = 0; i < 4; i++)
    {
      link[i] = -1;
      dist[i] = -1.0;
    }
  }
  ~Node() {
      delete [] link;
      delete [] dist;
  }

  void add_node(int m_next_id, float x, float y)
  {
    int i;
    for (i = 0; i < 4; i++)
    {
      if (link[i] == -1)
        break;
    }
    link[i] = m_next_id;
    dist[i] = sqrt((x-axis_real_x)*(x-axis_real_x)+(y-axis_real_y)*(y-axis_real_y));
  }

  int id;                  
//  char** name;             // can be presented on screen (UI)
  float axis_real_x;
  float axis_real_y;       // axis read from data file
  float axis_canvas_x;   
  float axis_canvas_y;     // axis for UI
  int* link;               // which constrains that a node has 4 links at most
  float* dist;
};

#endif
