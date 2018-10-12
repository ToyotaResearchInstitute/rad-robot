#pragma once

typedef struct {
  int ioffset;
  int joffset;
  double distance;
} NeighborStruct;

#ifdef __cplusplus
extern "C"
#endif
int dijkstra_holonomic(double* cost_to_go, // output
                       double* costmap, //input
                       unsigned int m, unsigned int n,
                       int* p_goal, int* p_start,
                       int nNeighbors);

int dijkstra_nonholonomic(double* cost_to_go, // output
                          double* costmap, //input
                          unsigned int m, unsigned int n,
                          int* p_goal, int* p_start,
                          int nNeighbors);
