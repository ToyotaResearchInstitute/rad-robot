// C parts
#include <math.h>
#include <stddef.h>
// C++ parts
#include <set>

// For non-holonomic porion
#define TURN_COST 1.2
// #define TURN_COST 2.5
// #define TURN_COST 5
// #define TURN_COST 12
#define STOP_FACTOR 1.1

typedef std::pair<double, int> CostNodePair; // (cost, node)

typedef struct {
  int joffset;
  int ioffset;
  double distance;
} NeighborStruct;

// static const NeighborStruct neighbors[] = {
//   // 4-connected
//   {-1, 0, 1.0},
//   {1, 0, 1.0},
//   {0, -1, 1.0},
//   {0, 1, 1.0},
//   // 8-connected
//   {-1, -1, M_SQRT2},
//   {1, -1, M_SQRT2},
//   {-1, 1, M_SQRT2},
//   {1, 1, M_SQRT2},
//   // 16-connected
//   {2, 1, sqrt(5)},
//   {1, 2, sqrt(5)},
//   {-1, 2, sqrt(5)},
//   {-2, 1, sqrt(5)},
//   {-2, -1, sqrt(5)},
//   {-1, -2, sqrt(5)},
//   {1, -2, sqrt(5)},
//   {2, -1, sqrt(5)},
// };

static const NeighborStruct neighbors4[] = {
  // 4-connected
  {1, 0, 1.0},
  {0, 1, 1.0},
  {-1, 0, 1.0},
  {0, -1, 1.0},
};
static const NeighborStruct neighbors8[] = {
  {1,0,1.0}, {1,1,M_SQRT2},
  {0,1,1.0}, {-1,1,M_SQRT2},
  {-1,0,1.0}, {-1,-1,M_SQRT2},
  {0,-1,1.0}, {1,-1,M_SQRT2},
};
static const NeighborStruct neighbors16[] = {
  {1,0,1.0}, {2,1,sqrt(5)}, {1,1,M_SQRT2}, {1,2,sqrt(5)},
  {0,1,1.0}, {-1,2,sqrt(5)}, {-1,1,M_SQRT2}, {-2,1,sqrt(5)},
  {-1,0,1.0}, {-2,-1,sqrt(5)}, {-1,-1,M_SQRT2}, {-1,-2,sqrt(5)},
  {0,-1,1.0}, {1,-2,sqrt(5)}, {1,-1,M_SQRT2}, {2,-1,sqrt(5)},
};

// Run Dijkstra on an input matrix
// Returns an error code
#ifdef __cplusplus
extern "C"
#endif
    int
    dijkstra_holonomic(double *cost_to_go, // output
                    double *costmap,    // input
                    unsigned int m, unsigned int n,
                    int* p_goal, int* p_start,
                    int nNeighbors) {

  const NeighborStruct * neighbors;
  switch(nNeighbors) {
    case 16:
    neighbors = neighbors16;
    break;
    case 8:
    neighbors = neighbors8;
    break;
    case 4:
    default:
    nNeighbors = 4;
    neighbors = neighbors4;
    break;
  }

  // Goal
  int iGoal = p_goal[0];
  int jGoal = p_goal[1];
  // Check the boundaries
  if (iGoal >= m) {
    iGoal = m - 1;
  } else if (iGoal < 0) {
    iGoal = 0;
  }
  if (jGoal >= n) {
    jGoal = n - 1;
  } else if (jGoal < 0) {
    jGoal = 0;
  }
  // Start
  int iStart = -1;
  int jStart = -1;
  if (p_start != NULL) {
    iStart = p_start[0];
    jStart = p_start[1];
    // Check the boundaries
    if (iStart >= m) {
      iStart = m - 1;
    } else if (iStart < 0) {
      iStart = 0;
    }
    if (jStart >= n) {
      jStart = n - 1;
    } else if (jStart < 0) {
      jStart = 0;
    }
  }

  // Characterize the matrix with graph information
  size_t nMatrixNodes = m * n;
  // size_t nMatrixEdges = (nNeighbors / 2) * nMatrixNodes + m + n;

  // Form the cost-to-go map
  int i;
  for (i = 0; i < nMatrixNodes; i++) {
    cost_to_go[i] = INFINITY;
  }

  // Add the goal state
  // int indGoal = n * iGoal + jGoal;
  size_t indGoal = m * jGoal + iGoal;
  cost_to_go[indGoal] = 0;
  std::set<CostNodePair> Q; // Sorted set of (cost to go, node)
  Q.insert(CostNodePair(0, indGoal));

  // Iterate through the states
  while (!Q.empty()) {
    // Fetch closest node in queue
    CostNodePair top = *Q.begin();
    Q.erase(Q.begin());
    double c0 = top.first;
    int ind0 = top.second;

    // fprintf(stderr, "\n==\nNode %d \n", ind0);
    double cost0 = costmap[ind0];

    // Array subscripts of item
    // int i0 = ind0 / n;
    // int j0 = ind0 % n;
    int j0 = ind0 / m;
    int i0 = ind0 % m;

    // printf("Coord (%d, %d)\n", i0, j0);

    // Iterate over neighbor items
    int k;
    for (k = 0; k < nNeighbors; k++) {
      NeighborStruct nbr = neighbors[k];
      int ioffset = nbr.ioffset;
      int joffset = nbr.joffset;

      int i1 = i0 + ioffset;
      if ((i1 < 0) || (i1 >= m)) {
        continue;
      }
      int j1 = j0 + joffset;
      if ((j1 < 0) || (j1 >= n)) {
        continue;
      }

/*
      // Raytracing cost
      double d = neighbors[k].distance;
      int koffset = floor(d);
      for (int k = 1; k <= koffset; k++) {
        int ik = i0 + (k*ioffset)/koffset;
        int jk = j0 + (k*joffset)/koffset;
        int ind = m*jk+ik;
        // fprintf(stderr, "ij0: %d | ind: %d\n", ij0, ind);
        cost += costmap[ind];
      }
      // cost /= koffset;
      cost /= d;
*/

      // size_t ind1 = i1 * n + j1;
      size_t ind1 = j1 * m + i1;
      // fprintf(stderr ,"New item: %d\n", ind1);
      // Generate the new cost
      double avg = 0.5 * (cost0 + costmap[ind1]);
      // printf("Avg cost %f\n", avg);
      double c1 = c0 + avg * neighbors[k].distance;
      // fprintf(stderr, "\nNew cost %f\n", c1);
      // Heuristic cost:
      double h1 = p_start ? sqrt((iStart-i1)*(iStart-i1)+(jStart-j1)*(jStart-j1)) : 0;
      // Estimated total cost:
      double f1 = c1 + h1;
      double c2g = cost_to_go[ind1];
      // fprintf(stderr, "cost_to_go[%d] = %f\n", ind1, c2g);
      if (c1 < c2g) {
        // Check if the first time we are inspecting the item
        if (!isinf(c2g)) {
          Q.erase(Q.find(CostNodePair(c2g + h1, ind1)));
        }
        cost_to_go[ind1] = c1;
        Q.insert(CostNodePair(f1, ind1));
      } // Updating c2g
    }
    // fprintf(stderr, "Done iteration.\n");
  }
  // fprintf(stderr, "Done iterating!\n");

  // Yield the cost-to-go map
  return 0;
}

/*
  [cost_to_go] = dijkstra_nonholonomic(A, xya_goal, xya_start, [nNeighbors]);
  where positive costs are given in matrix A
  with nNeighbors different orientations.
*/
// Costmap is 2D, but the output is 3D, with turning?
#ifdef __cplusplus
extern "C"
#endif
    int
    dijkstra_nonholonomic(double *cost_to_go,
                          double *costmap, // input
                          unsigned int m, unsigned int n,
                          int *p_goal, int *p_start,
                          int nNeighbors) {

  /*
  Due to hopping in 16 neighbor pattern, need to seed
     a cluster of goal states--using 4 corners with (iGoal, jGoal)
     being one corner
     */

  const NeighborStruct * neighbors0;
  switch(nNeighbors) {
    case 16:
    neighbors0 = neighbors16;
    break;
    case 8:
    default:
    neighbors0 = neighbors8;
    break;
  }

  int shift_amount = (nNeighbors < 16) ? 1 : 2;

  // Goal
  int iGoal = p_goal[0];
  int jGoal = p_goal[1];
  // Check the boundaries
  if (iGoal > m - 2) {
    iGoal = m - 2;
  } else if (iGoal < 0) {
    iGoal = 0;
  }
  if (jGoal > n-2) {
    jGoal = n - 2;
  } else if (jGoal < 0) {
    jGoal = 0;
  }
  // Angular discretized
  int aGoal = p_goal[2];
  if (aGoal >= nNeighbors) {
    aGoal = nNeighbors - 1;
  } else if (aGoal < 0) {
    aGoal = 0;
  }
  // Start
  int iStart = -1;
  int jStart = -1;
  int aStart = -1;
  if (p_start != NULL) {
    iStart = p_start[0];
    // Check the boundaries
    if (iStart >= m) {
      iStart = m - 1;
    } else if (iStart < 0) {
      iStart = 0;
    }
    jStart = p_start[1];
    if (jStart >= n) {
      jStart = n - 1;
    } else if (jStart < 0) {
      jStart = 0;
    }
    aStart = p_start[2];
    if (aStart >= nNeighbors) {
      aStart = nNeighbors - 1;
    } else if (aStart < 0) {
      aStart = 0;
    }
  }

  // Map size
  size_t nMatrixNodes = m * n;

  // linear index
  int indGoalCostMap = m * jGoal + iGoal;
  int indGoal = nMatrixNodes * aGoal + indGoalCostMap;
#ifdef USE_SHORT_CIRCUIT
  int indStart = nMatrixNodes * aStart + m * jStart + iStart;
#endif
  // fprintf(stderr, "aStart: %d | aGoal: %d\n", aStart, aGoal);
  // fprintf(stderr, "nNeighbors: %d\n", nNeighbors);
  // Initiate cost to go values
  for (int i = 0; i < nNeighbors * nMatrixNodes; i++) {
    cost_to_go[i] = INFINITY;
  }

  // Priority queue implementation as STL set
  std::set<CostNodePair> Q; // Sorted set of (cost to go, node)
  // Seeding goal states
  cost_to_go[indGoal] = 0;
  Q.insert(CostNodePair(0, indGoal));

  // TODO: Ensure indices are all valid
  // 16 neighbors means some skipping
  if (nNeighbors > 8){
    int i1;
    for (i1=0; i1 < 4; i1++) {
      int jGoal1 = jGoal + neighbors4[i1].joffset;
      if (jGoal1 >= n || jGoal1 < 0) { continue; }
      int iGoal1 = iGoal + neighbors4[i1].ioffset;
      if (iGoal1 >= m || iGoal1 < 0) { continue; }
      int indGoalCostMap1 = m * jGoal1 + iGoal1;
      int indGoal1 = nMatrixNodes * aGoal + indGoalCostMap1;
      cost_to_go[indGoal1] = 0;
      Q.insert(CostNodePair(0, indGoal1));
    }
  }

  int n_directional_neighbors = 2 * shift_amount + 1;
#ifdef USE_BACKWARDS
  n_directional_neighbors *= 2;
#endif
  // fprintf(stderr, "n_directional_neighbors: %d\n", n_directional_neighbors);
  NeighborStruct neighbors_local[n_directional_neighbors];
  int angles_local[n_directional_neighbors];
  double costs_local[n_directional_neighbors];

  int nNode = 0;
  while (!Q.empty()) {
    nNode++;
    // fprintf(stderr, "nNode: %d | Size: %lu\n", nNode, Q.size());
    // Fetch closest node in queue
    CostNodePair top = *Q.begin();
    Q.erase(Q.begin());
    double c0 = top.first;
    int ind0 = top.second;

#ifdef USE_SHORT_CIRCUIT
    // Short circuit computation if path to start has been found:
    if (c0 > STOP_FACTOR * cost_to_go[indStart]) {
      break;
    }
#endif

    // Array subscripts of node:
    int a0 = ind0 / nMatrixNodes;
    int ij0 = ind0 - a0 * nMatrixNodes;
    int j0 = ij0 / m;
    int i0 = ij0 % m;
    // Find the cost of the index

    double costmap_of_self = costmap[ij0];
    // fprintf(stderr, "Check costmap with ij0=%d / %lu: %lf\n",
    //   ij0, nMatrixNodes, costmap_of_self);
    // fprintf(stderr, "Check costmap (%d, %d): %lf\n",
    //   i0, j0, costmap_of_self);

    int i_nbr = 0;
    // Non-negative heading index:
    for (int ashift = -shift_amount; ashift <= shift_amount; ashift++) {
      // Iterate over forward neighbor nodes:
      // (+ nNeighbors) so that the modulo works properly
      int a1 = (a0 + nNeighbors + ashift) % nNeighbors;
      angles_local[i_nbr] = a1;
      // fprintf(stderr, "i_nbr: %d | a0: %d | ashift: %d | a1: %d\n", i_nbr, a0, ashift, a1);
      neighbors_local[i_nbr] = neighbors0[a1];
      double dir_cost = (ashift == 0) ? 1 : (TURN_COST * abs(ashift));
      costs_local[i_nbr] = dir_cost;
      i_nbr++;
    }
#ifdef USE_BACKWARDS
    for (int ashift = -shift_amount; ashift <= shift_amount; ashift++) {
      // Backwards
      int a1 = (a0 + nNeighbors + ashift) % nNeighbors;
      angles_local[i_nbr] = a1;
      int a2 = (a0 + nNeighbors + ashift + nNeighbors / 2) % nNeighbors;
      neighbors_local[i_nbr] = neighbors0[a2];
      double dir_cost = (ashift == 0) ? 1 : (TURN_COST * abs(ashift));
      // fprintf(stderr, "Fwd: %d -> Back: %d | Cost: %lf\n", a1, a2, dir_cost);
      costs_local[i_nbr] = 4 * dir_cost;
      i_nbr++;
    }
#endif

    for (int i_nbr=0;i_nbr<n_directional_neighbors;i_nbr++){
      int a1 = angles_local[i_nbr];

      // fprintf(stderr, "a0: %d | ashift: %d | a1: %d\n", a0, ashift, a1);
      NeighborStruct nbr = neighbors_local[i_nbr];
      int ioffset = nbr.ioffset;
      int joffset = nbr.joffset;
      double dist_nbr = nbr.distance;

      // fprintf(stderr, "i_nbr %d | ioffset: %d, joffset: %d, dist: %lf\n",
      //   i_nbr, ioffset, joffset, dist_nbr);

      // Ensure valid within map dimensions
      int i1 = i0 - ioffset;
      if ((i1 < 0) || (i1 >= m)){
        continue;
      }
      int j1 = j0 - joffset;
      if ((j1 < 0) || (j1 >= n)){
        continue;
      }

      // Current cost to go from neighbor node to the goal
      int ij1 = i1 + m * j1;
      int ind1 = ij1 + nMatrixNodes * a1;
      // fprintf(stderr, "Checking ind1=%d / %lu\n", ind1, nMatrixNodes * nNeighbors);
      double c2g = cost_to_go[ind1];

#ifdef USE_RAYTRACE_NEIGHBOR_COSTS
      double cost = costmap_of_self;
      // Raytracing cost
      // Number of cells towards our neighbor
      int koffset = floor(dist_nbr);
      for (int k = 1; k <= koffset; k++) {
        int ik = i0 - (k*ioffset)/koffset;
        int jk = j0 - (k*joffset)/koffset;
        int ind = m*jk+ik;
        // int ind = ij0 + (k * (m * joffset + ioffset)) / koffset;
        // fprintf(stderr, "ij0: %d | ind: %d | nMatrixNodes %lu\n", ij0, ind, nMatrixNodes);
        cost += costmap[ind];
      }
      // cost /= koffset;
      // cost /= dist_nbr;
#else
      double cost = 0.5 * (costmap_of_self + costmap[ij1]) * (dist_nbr);
#endif
      // fprintf(stderr, "cost=%lf\n", cost);

      // New node cost to go
      // double turn_cost_factor = (ashift == 0) ? 1 : (TURN_COST * abs(ashift));
      double turn_cost_factor = costs_local[i_nbr];
      double c1 = c0 + cost * turn_cost_factor;
      // fprintf(stderr, "turn_cost_factor=%lf\n", turn_cost_factor);
      // double c1 = c0 + cost;

      if (c1 < c2g) {
        // Heuristic cost:
        double h1 = p_start ? sqrt(pow(jStart - j1, 2) + pow(iStart - i1, 2)) : 0;
        // Estimated total cost:
        double f1 = c1 + h1;
        if (!isinf(c2g)) {
          Q.erase(Q.find(CostNodePair(c2g + h1, ind1)));
        }
        // Update the map cost
        cost_to_go[ind1] = c1;
        Q.insert(CostNodePair(f1, ind1));
      }
    }
  }

  return 0;
}

/*
  [cost_to_go, next_index] = dijkstra_graph(A, goal_index)
  where connected edge costs i->j are given as positive entries
  in the sparse adjacency matrix A(i,j).

  dist is the cost to go to the goal node
  next_index containts the next index to traverse


typedef pair<double,int> di;  // (cost, from node)
typedef vector<di> vdi; // edge list for a single to node
typedef vector<vdi> vvdi; // vector of edge lists to all nodes

static int lua_dijkstra_graph(lua_State *L) {
  return 0;
}
*/
