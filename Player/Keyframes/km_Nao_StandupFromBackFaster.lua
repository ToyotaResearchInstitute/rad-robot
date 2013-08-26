local mot = {};
mot.servos = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
mot.keyframes = {
  {
    angles = {0.000, 0.175, 1.571, 0.262, -1.571, 0.000, 0.000, 0.000, -0.349, 0.000, 0.000, 0.000, 0.000, 0.000, -0.349, 0.000, 0.000, 0.000, 1.571, -0.262, 1.571, 0.000, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.400;
  },
  {
    angles = {0.000, 0.000, 0.000, 1.571, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.524, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -1.571, 0.000, 0.000, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.400;
  },
  {
    angles = {0.000, 0.000, 2.094, 0.803, 0.157, 0.000, 0.000, 0.000, -0.175, 1.676, 0.244, 0.000, -0.524, 0.000, -0.175, 1.676, 0.244, 0.000, 2.094, -0.803, -0.157, 0.000, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.400;
  },
  {
    angles = {0.000, -0.785, 2.094, 0.471, 0.087, -1.658, 0.000, 0.000, -0.175, 1.676, 0.244, 0.000, -0.524, 0.000, -0.175, 1.676, 0.244, 0.000, 2.094, -0.471, -0.087, 1.658, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.470;
  },
  {
    angles = {0.000, 0.000, 2.094, 0.367, 0.087, -0.698, -0.663, 0.000, -1.571, 1.676, 0.244, 0.000, -0.524, 0.000, -1.571, 1.676, 0.244, 0.000, 2.094, -0.367, -0.087, 0.698, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.470;
  },
  {
    angles = {0.000, 0.349, 2.094, 0.000, 0.087, 0.000, -0.663, 0.541, -1.571, 1.676, 0.785, 0.000, -0.524, -0.541, -1.571, 1.676, 0.785, 0.000, 2.094, 0.000, -0.087, 0.000, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.400;
  },
  {
    angles = {-0.009, 0.332, 0.698, 1.047, 0.087, -0.489, -0.489, 0.157, -0.855, 2.199, -0.559, -0.384, -0.524, -0.559, -1.518, 1.222, 0.785, 0.009, 2.094, -0.576, -0.080, 0.070, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.400;
  },
  {
    angles = {-0.010, 0.332, 0.733, 0.506, 0.087, -0.820, -0.873, -0.297, 0.384, 1.780, -1.222, -0.105, -0.524, -0.559, -1.571, 1.065, 0.698, -0.129, 1.780, -0.262, -0.087, 0.052, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = .575;
  },
  {
    angles = {-0.009, 0.384, 0.733, 0.506, 0.087, -0.803, -0.401, 0.192, -0.855, 2.199, -1.222, 0.122, -0.873, -0.297, -0.890, 0.873, 0.401, 0.681, 0.890, -0.873, 0.000, 0.454, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = .575;
  },
  {
    angles = {-0.009, 0.384, 0.733, 0.506, 0.087, -0.803, -0.401, 0.367, -0.838, 2.199, -1.222, 0.087, -0.401, -0.017, -0.890, 1.763, -0.471, 0.279, 0.890, -0.681, 0.000, 0.559, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = .575;
  },
  {
    angles = {-0.009, 0.384, 1.710, 0.209, -1.257, -1.134, 0.000, 0.000, -0.873, 2.094, -1.222, 0.000, -0.401, 0.000, -0.873, 2.094, -1.222, 0.000, 1.710, -0.209, 1.257, 1.134, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = 0.470;
  },
  {
    angles = {0.000, -0.436, 2.094, 0.349, -1.396, -1.396, 0.000, 0.017, -0.723, 1.490, -0.767, -0.017, -0.000, -0.017, -0.723, 1.490, -0.767, 0.017, 2.094, -0.349, 1.396, 1.396, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = .675;
  },
  {
    angles = {-0.066, -0.678, 1.468, 0.229, -1.273, -0.305, 0.000, -0.003, -0.396, 0.946, -0.548, 0.002, 0.000, 0.026, -0.397, 0.945, -0.548, -0.025, 1.494, -0.253, 1.216, 0.502, },
    stiffnesses = {0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, 0.800, },
    duration = .575;
  },
};

return mot;
