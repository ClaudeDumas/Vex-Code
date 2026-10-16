#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <vector>

#include "v5.h"
#include "v5_vcs.h"
#include "robot-config.h"
#include "drive.h"
#include "util.h"
#include "Autonomous/PID.h"
#include "Autonomous/odom.h"
#include "Pathfinding/pathfinding.h"
#include "Pathfinding/navmesh.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)