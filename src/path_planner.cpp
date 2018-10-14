#ifndef PATH_PLANNER
#define PATH_PLANNER

#include "path_planner.h"

using namespace std;

const double LANE_WIDTH = 4.0; 
const double TIME_INTERVAL = 0.02;
const double SAFE_DISTANCE_AHEAD = 30;
const double SAFE_DISTANCE_BEHIND = 15;
const double MAX_SPEED = 22.35;

//int PathPlanner::detectLane(double d)
//{
//    return floor(d / LANE_WIDTH);
//}
