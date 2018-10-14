#ifndef PATH_PLANNER
#define PATH_PLANNER

using namespace std;

const double LANE_WIDTH = 4.0; 
const double TIME_INTERVAL = 0.02;
const double SAFE_DISTANCE_AHEAD = 30;
const double SAFE_DISTANCE_BEHIND = 15;
const double MAX_SPEED = 22.35;

template <typename T>
int PathPlanner::ChooseLane(double d, double s, double & ref_vel, int prev_path_size, const T& sensor_fusion)
{
    int lane = detectLane(d);
    bool left_clear = (lane > 0);
    bool right_clear = (lane < 3);
    bool car_ahead = false;

    for (size_t i = 0; i != sensor_fusion.size(), ++i)
    {
        // predict the future position for each of the other vehicles
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double speed = sqrt(vx*vx + vy*vy);

        double car_s = sensor_fusion[i][5];
        car_s = prev_path_size * TIME_INTERVAL * speed;

        double car_d = sensor_fusion[i][6];
        car_lane = detectLane(car_d);

        if (car_s >= s && car_s - s <= SAFE_DISTANCE_AHEAD)
        {
            if (car_lane == lane)
            {
                car_ahead = true;
            }
            else if (car_lane - lane == 1)
            {
                right_clear = false;
            }
            else if (car_lane - lane == -1)
            {
                left_clear = false;
            }
        }
        else if (car_s < s && s - car_s <= SAFE_DISTANCE_BEHIND)
        {
            if (car_lane - lane == 1)
            {
                right_clear = false;
            }
            else if (car_lane - lane == -1)
            {
                left_clear = false;
            }
        }
    }

    if (car_ahead)
    {
        ref_vel -= 0.02;

        if (left_clear)
        {
            return lane-1;
        }
        else if (right_clear)
        {
            return lane + 1;
        }

    }
    return lane;

}

int PathPlanner::detectLane(double d)
{
    return floor(d / LANE_WIDTH);
}

#endif
