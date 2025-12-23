#include <vector>
#include <utility>
#include <SDL2/SDL.h>

class Robot
{
public:
    Robot(double x, double y, double width);

    void avoidObstacles(const std::vector<std::pair<int, int>> &cloud);
    void handleKeyboard(const uint8_t* keys_pressed);
    void forward();
    void backward();
    void kinematics(double dt);

    double x, y, heading;
    double v_r, v_l;

private:
    double width;
    double max_speed, min_speed;
    double min_obstacle_dist;
    int countdown;
};