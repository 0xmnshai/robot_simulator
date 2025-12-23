#include <cmath>

#include "robot_simulator_pkg/robot.hpp"

static double dist(double x1, double y1, double x2, double y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

Robot::Robot(double x_, double y_, double w)
    : x(x_), y(y_), heading(0.0), width(w),
      v_r(30), v_l(30), max_speed(80), min_speed(20),
      min_obstacle_dist(80), countdown(3) {}

void Robot::forward()
{
    v_r = max_speed;
    v_l = max_speed;
}

void Robot::backward()
{
    v_r = -min_speed;
    v_l = -min_speed * 0.3;
}

void Robot::handleKeyboard(const uint8_t *keys_pressed)
{
    if (keys_pressed[SDL_SCANCODE_UP])
    {
        // Move forward
        v_r = max_speed;
        v_l = max_speed;
    }
    else if (keys_pressed[SDL_SCANCODE_DOWN])
    {
        // Move backward
        v_r = -min_speed;
        v_l = -min_speed * 0.3;
    }
    else if (keys_pressed[SDL_SCANCODE_LEFT])
    {
        // Turn left
        v_r = max_speed;
        v_l = min_speed * 0.3;
    }
    else if (keys_pressed[SDL_SCANCODE_RIGHT])
    {
        // Turn right
        v_r = min_speed * 0.3;
        v_l = max_speed;
    }
    else
    {
        // No keyboard input, use default forward
        forward();
    }
}

void Robot::avoidObstacles(const std::vector<std::pair<int, int>> &cloud)
{
    if (cloud.empty())
    {
        forward();
        return;
    }

    double min_d = 1e9;
    for (auto &p : cloud)
    {
        min_d = std::min(min_d, dist(x, y, p.first, p.second));
    }

    if (min_d < min_obstacle_dist && countdown > 0)
    {
        countdown--;
        backward();
    }
    else
    {
        countdown = 5;
        forward();
    }
}

void Robot::kinematics(double dt)
{
    double v = (v_r + v_l) * 0.5;
    x += v * std::cos(heading) * dt;
    y -= v * std::sin(heading) * dt;
    heading += (v_r - v_l) / width * dt;
}
