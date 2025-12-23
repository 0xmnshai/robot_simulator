
#include <thread>
#include <iostream>
#include <cstdlib>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "robot_simulator_pkg/robot.hpp"
#include "robot_simulator_pkg/graphics.hpp"
#include "robot_simulator_pkg/ultrasonic.hpp"
#include "robot_simulator_pkg/ros_interface.hpp"

int main(int argc, char **argv)
{
    std::cout << "Initializing SDL..." << std::endl;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0)
    {
        std::cerr << "Failed to init SDL: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    if (IMG_Init(IMG_INIT_PNG | IMG_INIT_JPG) == 0)
    {
        std::cerr << "Failed to init SDL_image: " << IMG_GetError() << std::endl;
        SDL_Quit();
        return EXIT_FAILURE;
    }
    std::cout << "SDL initialized." << std::endl;

    int dimensions_width = 1024;
    int dimensions_height = 720;

    SDL_Window *window = SDL_CreateWindow("ROS2 Robot Simulator",
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          dimensions_width, dimensions_height,
                                          SDL_WINDOW_SHOWN);

    if (!window)
    {
        std::cerr << "Failed to create SDL window: " << SDL_GetError() << std::endl;
        IMG_Quit();
        SDL_Quit();
        return EXIT_FAILURE;
    }
    std::cout << "Window created." << std::endl;

    SDL_Surface *screen = SDL_GetWindowSurface(window);
    if (!screen)
    {
        std::cerr << "Failed to get window surface: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        IMG_Quit();
        SDL_Quit();
        return EXIT_FAILURE;
    }

    std::cout << "Initializing ROS..." << std::endl;
    rclcpp::init(argc, argv);
    std::shared_ptr<ROSInterface> ros;
    try
    {
        ros = std::make_shared<ROSInterface>();
        std::cout << "ROS initialized." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "ROS initialization error: " << e.what() << std::endl;
        SDL_DestroyWindow(window);
        IMG_Quit();
        SDL_Quit();
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    std::cout << "Initializing graphics..." << std::endl;
    Graphics gfx(dimensions_width, dimensions_height, "/Users/lazycodebaker/Documents/Dev/C++/robot_simulator/robot_simulator_pkg/robot.png", "/Users/lazycodebaker/Documents/Dev/C++/robot_simulator/robot_simulator_pkg/floor_map.png");
    std::cout << "Graphics initialized." << std::endl;

    std::cout << "Initializing robot and sensor..." << std::endl;
    Robot robot(150, 200, 40);
    Ultrasonic sensor(250, M_PI / 4.5, screen);
    std::cout << "Robot and sensor initialized." << std::endl;

    bool running = true;
    bool keyboard_active = false;
    Uint32 last_ticks = SDL_GetTicks();
    const float FPS = 60.0f;
    const float frame_time = 1000.0f / FPS;
    int frame_count = 0;

    std::cout << "Starting main loop..." << std::endl;

    while (running && rclcpp::ok())
    {
        Uint32 now_ticks = SDL_GetTicks();
        double dt = (now_ticks - last_ticks) / 1000.0;
        last_ticks = now_ticks;

        // Clamp dt
        if (dt > 0.05)
            dt = 0.05;
        if (dt < 0.001)
            dt = 0.001;

        // Handle events
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }

        // Get keyboard state
        const uint8_t *keys = SDL_GetKeyboardState(nullptr);
        if (!keys)
        {
            std::cerr << "Failed to get keyboard state" << std::endl;
            continue;
        }

        keyboard_active = keys[SDL_SCANCODE_UP] || keys[SDL_SCANCODE_DOWN] ||
                          keys[SDL_SCANCODE_LEFT] || keys[SDL_SCANCODE_RIGHT];

        // Get sensor data
        auto hits = sensor.sense(robot.x, robot.y, robot.heading);

        // Handle robot movement
        if (keyboard_active)
        {
            robot.handleKeyboard(keys);
        }
        else
        {
            std::vector<std::pair<int, int>> point_cloud;
            for (const auto &hit : hits)
            {
                point_cloud.emplace_back(std::get<0>(hit), std::get<1>(hit));
            }
            robot.avoidObstacles(point_cloud);
        }

        // Update robot kinematics
        robot.kinematics(dt);

        // Publish to ROS
        ros->publishLaserScan(hits, robot.heading);
        ros->updateMap(robot.x, robot.y, hits);
        ros->broadcastTF(robot.x, robot.y, robot.heading);

        // Clear screen
        SDL_FillRect(screen, nullptr, SDL_MapRGB(screen->format, 255, 255, 255));

        // Draw graphics
        gfx.drawMap(screen);
        gfx.drawRobot(screen, robot.x, robot.y, robot.heading);

        std::vector<std::pair<int, int>> sensor_points;
        for (const auto &hit : hits)
        {
            sensor_points.emplace_back(std::get<0>(hit), std::get<1>(hit));
        }
        gfx.drawSensorData(screen, sensor_points, robot.x, robot.y);

        // Update display
        SDL_UpdateWindowSurface(window);

        // Frame rate limiting
        Uint32 frame_end = SDL_GetTicks();
        int delay = static_cast<int>(frame_time) - static_cast<int>(frame_end - now_ticks);
        if (delay > 0)
        {
            SDL_Delay(delay);
        }

        frame_count++;
        if (frame_count % 60 == 0)
        {
            std::cout << "Frame " << frame_count << std::endl;
        }
    }

    std::cout << "Shutting down..." << std::endl;
    rclcpp::shutdown();
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();

    return EXIT_SUCCESS;
}