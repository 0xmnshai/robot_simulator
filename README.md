# Robot Simulator Package

A ROS 2 robot simulator package that combines SDL graphics rendering with ROS integration, featuring ultrasonic sensor simulation and robot navigation capabilities.

## Features

- **3D Graphics Rendering**: SDL-based visualization of the robot and environment
- **ROS 2 Integration**: Full ROS 2 integration for robot control and sensing
- **Ultrasonic Sensor Simulation**: Simulated ultrasonic sensors for obstacle detection
- **Robot Control**: Interface for controlling robot movement and navigation

## Screenshots

### SDL Graphics Window
![SDL Window](https://raw.githubusercontent.com/0xmnshai/robot_simulator/main/screenshots/sdl_window.png)

### RViz Map Visualization
![RViz Map](https://raw.githubusercontent.com/0xmnshai/robot_simulator/main/screenshots/rviz_map.png)

## Project Structure

```
robot_simulator_pkg/
├── include/
│   └── robot_simulator_pkg/
│       ├── graphics.hpp          # Graphics rendering interface
│       ├── robot.hpp             # Robot model and physics
│       ├── ros_interface.hpp      # ROS 2 communication
│       └── ultrasonic.hpp         # Ultrasonic sensor simulation
├── src/
│   ├── graphics.cpp              # Graphics implementation
│   ├── robot.cpp                 # Robot implementation
│   ├── ros_interface.cpp          # ROS interface implementation
│   ├── ultrasonic.cpp            # Ultrasonic sensor implementation
│   └── main.cpp                  # Main entry point
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

- ROS 2 (Humble or later)
- C++ 14 or later
- SDL2 (for graphics rendering)
- CMake 3.14 or later

## Installation

1. **Clone the repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> robot_simulator_pkg
   ```

2. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_simulator_pkg
   ```

4. **Source the workspace**:
   ```bash
   source install/local_setup.bash
   ```

## Usage

### Running the Robot Simulator

Launch the robot simulator:
```bash
ros2 run robot_simulator_pkg robot_simulator
```

### With RViz Visualization

To visualize the robot in RViz:
```bash
ros2 launch robot_simulator_pkg robot_simulator.launch.py
```

## Components

### Robot
- Handles robot kinematics and physics
- Manages position and orientation
- Processes control commands

### Graphics
- Renders the simulation environment using SDL2
- Displays robot and sensor data
- Provides real-time visualization

### ROS Interface
- Publishes sensor data (ultrasonic readings, odometry)
- Subscribes to control commands
- Integrates with ROS 2 ecosystem

### Ultrasonic Sensor
- Simulates multiple ultrasonic sensors
- Ray-casting based distance measurement
- Real-time obstacle detection

## Building from Source

```bash
mkdir -p build
cd build
cmake ..
make
```

## License

[Add your license here]

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For issues or questions, please open an issue on the GitHub repository.
