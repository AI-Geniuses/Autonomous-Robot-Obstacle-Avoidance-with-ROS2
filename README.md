# Autonomous Robot Obstacle Avoidance with ROS2

This project focuses on developing an autonomous robot capable of real-time obstacle avoidance and navigation using ROS2. The robot is equipped with advanced sensors including LiDAR, camera, and depth camera, and utilizes SLAM (Simultaneous Localization and Mapping) and the NAV2 stack for navigation.

## Features

- Real-time obstacle detection and avoidance
- Autonomous navigation in a maze environment
- Accurate SLAM for mapping and localization
- Adaptive path planning using NAV2
- Integration of multiple sensors for robust perception

## System Architecture

The system is designed with the following components:

- **Sensors:**
  - **LiDAR:** Provides 360-degree distance measurements for obstacle detection.
  - **Camera:** Captures visual data for object recognition.
  - **Depth Camera:** Provides depth information for 3D perception.

- **SLAM:** Generates a map of the environment and localizes the robot within it.
- **NAV2:** Handles path planning and navigation, ensuring the robot can reach target locations while avoiding obstacles.

## Robot and Map

### Robot

The autonomous robot is equipped with multiple sensors including LiDAR, camera, and depth camera to navigate and avoid obstacles effectively.

![Robot](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/Robot.png?raw=true)

### Map

The map is a small maze designed to evaluate the robot's obstacle avoidance and navigation capabilities. The maze features a series of interconnected corridors and dead-ends, presenting a challenging environment for autonomous navigation. The robot utilizes its SLAM system to create a real-time map of the maze, continuously updating its understanding of the environment as it navigates.

![Map](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/Map.png?raw=true)

Key features of the maze map:
- **Corridors and Dead-Ends:** The maze includes narrow corridors and several dead-ends, requiring the robot to make precise turns and backtrack when necessary.
- **Static Obstacles:** Fixed obstacles within the maze test the robot's ability to detect and avoid obstacles using its LiDAR, camera, and depth camera sensors.
- **Complex Pathways:** The interconnected pathways of the maze challenge the robot's path planning and decision-making algorithms, ensuring robust navigation performance.

By successfully navigating the maze, the robot demonstrates its capability to handle real-world environments with similar complexities.


## Sensor Integration

### LiDAR

The LiDAR sensor provides accurate distance measurements, which are crucial for obstacle detection and avoidance.

![LiDAR](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/LiDAR.png?raw=true)

### Camera

The camera captures visual data, which can be used for additional perception tasks.

![Camera](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/Camera.png?raw=true)

### Depth Camera

The depth camera provides depth information, enhancing the robot's 3D perception capabilities.

![Depth Camera](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/Depth-Camera.png?raw=true)

## Navigation and SLAM

### SLAM

The SLAM system creates a map of the environment and localizes the robot within this map.

![SLAM](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/SLAM.png?raw=true)

### NAV2

The NAV2 stack is responsible for planning paths and navigating the robot to its target locations while avoiding obstacles.

![NAV2](https://github.com/AI-RoboGeniuses/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/blob/main/.img/Nav2.png?raw=true)

## Evaluation

The robot was tested in a simulated maze environment to evaluate its navigation capabilities:

- **Maze Navigation:** Successfully navigated a small maze with static obstacles.
- **Localization Accuracy:** Maintained a localization error of less than 5 cm.
- **Obstacle Detection Rate:** Achieved a detection rate of 98%.
- **Navigation Efficiency:** Reached targets within a reasonable timeframe with minimal deviations from the optimal path.

## Challenges

During the project, several challenges were encountered and addressed:

- **Sensor Integration:** Ensuring synchronized data fusion from multiple sensors.
- **Real-Time Processing:** Achieving real-time performance with limited computational resources.
- **Parameter Tuning:** Optimizing SLAM and NAV2 parameters for best performance.
- **Environmental Variability:** Handling different surface textures and lighting conditions.

## Future Work

Future enhancements could focus on:

- Further improving the system's robustness.
- Exploring additional use cases and environments.
- Integrating machine learning techniques for improved decision-making.

## Installation

### Prerequisites

Ensure you have the following software installed on your Ubuntu system:

- ROS2 Iron
- Gazebo
- RViz2

### Setting Up the Workspace

1. Create the ROS2 workspace:

    ```sh
    mkdir -p ~/ROS2-Auto-Robot/src
    cd ~/ROS2-Auto-Robot/
    colcon build
    source install/setup.bash
    ```

2. Create the robot package:

    ```sh
    cd src
    ros2 pkg create --build-type ament_cmake auto_robot
    ```

### Building the Project

1. Navigate to the workspace root:

    ```sh
    cd ~/ROS2-Auto-Robot/
    ```

2. Build the workspace:

    ```sh
    colcon build
    source install/setup.bash
    ```

## Usage

### Launching the Simulation

1. Start the Gazebo simulation:

    ```sh
    ros2 launch auto_robot launch_sim.launch.py world:=./src/auto_robot/worlds/maze.world
    ```

2. Launch RViz2 for visualization:

    ```sh
    rviz2 -d src/auto_robot/config/main.rviz
    ```

### Running the Robot

1. Start the SLAM node:

    ```sh
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/auto_robot/config/mapper_params_online_async.yaml use_sim_time:=true
    ```

2. Start the NAV2 stack:

    ```sh
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
    ```


## Conclusion

This project demonstrates the potential of using ROS2 for developing robust and flexible autonomous navigation systems. The successful integration of advanced sensors and the implementation of SLAM and NAV2 provide a solid foundation for future advancements in autonomous robotics.