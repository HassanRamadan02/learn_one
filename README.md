# Autonomous Robot Car with Obstacle Avoidance and Waypoint Navigation

Welcome to the repository for my Autonomous Robot Car project! This robot is designed to navigate to a goal point while avoiding both static and dynamic obstacles. It can also follow a set of waypoints using the **Nav2** navigation stack in ROS 2. This project is perfect for anyone interested in robotics, autonomous navigation, and ROS 2.

---

## Features
- **Autonomous Navigation**: The robot can autonomously navigate to a specified goal point.
- **Obstacle Avoidance**: It can detect and avoid both static and dynamic obstacles using sensors and algorithms.
- **Waypoint Following**: The robot can follow a predefined set of waypoints.
- **Nav2 Integration**: Utilizes the **Nav2** package for robust navigation and path planning.
- **Customizable**: Easily extendable for additional features or integration with other ROS 2 packages.

---

## Prerequisites
Before using this project, ensure you have the following installed:
- **ROS 2 (Humble recommended)**
- **Nav2** package (`navigation2`)
- **Gazebo (classic)** or another simulation environment (optional but recommended for testing)
- **Rviz2** for visualization
- **Python 3** and **C++** (for custom scripts and nodes)

---

## Installation
1. Clone this repository into your ROS 2 workspace:
   ```bash
   git clone https://github.com/HassanRamadan02/learn_one.git
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Usage
1. **Launch the Simulation** (if using Gazebo):
   ```bash
   ros2 launch learn_one launch_sim.launch.py
   ```
2. **Start Navigation**:
   ```bash
   ros2 launch learn_one navigation.launch.py
   ```
3. **Set Goal Points or Waypoints**:
   - Use Rviz2 to set a goal point or load a waypoint file.
   - Alternatively, use a custom script to send waypoints programmatically.

4. **Monitor Robot Behavior**:
   - Observe the robot navigating to the goal while avoiding obstacles.
   - Check the terminal for logs and diagnostics.

---

## Configuration
- **Sensor Configuration**: Adjust sensor parameters (e.g., LiDAR, camera) in the `config/` directory.
- **Navigation Parameters**: Tune navigation behavior in the `config/` directory.

---

## Project Structure
```
learn_one/
├── config/              # Configuration files for sensors and navigation, behavior tree parameters
├── launch/              # Launch files for simulation and navigation
├── worlds/              # Map files for navigation
├── description/         # Describe robot and sensor URDFs
├── README.md            # This file
└── LICENSE              # License file
```

---

## Demo Videos
- [[Link to Demo Video 1](https://drive.google.com/file/d/1b1No2gyTRbfAnZGdnJxQGvkE4-uTvFYW/view?usp=drive_link)](#)
- [[Link to Demo Video 2](https://drive.google.com/file/d/1YPHSkR8ZWa7WKa8TatZfDhz-3KHgRu42/view?usp=sharing)]
- [[Link to Demo Video 3](https://drive.google.com/file/d/1nCtHUQhUwrBLbh_UoEdj6eof0bj4e-_i/view?usp=sharing)]

---

## Documentation
- [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
- [Nav2 Official Documentation](https://navigation.ros.org)

---

## Contributing
Contributions are welcome! If you'd like to contribute, please:
1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Submit a pull request.

---

## License
This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments
- Thanks to the **ROS 2** and **Nav2** communities for their incredible work.
- Special thanks to [](joshnewans) for their guidance.

---

## Contact
For questions or feedback, feel free to reach out:
- **Email**: hassan.ramadan1409@gmail.com
- **GitHub**: [HassanRamadan02](https://github.com/your-username)
- **LinkedIn**: [www.linkedin.com/in/eng-hassan-ramadan](#)

---

Happy coding and robot building! 🚀🤖
