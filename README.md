# Assetto Corsa ROS

This repository integrates Assetto Corsa with ROS 2. To set it up inside an existing ROS 2 workspace, follow these steps:

1. Navigate to the `src` directory of your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   ```
2. Create a folder to hold the simulation packages and enter it:
   ```bash
   mkdir -p simulation
   cd simulation
   ```
3. Clone this repository into the `simulation` directory:
   ```bash
   git clone git@github.com:cast-racing/assetto_corsa_ros.git
   cd assetto_corsa_ros
   ```
4. Run the setup script to install dependencies and configure the environment:
   ```bash
   ./assetto_corsa_ros_setup.sh
   ```

Once the setup script completes, you can build and source your workspace as usual.
