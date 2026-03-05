# ObstacleChecker

A ROS 2 node that ensures the robot stops when any LIDAR reading is less than or equal to 0.5 m.

## Solution Description

The `ObstacleChecker` node subscribes to the LIDAR scan topic (`/scan`) and monitors all distance readings. When any reading is less than or equal to the configured stop distance threshold (0.5 m), the node intercepts incoming velocity commands (`/diff_drive_controller/cmd_vel`) and publishes a stop command (zero linear velocity) to prevent collision. To prevent movement where there is no velocity commands also stop command publishing could be executed on scan callback.

## Video
[Video link](https://icedrive.net/s/CBb32g7GGataQg8PvQNWaSkakX7B)

## Running Locally with Docker Container without node compilation and package creation (preview purpose)

### Steps

1. **Start the container:**

   ```bash
   ./docker_run.sh
   ```

   This script will either start an existing container named `ros_gz_gui` or create a new one using the `exentr0/ros2-test:jazzy` image.

   **scripts** folder will be mounted to running container by path 
   ~/ros2_ws/src/my_diff_robot/scripts/safety

2. **Run the obstacle checker node and teleop node all in once:**

   ```bash
   cd ~/ros2_ws/src/my_diff_robot/scripts/safety
   bash ./safety_teleop.sh
   ```
3. **Start simulation**

   Open separate terminal vindow and attach to container shell
   ```bash
   bash ./container_shell.sh
   ```

   Start the simulation
   ```bash
   ros2 launch my_diff_robot robot.launch.py
   ```

## Design Description

### Key Design Choices

1. **Message Types:**
   - Input: `sensor_msgs.msg.LaserScan` for LIDAR data
   - Input/Output: `geometry_msgs.msg.TwistStamped` for velocity commands

2. **Topics:**
   - LIDAR topic: `/scan`
   - Command velocity: `/diff_drive_controller/cmd_vel`

3. **Obstacle Detection Logic:**
   - The node checks each valid LIDAR reading (ignoring `inf` values and those outside sensor range)
   - Stop distance threshold: 0.5 meters
   - Only considers readings within specified angle range of the robot's forward direction

4. **Stop Behavior:**
   - When an obstacle is detected AND the robot is moving (linear velocity > 0), the node publishes a stop command with zero linear velocity
   - Angular velocity is also set to zero to bring the robot to a complete stop
   - The node acts as a safety interceptor by republishing on the same cmd_vel topic

### Assumptions

- The LIDAR sensor publishes valid readings in the `/scan` topic
- The robot's controller subscribes to `/diff_drive_controller/cmd_vel` for velocity commands
- The robot's forward direction corresponds to 0° in the LIDAR scan 
- The node runs alongside other navigation nodes and has access to the same ROS 2 network

### File Structure

```
robo_test/
├── README.md                    # This file
├── docker_run.sh               # Script to run Docker container
├── container_shell.sh          # Container shell script
└── scripts/
    └── obstacle_checker.py     # Main ROS 2 node implementation
    └── safety_teleop.sh        # Run two main controll nodes

```
