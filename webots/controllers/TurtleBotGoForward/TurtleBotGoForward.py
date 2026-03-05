"""TurtleBotGoForward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from robot_comm import RobotPublisher, RobotSubscriber
import math

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

#Drive Setup
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Position sensors (encoders)
leftPosition = robot.getDevice("left wheel sensor")
rightPosition = robot.getDevice("right wheel sensor")
leftPosition.enable(TIME_STEP)
rightPosition.enable(TIME_STEP)

# IMU Sensors
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)
accelerometer = robot.getDevice("accelerometer")
accelerometer.enable(TIME_STEP)
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

# Lidar Setup
lidar = robot.getLidar("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# ZMQ Publisher Setup (send data to C++)
publisher = RobotPublisher()
# ZMQ Subscriber Setup (receive responses fadrom C++)
subscriber = RobotSubscriber()
subscriber.subscribe("response")  # Subscribe to response topic

step_count = 0

while robot.step(TIME_STEP) != -1:
    
    key = keyboard.getKey()

    linear_speed = 0.0
    angular_speed = 0.0

    if key == ord('W') or key == ord('w'):
        linear_speed = MAX_SPEED
    elif key == ord('S') or key == ord('s'):
        linear_speed = -MAX_SPEED 

    if key == ord('A') or key == ord('a'):
        angular_speed = MAX_SPEED 
    elif key == ord('D') or key == ord('d'):
        angular_speed = -MAX_SPEED 

    leftMotor.setVelocity(linear_speed - angular_speed)
    rightMotor.setVelocity(linear_speed + angular_speed)

    # Collect sensor data
    step_count += 1
    
    # Header data
    header = {
        "timestamp": robot.getTime(),
        "step_count": step_count
    }
    
    # Odometry data
    compass_values = compass.getValues()
    compass_heading = math.atan2(compass_values[0], compass_values[1])
    
    odometry = {
        "left_encoder": leftPosition.getValue(),
        "right_encoder": rightPosition.getValue(),
        "imu_gyro_z": gyro.getValues()[2],  # Z-axis angular velocity
        "imu_accel": list(accelerometer.getValues()),  # [x, y, z]
        "compass_heading": compass_heading  # Absolute heading in radians
    }
    
    # Lidar data
    lidar_ranges = lidar.getRangeImage()
    lidar_fov = lidar.getFov()  # actual FOV in radians 
    lidar_data = {
        "count": len(lidar_ranges),
        "angle_min": -(lidar_fov / 2.0),
        "angle_max":  (lidar_fov / 2.0),
        "range_min": lidar.getMinRange(),
        "range_max": lidar.getMaxRange(),
        "ranges": [float('inf') if r == float('inf') else r for r in lidar_ranges]
    }
    
    # Send robot state every X timesteps
    if step_count % 2 == 0:
        publisher.publish_robot_state(header, odometry, lidar_data)
        print(f"[Python] Sent robot state at step {step_count}")
    
    # Check for responses from C++ (non-blocking)
    response = subscriber.receive_message(timeout_ms=0)
    if response:
        topic, msg = response
        print(f"[Python] Received from C++ on '{topic}': {msg}")
    



    
