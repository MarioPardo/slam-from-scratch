"""TurtleBotGoForward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from robot_comm import RobotPublisher, RobotSubscriber
import math
import random

# Encoder noise 
ENCODER_NOISE_STD = 0.2  # radians

#Wheel slip 
SLIP_PROBABILITY     = 0.1   # chance per step  (0 = off, 0.01 = ~1% per step)
SLIP_MAGNITUDE_MEAN  = 0.2     # mean extra encoder rotation per slip event (rad)
SLIP_MAGNITUDE_STD   = 0.2    # std of slip magnitude (rad)

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

# Optional ground-truth position sensor (if present in robot model)
gps = None
try:
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)
    print("[Python] GPS enabled for ground-truth position logging")
except Exception:
    gps = None
    print("[Python] GPS device not found; falling back to Robot node position if available")

# Fallback: direct node position (may not be available for non-supervisor controllers)
robot_node = None
if gps is None:
    try:
        robot_node = robot.getSelf()
    except Exception:
        robot_node = None

POSITION_PRINT_EVERY_STEPS = 10

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
# Accumulated slip offsets — grow permanently each time a slip fires
left_slip_offset  = 0.0
right_slip_offset = 0.0

while robot.step(TIME_STEP) != -1:
    
    key = keyboard.getKey()

    linear_speed = 0.0
    angular_speed = 0.0

    if key == ord('W') or key == ord('w'):
        linear_speed = MAX_SPEED
    elif key == ord('S') or key == ord('s'):
        linear_speed = -MAX_SPEED 

    if key == ord('A') or key == ord('a'):
        angular_speed = MAX_SPEED*0.5 
    elif key == ord('D') or key == ord('d'):
        angular_speed = -MAX_SPEED*0.5 

    leftMotor.setVelocity(linear_speed - angular_speed)
    rightMotor.setVelocity(linear_speed + angular_speed)

    # Collect sensor data
    step_count += 1
    
    # Header data
    header = {
        "timestamp": robot.getTime(),
        "step_count": step_count
    }

    # Ground truth from GPS + compass 
    gt_x, gt_y = 0.0, 0.0
    if gps is not None:
        gps_vals = gps.getValues()
        gt_x, gt_y = gps_vals[0], gps_vals[1] 
    elif robot_node is not None:
        pos = robot_node.getPosition()
        gt_x, gt_y = pos[0], pos[1]

    compass_values = compass.getValues()
    gt_heading = math.atan2(compass_values[0], compass_values[1])

    ground_truth = {
        "gt_x": gt_x,
        "gt_y": gt_y,
        "heading": gt_heading
    }

    # Wheel slip: randomly fire a slip event and grow the permanent offset
    if random.random() < SLIP_PROBABILITY:
        left_slip_offset  += abs(random.gauss(SLIP_MAGNITUDE_MEAN, SLIP_MAGNITUDE_STD))
        right_slip_offset += abs(random.gauss(SLIP_MAGNITUDE_MEAN, SLIP_MAGNITUDE_STD))
        
    # Odometry data
    compass_heading = gt_heading

    odometry = {
        "left_encoder":  leftPosition.getValue()  + left_slip_offset  + random.gauss(0, ENCODER_NOISE_STD),
        "right_encoder": rightPosition.getValue() + right_slip_offset + random.gauss(0, ENCODER_NOISE_STD),
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
    if step_count % 1 == 0:
        publisher.publish_robot_state(header, odometry, lidar_data, ground_truth)
        print(f"[Python] Sent robot state at step {step_count}")
    
    # Check for responses from C++ (non-blocking)
    response = subscriber.receive_message(timeout_ms=0)
    if response:
        topic, msg = response
        print(f"[Python] Received from C++ on '{topic}': {msg}")
    



    
