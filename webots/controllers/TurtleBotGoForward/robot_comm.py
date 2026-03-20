"""ZMQ communication module for robot data publishing."""

import zmq
import json
import time


class RobotPublisher:
    """Publisher for sending robot sensor and state data via ZMQ."""
    
    def __init__(self, address="tcp://localhost:5555"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(address)
        print(f"[RobotPublisher] Bound to {address}")
        
        # Small delay to allow subscribers to connect
        time.sleep(0.1)
    
    def publish_message(self, topic, message_dict):

        # Add timestamp if not present
        if 'timestamp' not in message_dict:
            message_dict['timestamp'] = time.time()
        
        # Serialize to JSON
        message_json = json.dumps(message_dict)
        
        # Send as topic + space + message
        full_message = f"{topic} {message_json}"
        self.socket.send_string(full_message)
    
    def publish_hello(self, message="Hello from robot!"):
        self.publish_message("hello", {"message": message})
    
    def publish_data(self, data):
        """Publish sensor data to C++."""
        self.publish_message("data", {"value": data})
    
    def publish_robot_state(self, header, odometry, lidar, ground_truth=None):
        """Publish comprehensive robot state."""
        robot_state = {
            "header": header,
            "odometry": odometry,
            "lidar": lidar
        }
        if ground_truth is not None:
            robot_state["ground_truth"] = ground_truth
        self.publish_message("robot_state", robot_state)
    
    def close(self):
        """Clean up ZMQ resources."""
        self.socket.close()
        self.context.term()
        print("[RobotPublisher] Closed")


class RobotSubscriber:
    """Subscriber for receiving processed data from C++ via ZMQ."""
    
    def __init__(self, address="tcp://localhost:5556"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(address)
        print(f"[RobotSubscriber] Connected to {address}")
    
    def subscribe(self, topic=""):
        """Subscribe to a topic. Empty string subscribes to all topics."""
        self.socket.subscribe(topic)
        print(f"[RobotSubscriber] Subscribed to topic: '{topic}'")
    
    def receive_message(self, timeout_ms=0):
        """
        Receive a message with optional timeout.
        Returns (topic, message_dict) or None if timeout.
        """
        # Set receive timeout (0 = non-blocking)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        
        try:
            full_message = self.socket.recv_string()
            
            # Split into topic and message (format: "topic message_json")
            parts = full_message.split(' ', 1)
            if len(parts) != 2:
                print(f"[RobotSubscriber] Invalid message format: {full_message}")
                return None
            
            topic = parts[0]
            message_dict = json.loads(parts[1])
            
            return (topic, message_dict)
            
        except zmq.Again:
            # Timeout - no message available
            return None
        except Exception as e:
            print(f"[RobotSubscriber] Error receiving message: {e}")
            return None
    
    def close(self):
        """Clean up ZMQ resources."""
        self.socket.close()
        self.context.term()
        print("[RobotSubscriber] Closed")
