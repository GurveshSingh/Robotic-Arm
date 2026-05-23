import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2  

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. Setup YOLO (Smallest model for speed)
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()

        # 2. Coordinates: Mapping pixels to your 3-DOF Arm's reach
        # x_min/max = how far forward/back (meters)
        # y_min/max = how far left/right (meters)
        self.x_range = [0.15, 0.45] 
        self.y_range = [-0.3, 0.3]
        self.fixed_z = 0.2  # The arm will hover 20cm above the table

        # 3. ROS Publishers and Subscribers
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        self.get_logger().info("YOLO Node Started. Looking for bottles...")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape

        # Run Detection
        results = self.model(frame, verbose=False)
        
        for r in results:
            for box in r.boxes:
                # We only care about 'bottle' (Class 39 in COCO dataset)
                if self.model.names[int(box.cls)] == 'bottle':
                    # Get center of the box in pixels
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

                    # --- THE MATH: Map Pixels to Meters ---
                    # Normalizing pixels (0 to 1)
                    norm_x = cx / w 
                    norm_y = cy / h

                    # Mapping to Robot Space
                    # Note: Image X usually maps to Robot Y (lateral)
                    # Note: Image Y usually maps to Robot X (depth)
                    robot_y = self.y_range[1] - (norm_x * (self.y_range[1] - self.y_range[0]))
                    robot_x = self.x_range[0] + (norm_y * (self.x_range[1] - self.x_range[0]))

                    self.publish_target(robot_x, robot_y)

                    # Draw for you to see
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    break 

        cv2.imshow("YOLO Monitor", frame)
        cv2.waitKey(1)

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = "world" # Your robot's base frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = self.fixed_z
        msg.pose.orientation.w = 1.0 # Keep gripper facing forward
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(YoloNode())
    rclpy.shutdown()