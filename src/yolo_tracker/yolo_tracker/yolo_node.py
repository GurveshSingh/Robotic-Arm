import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.box_padding = 20  # pixels to expand bbox on each side

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos)

        self.error_pub = self.create_publisher(Point, '/visual_error', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated', 10)

        self.get_logger().info("YOLO Node Started.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape
        cx_img, cy_img = w // 2, h // 2

        results = self.model(frame, verbose=False)

        error_msg = Point()
        error_msg.z = 0.0

        for r in results:
            for box in r.boxes:
                if self.model.names[int(box.cls)] == 'bottle':
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    # Expand bounding box
                    p = self.box_padding
                    x1e = max(0,   x1 - p)
                    y1e = max(0,   y1 - p)
                    x2e = min(w,   x2 + p)
                    y2e = min(h,   y2 + p)

                    # Error based on original box center
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    err_x = (cx - cx_img) / (w / 2)
                    err_y = (cy - cy_img) / (h / 2)

                    # Centered = camera crosshair inside EXPANDED box
                    inside = (x1e < cx_img < x2e) and (y1e < cy_img < y2e)

                    error_msg.z = 1.0
                    if inside:
                        error_msg.x = 0.0
                        error_msg.y = 0.0
                        self.get_logger().info("Centered — crosshair inside box!")
                    else:
                        error_msg.x = err_x
                        error_msg.y = err_y

                    # Draw original box (green) and expanded box (yellow)
                    cv2.rectangle(frame,
                        (int(x1),  int(y1)),
                        (int(x2),  int(y2)),
                        (0, 255, 0), 2)
                    cv2.rectangle(frame,
                        (int(x1e), int(y1e)),
                        (int(x2e), int(y2e)),
                        (0, 255, 255), 1)

                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.line(frame, (cx_img, cy_img), (int(cx), int(cy)), (255, 0, 0), 2)
                    cv2.putText(frame, f'err x:{err_x:.2f} y:{err_y:.2f}',
                                (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    break

            if error_msg.z == 1.0:
                break

        self.error_pub.publish(error_msg)

        # Crosshair
        cv2.line(frame, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (255, 255, 255), 1)
        cv2.line(frame, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (255, 255, 255), 1)

        if error_msg.z == 1.0 and error_msg.x == 0.0:
            status, color = "CENTERED", (0, 255, 0)
        elif error_msg.z == 1.0:
            status, color = "DETECTED", (0, 165, 255)
        else:
            status, color = "searching...", (0, 0, 255)

        cv2.putText(frame, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        cv2.imshow("YOLO Monitor", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()