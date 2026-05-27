#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math


class YoloIKNode(Node):
    def __init__(self):
        super().__init__('yolo_ik_node')

        # ── D455 RGB 1280×720 defaults (overridden by /camera_info if received) ──
        self.fx = 642.29
        self.fy = 641.56
        self.cx_cam = 652.59
        self.cy_cam = 360.33
        self.intrinsics_received = False

        # ── Current joint angles ──
        self.q = [0.0, 0.0, 0.0]   # [θ₀, θ₁, θ₂]
        self.got_joints = False

        # ── Latest detection (bbox centre in pixels, confidence flag) ──
        self.det_u = None
        self.det_v = None
        self.det_valid = False

        # ── Weighted least-norm null-space gain (k) ──
        self.k = 0.05

        # ── Joint limits ──
        self.q0_lim = (-math.pi, math.pi)
        self.q1_lim = (-1.5, 1.5)
        self.q2_lim = (-1.5, 1.5)

        # ── YOLO ──
        self.model = YOLO('/home/gurvesh/ros2_ws/hand_yolov8n.pt')
        self.bridge = CvBridge()
        self.prev_delta_u = 0.0
        self.prev_delta_v = 0.0
        self.kd_x = 1.1 # more is more damping less overshoot
        self.kd_y = 1.1
        self.kp_x= 0.7 #more is faster but mroe overshoot
        self.kp_y= 0.7

        # ── QoS: BEST_EFFORT for camera (drops frames rather than blocking) ──
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            cam_qos
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            cam_qos
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # ── Publisher ──
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # ── 50 Hz control timer ──
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info('YoloIKNode ready — 50 Hz control loop running.')

    # ────────────────────────────────────────────────────────────────
    # Camera intrinsics — read once from /camera_info K matrix
    # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    # ────────────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        if not self.intrinsics_received:
            self.fx     = msg.k[0]   # K[0,0]
            self.fy     = msg.k[4]   # K[1,1]
            self.cx_cam = msg.k[2]   # K[0,2]
            self.cy_cam = msg.k[5]   # K[1,2]
            self.intrinsics_received = True
            self.get_logger().info(
                f'Intrinsics set: fx={self.fx:.2f} fy={self.fy:.2f} '
                f'cx={self.cx_cam:.2f} cy={self.cy_cam:.2f}'
            )

    # ────────────────────────────────────────────────────────────────
    # Joint states — keep current θ₀ θ₁ θ₂
    # ────────────────────────────────────────────────────────────────
    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'base_yaw_joint':
                self.q[0] = msg.position[i]
            elif name == 'arm_joint1':
                self.q[1] = msg.position[i]
            elif name == 'arm_joint2':
                self.q[2] = msg.position[i]
        self.got_joints = True

    # ────────────────────────────────────────────────────────────────
    # Image callback — run YOLO, store bbox centre pixels only
    # ────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        display_scale = 1.8  # change to whatever size you want
        display_frame = cv2.resize(frame, None, fx=display_scale, fy=display_scale)

        results = self.model(frame, verbose=False)

        self.det_valid = False

        for result in results:
            for box in result.boxes:
                # Target class: 39 = bottle in COCO; adjust if needed
                if int(box.cls[0]) != 0:
                    continue
                if float(box.conf[0]) < 0.5:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                self.det_u = (x1 + x2) / 2.0   # bbox centre u (pixels)
                self.det_v = (y1 + y2) / 2.0   # bbox centre v (pixels)
                self.det_valid = True

                # Overlay for debug window (scaled to display_frame)
                cv2.rectangle(display_frame,
                              (int(x1 * display_scale), int(y1 * display_scale)),
                              (int(x2 * display_scale), int(y2 * display_scale)),
                              (0, 255, 0), 2)
                cv2.circle(display_frame,
                           (int(self.det_u * display_scale), int(self.det_v * display_scale)),
                           5, (0, 0, 255), -1)
                break   # use first valid detection only

        cv2.imshow('YOLO', display_frame)
        cv2.waitKey(1)

    # ────────────────────────────────────────────────────────────────
    # 50 Hz control loop — all IK math lives here
    # ────────────────────────────────────────────────────────────────
    def control_loop(self):
     if not self.got_joints:
         self.get_logger().warn('Waiting for /joint_states...', throttle_duration_sec=2.0)
         return
     if not self.det_valid:
         return   # no detection — hold position, publish nothing
     self.det_valid = False  # consume detection — one command per detection
     # ── Step 1: pixel error ──────────────────────────────────────
     # δu = u_bbox - cx_image,  δv = v_bbox - cy_image
     delta_u = self.det_u - self.cx_cam
     delta_v = self.det_v - self.cy_cam 

     #very high value too much overshoot # very low value to many nputs controler not process
     # ── Skip if pixel error hasn't changed significantly since last command ──
     if abs(delta_u - self.prev_delta_u) < 30 and abs(delta_v - self.prev_delta_v) < 30:
        return

     # ── D term — rate of change of error ────────────────────────
     # d_delta_u/dt: how fast error is changing, opposes overshoot
     d_delta_u = delta_u - self.prev_delta_u
     d_delta_v = delta_v - self.prev_delta_v
     self.prev_delta_u = delta_u
     self.prev_delta_v = delta_v
     
     

     # ── Dead zone — skip if hand is already near image centre ────
     # avoids flooding controller with tiny corrections
     #if abs(delta_u) < 30 and abs(delta_v) < 60:
        # return

     # ── Step 2: angular error via pinhole model ──────────────────
     # α_x = arctan(δu / fx)   — horizontal angle to object
     # α_y = arctan(δv / fy)   — vertical angle to object
     alpha_x = self.kp_x * math.atan2(delta_u, self.fx) - self.kd_x * math.atan2(d_delta_u, self.fx)
     alpha_y = self.kp_y * math.atan2(delta_v, self.fy) - self.kd_y * math.atan2(d_delta_v, self.fy)

     # ── Step 3: solve θ₀ (yaw) ───────────────────────────────────
     # θ₀_new = θ₀_current + α_x
     q0_new = self.q[0] + alpha_x

     # ── Step 4: solve θ₁ θ₂ — Option B Weighted Least Norm ──────
     # Constraint:  Δθ₁ + Δθ₂ = α_y   (both joints share pitch axis)
     # Null-space gradient projection minimises distance from joint centre:
     #   Δθ₁ = α_y/2 − k·θ₁
     #   Δθ₂ = α_y/2 + k·θ₁
     # k pushes joints back toward 0 (midpoint of ±1.5 limits)
     delta_q1 = alpha_y / 2.0 - self.k * self.q[1]
     delta_q2 = alpha_y / 2.0 + self.k * self.q[1]
     q1_new = self.q[1] + delta_q1
     q2_new = self.q[2] + delta_q2

     # ── Step 5: clamp to joint limits ────────────────────────────
     q0_new = max(self.q0_lim[0], min(self.q0_lim[1], q0_new))
     q1_new = max(self.q1_lim[0], min(self.q1_lim[1], q1_new))
     q2_new = max(self.q2_lim[0], min(self.q2_lim[1], q2_new))

     # ── Joint change gate — skip if change is negligible ─────────
     # prevents flooding controller when hand is nearly centred
     #dq0 = abs(q0_new - self.q[0])
     #dq1 = abs(q1_new - self.q[1])
     #dq2 = abs(q2_new - self.q[2])
     #if dq0 < 0.1 and dq1 < 0.1 and dq2 < 0.1:
         #return

     self.get_logger().info(
         f'δu={delta_u:.1f} δv={delta_v:.1f} | '
         f'αx={math.degrees(alpha_x):.2f}° αy={math.degrees(alpha_y):.2f}° | '
         f'q0={q0_new:.3f} q1={q1_new:.3f} q2={q2_new:.3f}'
     )

     # ── Step 6: publish to joint trajectory controller ───────────
     # time_from_start = 0.1s (one timer period — always fresh short horizon)
     traj = JointTrajectory()
     traj.header.stamp = self.get_clock().now().to_msg()
     traj.joint_names = ['base_yaw_joint', 'arm_joint1', 'arm_joint2']
     pt = JointTrajectoryPoint()
     pt.positions  = [q0_new, q1_new, q2_new]
     pt.velocities = [0.0, 0.0, 0.0]
     pt.time_from_start.sec     = 0
     pt.time_from_start.nanosec = 100_000_000   # 0.1 s
     traj.points = [pt]
     self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = YoloIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()