import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Wrench
from nav_msgs.msg import Odometry


class GeometricController(Node):
    """
    Geometric tracking controller for quadcopter.
    Based on Lee et al. 2010 - geometric tracking control on SE(3).

    Subscribes:
        /model/quadcopter/pose      → position + orientation (quaternion)
        /model/quadcopter/odometry  → linear + angular velocity

    Publishes:
        /block/force                → thrust (force.z) + torque (torque.xyz)
    """

    def __init__(self):
        super().__init__('geometric_controller')

        # ── Gains ──────────────────────────────────────────────────
        self.Kp = np.diag([5.0,  5.0,  5.0 ])   # position gain
        self.Kd = np.diag([4.0,  4.0,  4.0 ])   # velocity gain
        self.Ki = np.diag([0.5,  0.5,  0.5 ])   # integral gain
        self.KR = np.diag([0.3,  0.3,  0.05])   # rotation gain
        self.Kw = np.diag([0.05, 0.05, 0.02])   # angular velocity gain

        # ── Physical parameters ────────────────────────────────────
        self.m  = 0.027                                       # mass (kg)
        self.g  = 9.81                                         # gravity (m/s²)
        self.J  = np.diag([1.395e-5, 1.395e-5, 2.173e-5])    # inertia matrix
        self.dt = 0.1                                        # control timestep (s)

        # ── Integral accumulator ───────────────────────────────────
        self.ep_int = np.zeros(3)

        # ── Current state (updated by subscribers) ─────────────────
        self.p     = np.array([0.0, 0.0, 0.0])                       # position
        self.q     = np.array([1.0, 0.0, 0.0, 0.0])   # quaternion [w, x, y, z]
        self.v     = np.zeros(3)                        # linear velocity
        self.omega = np.zeros(3)                        # angular velocity

        # ── Setpoints (edit here or make ROS params) ───────────────
        self.pd    = np.array([0.0, 0.0, 1.0])   # desired position (hover at 1m)
        self.vd    = np.zeros(3)                  # desired velocity
        self.ad    = np.zeros(3)                  # desired acceleration
        self.psi_d = 0.0                          # desired yaw angle (rad)

        # ── Subscribers ────────────────────────────────────────────
        self.create_subscription(
            PoseStamped,
            '/model/quadcopter/pose',
            self.pose_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            10
        )

        # ── Publisher ──────────────────────────────────────────────
        self.force_pub = self.create_publisher(Wrench, '/block/force', 10)

        # ── Control loop @ 1 kHz ───────────────────────────────────
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Geometric controller started. Hovering at z=1.0m')

    # ──────────────────────────────────────────────────────────────
    # Subscriber callbacks
    # ──────────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseStamped):
        """Extract position and orientation from /model/quadcopter/pose."""
        self.p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        # Store as [w, x, y, z] to match MATLAB convention
        self.q = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])

    def odom_callback(self, msg: Odometry):
        """Extract linear and angular velocity from /model/quadcopter/odometry."""
        self.v = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        self.omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])

    # ──────────────────────────────────────────────────────────────
    # Math helpers
    # ──────────────────────────────────────────────────────────────

    def quat_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
        w, x, y, z = q
        R = np.array([
            [1 - 2*y**2 - 2*z**2,   2*x*y - 2*z*w,       2*x*z + 2*y*w    ],
            [2*x*y + 2*z*w,          1 - 2*x**2 - 2*z**2,  2*y*z - 2*x*w    ],
            [2*x*z - 2*y*w,          2*y*z + 2*x*w,        1 - 2*x**2 - 2*y**2],
        ])
        return R

    def vee(self, S: np.ndarray) -> np.ndarray:
        """Vee map: extracts the 3-vector from a skew-symmetric 3x3 matrix."""
        return np.array([S[2, 1], S[0, 2], S[1, 0]])

    # ──────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────

    def control_loop(self):
        # ── Position & velocity errors ─────────────────────────────
        ep = self.p - self.pd          # position error
        ev = self.v - self.vd          # velocity error
        self.ep_int += ep * self.dt    # integral of position error

        # ── Desired acceleration ───────────────────────────────────
        a_des = (self.ad
                 - self.Kp @ ep
                 - self.Kd @ ev
                 - self.Ki @ self.ep_int)

        # ── Desired thrust vector (world frame) ────────────────────
        f_des = self.m * (a_des - np.array([0.0, 0.0, -self.g]))

        # ── Rotation matrix from current quaternion ────────────────
        R = self.quat_to_rotation_matrix(self.q)

        # ── Collective thrust (project onto body z-axis) ──────────
        thrust = float(f_des @ (R @ np.array([0.0, 0.0, 1.0])))
        thrust = float(np.clip(thrust, 0.0, 1.5))

        # ── Desired attitude (Rd) ──────────────────────────────────
        b3_des = f_des / np.linalg.norm(f_des)
        b1_c   = np.array([np.cos(self.psi_d), np.sin(self.psi_d), 0.0])
        b2_des = np.cross(b3_des, b1_c)
        b2_des = b2_des / np.linalg.norm(b2_des)
        b1_des = np.cross(b2_des, b3_des)           # orthogonalized b1
        Rd     = np.column_stack([b1_des, b2_des, b3_des])

        # ── Attitude error ─────────────────────────────────────────
        eR = 0.5 * self.vee(Rd.T @ R - R.T @ Rd)

        # ── Angular velocity error ─────────────────────────────────
        eomega = self.omega                          # desired omega = 0

        # ── Control torque ─────────────────────────────────────────
        torque = (- self.KR @ eR
                  - self.Kw @ eomega
                  + np.cross(self.omega, self.J @ self.omega))
        torque = np.clip(torque, -0.02, 0.02)

        #------------converter----------------
        thrust_vector = R @ np.array([0.0, 0.0, thrust])  # thrust in world frame
        torquer = R @ torque  # torque in world frame

        # ── Publish Wrench ─────────────────────────────────────────
        msg = Wrench()
        msg.force.x = thrust_vector[0]
        msg.force.y = thrust_vector[1]
        msg.force.z = thrust_vector[2]
        msg.torque.x = torquer[0]
        msg.torque.y = torquer[1]
        msg.torque.z = torquer[2]
        self.force_pub.publish(msg)

        # Optional debug log (comment out for performance)
        self.get_logger().info(f'thrust={thrust:.3f}  torque={torque}  pos={self.p}')
        # )


# ──────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = GeometricController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
