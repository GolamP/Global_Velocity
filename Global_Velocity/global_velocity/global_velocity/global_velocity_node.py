import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class GlobalVelocityNode(Node):
    def __init__(self):
        super().__init__('global_velocity')

        # Parameter robot Robin (dari file xacro)
        self.r = 0.1      # radius roda (m)
        self.s = 0.45     # jarak antarroda (m)
        self.left_joint = 'base_left_wheel_joint'
        self.right_joint = 'base_right_wheel_joint'

        # Kecepatan sudut roda terbaru (rad/s)
        self.phi_dot_left = None
        self.phi_dot_right = None

        # Listener TF untuk membaca orientasi robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Berlangganan ke /joint_states
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # Hitung dan tampilkan kecepatan setiap 0.5 detik
        self.create_timer(0.5, self.compute_callback)

    def joint_callback(self, msg):
        # Cari posisi kedua roda di dalam daftar nama joint
        if self.left_joint not in msg.name or self.right_joint not in msg.name:
            return

        i_left = msg.name.index(self.left_joint)
        i_right = msg.name.index(self.right_joint)

        if len(msg.velocity) <= max(i_left, i_right):
            return

        self.phi_dot_left = msg.velocity[i_left]
        self.phi_dot_right = msg.velocity[i_right]

    def compute_callback(self):
        if self.phi_dot_left is None or self.phi_dot_right is None:
            return

        # Ambil transformasi odom -> base_footprint
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF belum tersedia: {ex}')
            return

        # Konversi quaternion ke sudut yaw (theta)
        q = tf.transform.rotation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Kecepatan linear roda
        v_left = self.r * self.phi_dot_left
        v_right = self.r * self.phi_dot_right

        # Kecepatan linear dan sudut robot
        v_b = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.s

        # Kecepatan dalam koordinat global
        vx = v_b * math.cos(theta)
        vy = v_b * math.sin(theta)

        self.get_logger().info(f'Vx Global: {vx:.3f} m/s')
        self.get_logger().info(f'Vy Global: {vy:.3f} m/s')
        self.get_logger().info(f'Angular Velocity: {omega:.3f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()