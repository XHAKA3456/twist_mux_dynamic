import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
from rcl_interfaces.msg import SetParametersResult

class DynamicTwistMux(Node):
    def __init__(self):
        super().__init__('twist_mux_dynamic')

        # Declare parameters (defaults)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topics.navigation.priority', 50),
                ('topics.navigation.timeout', 0.5),
                ('topics.navigation.topic', 'cmd_vel_nav'),
                ('topics.joystick.priority', 70),
                ('topics.joystick.timeout', 0.5),
                ('topics.joystick.topic', 'cmd_vel_joy'),
                ('topics.arucomaker.priority', 30),
                ('topics.arucomaker.timeout', 0.5),
                ('topics.arucomaker.topic', 'cmd_vel_aruco'),
                ('topics.saftey.priority', 200),
                ('topics.saftey.timeout', 3.0),
                ('topics.saftey.topic', 'cmd_vel_safe'),
                ('locks.saftey.priority', 210),
                ('locks.saftey.topic', 'safe_priority'),
                ('locks.control.priority', 100),
                ('locks.control.topic', 'control_priority')
            ]
        )

        # Initialize input configuration
        self.inputs = {
            'navigation': {
                'priority': self.get_parameter('topics.navigation.priority').value,
                'timeout': self.get_parameter('topics.navigation.timeout').value,
                'topic': self.get_parameter('topics.navigation.topic').value
            },
            'joystick': {
                'priority': self.get_parameter('topics.joystick.priority').value,
                'timeout': self.get_parameter('topics.joystick.timeout').value,
                'topic': self.get_parameter('topics.joystick.topic').value
            },
            'arucomaker': {
                'priority': self.get_parameter('topics.arucomaker.priority').value,
                'timeout': self.get_parameter('topics.arucomaker.timeout').value,
                'topic': self.get_parameter('topics.arucomaker.topic').value
            },
            'saftey': {
                'priority': self.get_parameter('topics.saftey.priority').value,
                'timeout': self.get_parameter('topics.saftey.timeout').value,
                'topic': self.get_parameter('topics.saftey.topic').value
            }
        }

        self.locks = {}
        self.lock_priorities = {
            'saftey': self.get_parameter('locks.saftey.priority').value,
            'control': self.get_parameter('locks.control.priority').value
        }
        self.lock_topics = {
            'saftey': self.get_parameter('locks.saftey.topic').value,
            'control': self.get_parameter('locks.control.topic').value
        }

        # Last messages and timestamps
        self.last_msgs = {}
        self.last_msg_times = {}

        # Lock subscriptions
        for lock_key, topic_name in self.lock_topics.items():
            self.create_subscription(Bool, topic_name, lambda msg, k=lock_key: self.update_lock(k, msg), 10)

        # Create subscriptions dynamically based on input topics
        for key, cfg in self.inputs.items():
            self.create_subscription(Twist, cfg['topic'], lambda msg, k=key: self.store_msg(k, msg), 10)

        # Output publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to evaluate and publish
        self.create_timer(0.05, self.select_and_publish)

        # Parameter change callback
        self.add_on_set_parameters_callback(self.on_param_change)

    def store_msg(self, source, msg):
        self.last_msgs[source] = msg
        self.last_msg_times[source] = time.time()

    def update_lock(self, source, msg):
        self.locks[source] = msg.data

    def on_param_change(self, params):
        for param in params:
            name = param.name
            if name.startswith('topics.') and name.endswith('.priority'):
                key = name.split('.')[1]
                self.inputs[key]['priority'] = param.value
                self.get_logger().info(f"Updated priority: {key} = {param.value}")
            elif name.startswith('topics.') and name.endswith('.timeout'):
                key = name.split('.')[1]
                self.inputs[key]['timeout'] = param.value
                self.get_logger().info(f"Updated timeout: {key} = {param.value}")
            elif name.startswith('topics.') and name.endswith('.topic'):
                key = name.split('.')[1]
                self.inputs[key]['topic'] = param.value
                self.get_logger().warn(f"Updated topic name for {key}, but dynamic re-subscription not implemented")
            elif name.startswith('locks.') and name.endswith('.priority'):
                key = name.split('.')[1]
                self.lock_priorities[key] = param.value
                self.get_logger().info(f"Updated lock priority: {key} = {param.value}")
            elif name.startswith('locks.') and name.endswith('.topic'):
                key = name.split('.')[1]
                self.lock_topics[key] = param.value
                self.get_logger().warn(f"Updated lock topic name for {key}, but dynamic re-subscription not implemented")
        return SetParametersResult(successful=True)

    def select_and_publish(self):
        if not self.last_msgs:
            return

        now = time.time()
        valid_sources = []

        for k, cfg in self.inputs.items():
            if k not in self.last_msg_times:
                continue
            if now - self.last_msg_times.get(k, 0) >= cfg['timeout']:
                continue
            # Check locks
            blocked = False
            for lock, active in self.locks.items():
                if active and cfg['priority'] < self.lock_priorities[lock]:
                    blocked = True
                    break
            if blocked:
                continue
            valid_sources.append(k)

        if not valid_sources:
            return

        selected_source = max(valid_sources, key=lambda k: self.inputs[k]['priority'])
        self.cmd_pub.publish(self.last_msgs[selected_source])
        self.get_logger().info(f"Published cmd_vel from: {selected_source}")


def main():
    rclpy.init()
    node = DynamicTwistMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
