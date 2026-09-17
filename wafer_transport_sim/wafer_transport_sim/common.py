"""ROS wiring and simulation-clock watchdogs."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String, Float64, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DemoNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.values = {}
        self.received = {}
        self.publishers_by_topic = {}
        self.joints = {}
        self.joint_received = {}
        self.state = ''
        self.entered = self.now()
        self.param('sensor_timeout', 1.0)
        self.param('state_timeout', 30.0)

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def watch(self, topic, msg_type, qos=10):
        def receive(msg):
            self.values[topic] = msg
            self.received[topic] = self.now()
        self.create_subscription(msg_type, topic, receive, qos)

    def fresh(self, *topics):
        timeout = self.get_parameter('sensor_timeout').value
        return all(t in self.received and
                   0 <= self.now() - self.received[t] < timeout for t in topics)

    def feedback_issues(self, *topics):
        issues = []
        for topic in topics:
            if topic not in self.received:
                issues.append(topic + ': no messages')
            elif not self.fresh(topic):
                issues.append(f'{topic}: stale ({self.now() - self.received[topic]:.2f}s)')
        return issues

    def value(self, topic, default=None):
        msg = self.values.get(topic)
        return getattr(msg, 'data', default)

    def pub(self, topic, msg_type, latched=False):
        if topic not in self.publishers_by_topic:
            self.publishers_by_topic[topic] = self.create_publisher(
                msg_type, topic, LATCHED if latched else 10)
        return self.publishers_by_topic[topic]

    def send(self, topic, msg_type, value=None, latched=False):
        msg = msg_type()
        if value is not None:
            msg.data = value
        self.pub(topic, msg_type, latched).publish(msg)

    def change(self, state, message=None):
        if self.state != state:
            self.state = state
            self.entered = self.now()
            self.get_logger().info(message or state)

    def fault(self, reason):
        if self.state != 'FAULT':
            self.get_logger().error(reason)
            self.change('FAULT')
            self.send('/system/fault', String, f'{self.get_name()}: {reason}', True)

    def watch_joints(self, topic):
        def receive(msg):
            self.values[topic] = msg
            self.received[topic] = self.now()
            for name, position in zip(msg.name, msg.position):
                key = name.split('::')[-1]
                self.joints[key] = position
                self.joint_received[key] = self.now()
        self.create_subscription(JointState, topic, receive, 10)

    def at_joint(self, name, target, tolerance=0.002):
        return (name in self.joints and
                self.now() - self.joint_received[name] <
                self.get_parameter('sensor_timeout').value and
                abs(self.joints[name] - target) <= tolerance)

    def joint(self, name, target):
        self.send('/actuators/' + name, Float64, float(target))

    def attach(self, name, enabled):
        self.send('/attachments/' + name + ('/attach' if enabled else '/detach'), Empty)

    def attachment(self, name):
        return self.value('/attachments/' + name + '/state')

    def watch_attachment(self, name):
        self.watch('/attachments/' + name + '/state', String)

    def watch_pose(self, name):
        self.watch('/poses/' + name, PoseStamped)

    def position(self, name):
        msg = self.values.get('/poses/' + name)
        if msg is None:
            return (float('nan'),) * 3
        p = msg.pose.position
        return p.x, p.y, p.z


def run(node_type, args=None):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
