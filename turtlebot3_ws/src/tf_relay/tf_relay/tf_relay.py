from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TFRelay(Node):

    def __init__(self, namespace):
        node_suffix = namespace.replace('/', '_')
        super().__init__('tf_relay_' + node_suffix)
        tf_topic = '/' + namespace + '/tf'
        self.frame_prefix = namespace + '/'
        self.subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_topic,
            callback=self.tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        )
        self.publisher = self.create_publisher(
            msg_type=TFMessage,
            topic='/tf',
            qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10))


    def tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
        self.publisher.publish(msg)
