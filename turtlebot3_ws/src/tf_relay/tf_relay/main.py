import rclpy
from tf_relay.tf_relay import TFRelay
from tf_relay.tf_static_relay import TFStaticRelay
from rclpy.executors import SingleThreadedExecutor
import sys

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    arguments = rclpy.utilities.remove_ros_args(sys.argv)
    if len(arguments) != 2:
        raise RuntimeError(
            'Expected one comma-separated namespace argument, for example T3A,T3D.')
    namespaces = [
        namespace.strip().strip('/')
        for namespace in arguments[1].split(',')
        if namespace.strip()
    ]
    if not namespaces:
        raise RuntimeError('At least one namespace is required.')

    tf_relays = []
    tf_static_relays = []

    try:
        for namespace in namespaces:
            tf_relay = TFRelay(namespace)
            tf_static_relay = TFStaticRelay(namespace)
            tf_relays.append(tf_relay)
            tf_static_relays.append(tf_static_relay)

        for tf_relay in tf_relays:
            executor.add_node(tf_relay)
        
        for tf_static_relay in tf_static_relays:
            executor.add_node(tf_static_relay)

        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        executor.shutdown()


if __name__ == '__main__':
    main()
