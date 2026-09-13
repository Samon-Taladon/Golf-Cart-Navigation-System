#!/usr/bin/env python3
"""For-web entry point for the existing enhanced GNSS publisher.

This file intentionally reuses the reference implementation without editing it.
"""

import rclpy

from navigation_system.enhanced_gnss_publisher import GnssPosePublisher


def main(args=None):
    rclpy.init(args=args)
    node = GnssPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
