#!/usr/bin/env python3
"""
Pre-flight GPS position visibility check -- NOT part of formation_control_core.py.

Assumes formation_control_core.py has been updated to give each real
vehicle's uxrce_dds_client its own namespace (REAL_NAMESPACE dict /
get_namespace() fix -- see that file). Without that fix, every real
vehicle's GPS bridges to the SAME bare topic name and this script
(and the formation code) can't tell drones apart at all.

Checks, per drone, on its OWN Pi:
  1. Is this drone's own GPS global position valid (lat_lon_valid,
     alt_valid) -- same check own_global_position_callback() does.
  2. Can this Pi also see the OTHER drone's namespaced topic? This
     proves cross-machine DDS discovery is actually working, which
     is what formation_control_core.py's neighbor subscriptions
     depend on entirely.

TESTING JUST ONE DRONE (no multi-vehicle namespace setup yet):
    Skip --other entirely and use --topic to point at whatever your
    GPS topic actually is right now -- almost certainly the plain
    default, since the -n namespace flag is a multi-drone-only step:
        python3 gps_position_check.py --own 0 --topic /fmu/out/vehicle_global_position
    This only checks fix validity + prints live lat/lon/alt. No
    network/cross-machine check happens, since there's no "other" yet.

TESTING TWO+ DRONES (after applying the REAL_NAMESPACE fix and
starting each Pi's uxrce_dds_client with -n droneN):
    Run this on BOTH Raspberry Pis at the same time, each with its
    own --own/--other ids (topic defaults to /drone{id}/... in this
    case, no need to pass --topic):
        python3 gps_position_check.py --own 0 --other 1
    Each should print an OWN line AND an OTHER line (bridged in from
    the other Pi over DDS).

      - If you only ever see OWN: cross-machine discovery isn't
        working (check ROS_DOMAIN_ID matches on both Pis, and that
        your WiFi allows UDP multicast between them). Formation
        code's neighbor data will stay empty -- safe (permanent
        hover), but the formation won't move.
      - If OWN's lat/lon jumps between two clearly different,
        unrelated locations frame-to-frame instead of drifting
        smoothly: that's the topic-collision bug from before -- the
        namespace fix didn't take effect on one vehicle. Don't fly
        until that's resolved.

USAGE:
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 gps_position_check.py --own 0 --other 1
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleGlobalPosition


def make_callback(node, label):
    def callback(msg: VehicleGlobalPosition):
        valid = msg.lat_lon_valid and msg.alt_valid
        status = "VALID" if valid else "NOT VALID YET"
        node.get_logger().info(
            f"[{label}] {status}  lat={msg.lat:.7f} lon={msg.lon:.7f} alt={msg.alt:.2f}m"
        )
    return callback


class GpsPositionCheckNode(Node):
    def __init__(self, own_id: int, other_id: int | None, topic_override: str | None):
        super().__init__("gps_position_check")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Namespaced per drone by default -- must match whatever
        # namespace you started each vehicle's uxrce_dds_client with
        # (REAL_NAMESPACE in formation_control_core.py, e.g. "drone0").
        # --topic overrides this entirely, for single-drone testing
        # before any multi-vehicle namespace setup exists.
        own_topic = topic_override or f"/drone{own_id}/fmu/out/vehicle_global_position"

        self.create_subscription(
            VehicleGlobalPosition, own_topic, make_callback(self, f"OWN drone {own_id}"), qos_profile)
        self.get_logger().info(f"Listening for OWN GPS on {own_topic} ...")

        if other_id is not None:
            other_topic = f"/drone{other_id}/fmu/out/vehicle_global_position"
            self.create_subscription(
                VehicleGlobalPosition, other_topic, make_callback(self, f"OTHER drone {other_id}"), qos_profile)
            self.get_logger().info(f"Listening for OTHER drone's GPS on {other_topic} ...")
            self.get_logger().info(
                "If OTHER never prints within ~10s, cross-machine DDS discovery "
                "isn't working -- check ROS_DOMAIN_ID matches on both Pis and "
                "that your WiFi allows UDP multicast between them."
            )
        else:
            self.get_logger().info(
                "No --other id given -- checking this one drone's GPS only, "
                "no cross-drone network check."
            )


def main():
    parser = argparse.ArgumentParser(description="GPS position validity / cross-visibility check")
    parser.add_argument("--own", type=int, required=True, help="This drone's id")
    parser.add_argument("--other", type=int, default=None,
                         help="The other drone's id (omit if testing just one drone)")
    parser.add_argument("--topic", type=str, default=None,
                         help="Override the OWN topic (e.g. /fmu/out/vehicle_global_position "
                              "for a single drone with no namespace set up yet)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GpsPositionCheckNode(own_id=args.own, other_id=args.other, topic_override=args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
