#!/usr/bin/env python3
import argparse
import csv
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rcl_interfaces.srv import GetParameters


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bin-dir', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('use an isolated nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    executable = os.path.join(args.bin_dir, 'snapshot_planner')
    configuration = os.path.abspath(os.path.join(
        args.bin_dir, '..', '..', 'share', 'tdt_path_planning', 'cfg', 'planner.yaml'))
    for parameter in ('enable_execution:=true', 'allocation_speed:=0.0',
                      'max_control_points:=4294967298'):
        rejected = subprocess.run([executable, '--ros-args', '--params-file', configuration, '-p', parameter],
                                  capture_output=True, text=True, timeout=10)
        assert rejected.returncode == 2, rejected.stderr
        print('PASS startup rejects', parameter, flush=True)
    rclpy.init()
    node = rclpy.create_node('phase1_smoke')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    base_pub = node.create_publisher(OccupancyGrid, '/map', latched)
    local_pub = node.create_publisher(OccupancyGrid, '/costmap/costmap', latched)
    tf_pub = node.create_publisher(TFMessage, '/tf_static', latched)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 1)
    update_pub = node.create_publisher(OccupancyGridUpdate, '/costmap/costmap_updates', 10)
    statuses, paths = [], []
    node.create_subscription(String, '/snapshot_planner/status',
                             lambda message: statuses.append(message.data), latched)
    node.create_subscription(Path, '/snapshot_planner/optimized_path',
                             lambda message: paths.append(message), 10)
    directory = tempfile.TemporaryDirectory(prefix='phase1-smoke-')
    snapshot_file = os.path.join(directory.name, 'input.snapshot')
    log = open(os.path.join(directory.name, 'node.log'), 'w+')
    process = subprocess.Popen([executable, '--ros-args', '--params-file', configuration, '-p',
                                'snapshot_file:=' + snapshot_file], stdout=log, stderr=log)
    base = OccupancyGrid()
    base.header.frame_id = 'map'
    base.info.width, base.info.height = 80, 60
    base.info.resolution = 0.1
    base.info.origin.orientation.w = 1.0
    base.data = [0] * 4800
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
    transform.transform.translation.x = transform.transform.translation.y = 1.0
    transform.transform.rotation.w = 1.0
    publishing = True

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        last_publish = 0
        while time.monotonic() < deadline:
            assert process.poll() is None, 'planner exited unexpectedly'
            if publishing and time.monotonic() - last_publish >= 0.1:
                base.header.stamp = node.get_clock().now().to_msg()
                local_pub.publish(base)
                last_publish = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('timeout; statuses=' + repr(statuses[-10:]))

    def goal(x, y):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.orientation.w = 1.0
        message.pose.position.x, message.pose.position.y = float(x), float(y)
        goal_pub.publish(message)

    try:
        wait(lambda: any('ready:' in text for text in statuses))
        parameters = node.create_client(GetParameters, '/snapshot_planner/get_parameters')
        wait(lambda: parameters.service_is_ready())
        request = GetParameters.Request()
        request.names = ['enable_execution', 'body_length', 'clearance_margin', 'allocation_speed']
        values = parameters.call_async(request)
        wait(values.done)
        actual = values.result().values
        assert not actual[0].bool_value
        assert actual[1].double_value == 0.6
        assert actual[2].double_value == 0.08
        assert actual[3].double_value == 0.2
        print('PASS installed YAML parameters actually loaded', flush=True)
        assert not node.get_publishers_info_by_topic('/sPath')
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        base_pub.publish(base)
        tf_pub.publish(TFMessage(transforms=[transform]))
        wait(lambda: goal_pub.get_subscription_count() > 0)
        start = time.monotonic()
        wait(lambda: time.monotonic() - start > 0.5)
        goal(6, 4)
        wait(lambda: any(len(path.poses) > 1 for path in paths))
        planned = next(path for path in paths if len(path.poses) > 1)
        csv_file = os.path.join(directory.name, 'path.csv')
        offline = subprocess.run([os.path.join(args.bin_dir, 'offline_planner'), snapshot_file, csv_file],
                                 capture_output=True, text=True, timeout=10)
        assert offline.returncode == 0, offline.stdout + offline.stderr
        with open(csv_file) as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == len(planned.poses)
        for row, pose in zip(rows, planned.poses):
            assert abs(float(row['x_m']) - pose.pose.position.x) < 1e-9
            assert abs(float(row['y_m']) - pose.pose.position.y) < 1e-9
        print('PASS live full-map updates do not cancel; ROS/offline paths agree;', offline.stdout.strip(), flush=True)
        statuses.clear()
        goal(0.1, 0.1)
        wait(lambda: any('invalid start or goal' in text for text in statuses))
        assert not paths[-1].poses
        print('PASS invalid endpoint clears debug reference', flush=True)
        publishing = False
        start = time.monotonic()
        wait(lambda: time.monotonic() - start > 0.7)
        statuses.clear()
        goal(6, 4)
        wait(lambda: any('missing or stale' in text for text in statuses))
        print('PASS stale input rejected', flush=True)
        update = OccupancyGridUpdate()
        update.header.frame_id = 'map'
        update.header.stamp = node.get_clock().now().to_msg()
        update.width = update.height = 1
        update.data = [100]
        statuses.clear()
        update_pub.publish(update)
        wait(lambda: any('waiting for full map' in text for text in statuses))
        print('PASS unbound increment waits for full-map resynchronization', flush=True)
        print('PASS no execution publishers', flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.seek(0)
        print(log.read(), flush=True)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        directory.cleanup()


if __name__ == '__main__':
    main()
