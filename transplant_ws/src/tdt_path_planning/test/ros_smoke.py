#!/usr/bin/env python3
import argparse
import csv
import copy
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
from visualization_msgs.msg import Marker, MarkerArray


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
    dynamic_tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 1)
    update_pub = node.create_publisher(OccupancyGridUpdate, '/costmap/costmap_updates', 10)
    statuses, paths = [], []
    node.create_subscription(String, '/snapshot_planner/status',
                             lambda message: statuses.append(message.data),
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Path, '/snapshot_planner/optimized_path',
                             lambda message: paths.append(message), 10)
    raw_paths, simplified_paths, corridors = [], [], []
    node.create_subscription(Path, '/snapshot_planner/raw_path',
                             lambda message: raw_paths.append(message), 10)
    node.create_subscription(Path, '/snapshot_planner/simplified_path',
                             lambda message: simplified_paths.append(message), 10)
    node.create_subscription(MarkerArray, '/snapshot_planner/corridors',
                             lambda message: corridors.append(message), 10)
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
    publishing = False

    dynamic_message = copy.deepcopy(base)
    stamp_offset = 0.0

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        last_publish = 0
        while time.monotonic() < deadline:
            assert process.poll() is None, 'planner exited unexpectedly'
            if publishing and time.monotonic() - last_publish >= 0.1:
                stamp = node.get_clock().now().nanoseconds + int(stamp_offset * 1e9)
                dynamic_message.header.stamp.sec = stamp // 1000000000
                dynamic_message.header.stamp.nanosec = stamp % 1000000000
                local_pub.publish(dynamic_message)
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

    streams = (paths, raw_paths, simplified_paths, corridors)

    def pump(seconds=0.3):
        start = time.monotonic()
        wait(lambda: time.monotonic() - start >= seconds)

    def output_counts():
        return tuple(len(stream) for stream in streams)

    def assert_empty():
        wait(lambda: all(stream and not stream[-1].poses for stream in streams[:3]) and
             corridors and all(marker.action == Marker.DELETEALL for marker in corridors[-1].markers))

    def reject_goal(reason):
        statuses.clear()
        counts = output_counts()
        goal(6, 4)
        wait(lambda: any(reason in text for text in statuses))
        assert_empty()
        pump()
        for stream, count in zip(streams[:3], counts[:3]):
            assert all(not message.poses for message in stream[count:])
        assert all(marker.action == Marker.DELETEALL
                   for message in corridors[counts[3]:] for marker in message.markers)

    def successful_goal(x=6, y=4):
        pump(0.15)
        statuses.clear()
        counts = output_counts()
        goal(x, y)
        wait(lambda: any('total_ms=' in text for text in statuses))
        wait(lambda: all(len(stream) > count and stream[-1].poses
                         for stream, count in zip(streams[:3], counts[:3])) and
             len(corridors) > counts[3] and
             any(marker.action == Marker.ADD for marker in corridors[-1].markers))
        for stream in streams[:3]:
            assert abs(stream[-1].poses[-1].pose.position.x - x) < 1e-9
            assert abs(stream[-1].poses[-1].pose.position.y - y) < 1e-9
        return paths[-1]

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
        tf_pub.publish(TFMessage(transforms=[transform]))
        wait(lambda: goal_pub.get_subscription_count() > 0)
        reject_goal('static map not received')
        base_pub.publish(base)
        pump()
        reject_goal('dynamic full map not received')
        invalid = copy.deepcopy(base)
        invalid.header.frame_id = 'odom'
        local_pub.publish(invalid)
        pump()
        reject_goal('dynamic map rejected')
        publishing = True
        planned = successful_goal()
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
        pump()
        observed = output_counts()
        pump(0.6)
        assert observed == output_counts(), \
            'map callbacks published output after successful planning'
        print('PASS full-map updates preserve all displayed results', flush=True)
        update = OccupancyGridUpdate()
        update.header.frame_id = 'map'
        update.header.stamp = node.get_clock().now().to_msg()
        update.width = update.height = 1
        update.data = [100]
        latest_status = statuses[-1]
        for repeat in range(5):
            update_pub.publish(update)
            base_pub.publish(base)
            pump(0.1)
        assert output_counts() == observed
        assert statuses[-1] == latest_status
        assert not node.get_subscriptions_info_by_topic('/costmap/costmap_updates')
        assert not any('waiting for full map' in text for text in statuses)
        print('PASS increments and static/full-map updates preserve outputs and goal status', flush=True)

        publishing = False
        pump(0.7)
        assert output_counts() == observed
        for repeat in range(3):
            update_pub.publish(update)
            pump(0.1)
        reject_goal('dynamic receive age exceeded')
        assert any('clock=steady' in text and 'max_age_s=' in text for text in statuses)
        print('PASS expiry retains history; new goal clears it; increments cannot refresh input', flush=True)

        publishing = True
        pump(0.3)
        assert_empty()
        assert not any('planning snapshot' in text for text in statuses)
        print('PASS restored inputs do not retry a rejected goal', flush=True)
        successful_goal()

        observed = output_counts()
        invalid = copy.deepcopy(base)
        invalid.data = [0]
        base_pub.publish(invalid)
        pump()
        assert output_counts() == observed
        reject_goal('static map rejected')
        base_pub.publish(base)
        successful_goal()
        publishing = False
        observed = output_counts()
        local_pub.publish(invalid)
        pump()
        assert output_counts() == observed
        reject_goal('dynamic map rejected')
        print('PASS malformed static/dynamic maps invalidate inputs, not historical display', flush=True)

        local_pub.publish(base)
        pump(0.05)
        reject_goal('dynamic timestamp is zero')
        publishing = True
        stamp_offset = -2.0
        pump(0.2)
        reject_goal('dynamic timestamp age exceeded')
        assert any('stamp_age_s=' in text and 'clock=ROS' in text for text in statuses)
        stamp_offset = 2.0
        pump(0.2)
        reject_goal('dynamic timestamp is in future')
        stamp_offset = 0.0
        print('PASS zero, old and future dynamic timestamps rejected', flush=True)

        successful_goal()
        statuses.clear()
        goal(0.1, 0.1)
        wait(lambda: any('invalid start or goal' in text for text in statuses))
        assert_empty()
        pump()
        assert_empty()
        print('PASS invalid endpoint clears debug reference', flush=True)

        successful_goal()
        statuses.clear()
        counts = output_counts()
        goal(6, 4)
        wait(lambda: any('planning snapshot' in text for text in statuses))
        goal(5, 4)
        wait(lambda: sum('goal received' in text for text in statuses) >= 2)
        goal(4, 4)
        wait(lambda: sum('goal received' in text for text in statuses) >= 3)
        assert not any('total_ms=' in text for text in statuses), 'old task finished before replacement was accepted'
        wait(lambda: any('total_ms=' in text for text in statuses))
        wait(lambda: paths and paths[-1].poses and
             abs(paths[-1].poses[-1].pose.position.x - 4) < 1e-9)
        pump()
        for stream, count in zip(streams[:3], counts[:3]):
            for message in stream[count:]:
                if message.poses:
                    assert abs(message.poses[-1].pose.position.x - 4) < 1e-9
        print('PASS rapid new goals clear outputs and superseded async work cannot publish', flush=True)

        statuses.clear()
        goal(6, 4)
        wait(lambda: any('planning snapshot' in text for text in statuses))
        assert not any('total_ms=' in text for text in statuses)
        dynamic_message.data = [100] * len(base.data)
        local_pub.publish(dynamic_message)
        wait(lambda: any('latest map rejects candidate path' in text for text in statuses))
        assert_empty()
        print('PASS latest-map final collision validation rejects an obsolete candidate', flush=True)
        dynamic_message.data = list(base.data)
        successful_goal()

        statuses.clear()
        goal(6, 4)
        wait(lambda: any('planning snapshot' in text for text in statuses))
        transform.transform.translation.x = 1.2
        tf_pub.publish(TFMessage(transforms=[transform]))
        wait(lambda: any('start moved' in text for text in statuses))
        assert_empty()
        transform.transform.translation.x = 1.0
        tf_pub.publish(TFMessage(transforms=[transform]))
        print('PASS start drift check retained', flush=True)

        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        statuses.clear()
        process = subprocess.Popen([executable, '--ros-args', '--params-file', configuration,
                                    '-p', 'robot_frame:=test_dynamic_base'], stdout=log, stderr=log)
        wait(lambda: any('ready:' in text for text in statuses))
        wait(lambda: dynamic_tf_pub.get_subscription_count() > 0)
        moving_transform = copy.deepcopy(transform)
        moving_transform.child_frame_id = 'test_dynamic_base'
        moving_transform.header.stamp = node.get_clock().now().to_msg()
        moving_transform.header.stamp.sec -= 2
        dynamic_tf_pub.publish(TFMessage(transforms=[moving_transform]))
        pump(0.2)
        reject_goal('stale robot TF')
        print('PASS stale dynamic TF rejected (static TF has no age limit)', flush=True)

        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        statuses.clear()
        process = subprocess.Popen([executable, '--ros-args', '--params-file', configuration,
                                    '-p', 'qp_max_iterations:=1'], stdout=log, stderr=log)
        wait(lambda: any('ready:' in text for text in statuses))
        pump(0.5)
        reject_goal('planning failed:')
        assert any('planning snapshot' in text for text in statuses)
        print('PASS optimizer failure publishes no partial raw/simplified/corridor outputs', flush=True)

        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        statuses.clear()
        process = subprocess.Popen([executable, '--ros-args', '--params-file', configuration,
                                    '-p', 'static_only:=true'], stdout=log, stderr=log)
        publishing = False
        wait(lambda: any('ready:' in text for text in statuses))
        successful_goal()
        print('PASS explicit static-only mode remains available', flush=True)
        assert not node.get_publishers_info_by_topic('/sPath')
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        print('PASS no execution publishers', flush=True)
    finally:
        if process.poll() is None:
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
