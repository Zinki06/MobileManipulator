"""Single stationary-base grasp trial; never sends navigation or velocity commands."""

import argparse
from datetime import datetime
import json
import time

from cleanup_interfaces.srv import CaptureObjects, EvaluateGrasp
from geometry_msgs.msg import PointStamped
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main(args=None):
    """Capture and evaluate one banana, executing only with an explicit flag."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='Move the arm and attempt one pick; base always remains uncommanded')
    options = parser.parse_args(args)
    rclpy.init()
    node = rclpy.create_node('stationary_grasp_trial')
    events = node.create_publisher(String, '/cleanup/events', 20)
    targets = node.create_publisher(PointStamped, '/cleanup/pick_target', 10)

    def call(kind, name, request, timeout=30):
        client = node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'Service unavailable: {name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'Timeout: {name}; inspect robot before another request')
        response = future.result()
        if not response.success:
            raise RuntimeError(f'{name}: {response.message}')
        return response

    try:
        if options.execute:
            events.publish(String(data='TARGET_VIEW_START|stationary grasp trial'))
            call(Trigger, '/observe_floor', Trigger.Request())
            time.sleep(0.4)
        request = CaptureObjects.Request(
            mission_id='grasp_trial_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
            station_name='station_0_arm_only', heading_index=249,
            burst_frames=3, min_confirmations=3)
        response = call(CaptureObjects, '/cleanup/capture_objects', request)
        candidates = [o for o in response.observations
                      if o.class_name == 'banana' and o.grasp_valid]
        if len(candidates) != 1:
            raise RuntimeError(f'Expected exactly one valid banana, found {len(candidates)}; '
                               f'evidence={response.image_reference}')
        obj = candidates[0]
        target = PointStamped(header=obj.header, point=obj.centroid)
        reachable = call(EvaluateGrasp, '/cleanup/evaluate_grasp',
                         EvaluateGrasp.Request(target=target))
        print(json.dumps({'uuid': obj.object_uuid, 'image': obj.image_reference,
                          'grasp_pixel': [float(value) for value in obj.grasp_pixel],
                          'reachable': reachable.reachable, 'detail': reachable.message},
                         ensure_ascii=False), flush=True)
        if options.execute:
            if not reachable.reachable:
                raise RuntimeError(
                    'Outside arm workspace; NO automatic base approach in this trial')
            targets.publish(target)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            events.publish(String(data=f'PICK_START|UUID={obj.object_uuid}; stationary trial'))
            result = call(Trigger, '/execute_pick_and_place', Trigger.Request(), 45)
            print(result.message, flush=True)
            events.publish(String(data='PICK_VERIFIED|Stationary trial encoder hold verified'))
    except Exception as error:
        events.publish(String(data=f'OBJECT_FAILED|stationary trial: {error}'))
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
