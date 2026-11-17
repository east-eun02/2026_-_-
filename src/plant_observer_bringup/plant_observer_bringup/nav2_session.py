"""Run Nav2 with ordered lifecycle shutdown on Ctrl+C, SIGTERM, or terminal close.

    ros2 run plant_observer_bringup nav2_session --map ~/plant_ws/maps/demo_real3.yaml
    ros2 run plant_observer_bringup nav2_shutdown  # orderly stop for an existing Nav2

Keep the robot/simulator running until this command exits. Nav2 stops navigation,
then localization, before its launch process receives SIGINT. Runtime bonds and
collision protection remain enabled.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses, NavigateToPose
from std_srvs.srv import Trigger


MANAGERS = ('lifecycle_manager_navigation', 'lifecycle_manager_localization')
ACTIONS = {'follow_waypoints': FollowWaypoints,
           'navigate_through_poses': NavigateThroughPoses,
           'navigate_to_pose': NavigateToPose}


def service_name(namespace, node, suffix):
    return '/' + '/'.join(part for part in (namespace.strip('/'), node, suffix) if part)


def call_service(node, service_type, name, request, timeout=15., discovery_timeout=2., client=None,
                 stopping=None):
    """Use wall-clock deadlines even if /clock has stopped."""
    owned_client = client is None
    if owned_client:
        client = node.create_client(service_type, name)
    started = time.monotonic()
    future = None
    try:
        if not client.wait_for_service(timeout_sec=discovery_timeout):
            return {'service': name, 'success': False, 'reason': 'service_unavailable'}
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if stopping is not None and stopping.is_set():
                future.cancel()
                return {'service': name, 'success': False, 'reason': 'stopped'}
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        if not future.done():
            future.cancel()
            return {'service': name, 'success': False, 'reason': 'response_timeout'}
        response = future.result()
        details = {}
        if hasattr(response, 'success'):
            success = response.success
        elif hasattr(response, 'return_code'):
            success = response.return_code in (0, 3)
            details['goals_canceling'] = [
                [int(byte) for byte in goal.goal_id.uuid] for goal in response.goals_canceling]
        else:
            success = response.status in (4, 5, 6)  # succeeded, canceled, aborted: all terminal
            details['goal_status'] = response.status
        return {'service': name, 'success': bool(success),
                'elapsed_s': round(time.monotonic()-started, 3), **details}
    except Exception as exc:
        return {'service': name, 'success': False, 'reason': repr(exc)}
    finally:
        if owned_client:
            node.destroy_client(client)


def create_shutdown_clients(node, namespace=''):
    # Establish endpoints while Nav2 starts, instead of discovering new clients
    # between lifecycle transitions during teardown.
    clients = {}
    for name in MANAGERS:
        service = service_name(namespace, name, 'manage_nodes')
        clients[service] = node.create_client(ManageLifecycleNodes, service)
        service = service_name(namespace, name, 'is_active')
        clients[service] = node.create_client(Trigger, service)
    for name in ACTIONS:
        service = service_name(namespace, name, '_action/cancel_goal')
        clients[service] = node.create_client(CancelGoal, service)
        service = service_name(namespace, name, '_action/get_result')
        clients[service] = node.create_client(ACTIONS[name].Impl.GetResultService, service)
    return clients


def shutdown_stack(node, namespace='', timeout=15., clients=None, stop_process=False):
    """Cancel goals, then finalize navigation before localization while ROS is alive."""
    owned_clients = clients is None
    if owned_clients:
        clients = create_shutdown_clients(node, namespace)
    try:
        return _shutdown_stack(node, namespace, timeout, clients, stop_process)
    finally:
        if owned_clients:
            for client in clients.values():
                node.destroy_client(client)


def startup_stack(node, namespace, clients, timeout, stopping):
    """Observe upstream autostart with bounded, retryable read-only requests.

    Real startup lost the explicit STARTUP RPC before any transition ran.
    Retrying that state-changing request can duplicate lifecycle transitions;
    let Nav2 start itself and retry only the idempotent readiness queries.
    """
    deadline = time.monotonic() + timeout
    report = {'success': False, 'managers': [], 'strategy': 'autostart_with_bounded_readiness'}
    for manager in reversed(MANAGERS):
        name = service_name(namespace, manager, 'is_active')
        count = 0
        while not stopping.is_set() and time.monotonic() < deadline:
            count += 1
            result = call_service(node, Trigger, name, Trigger.Request(),
                                  timeout=min(2., max(.01, deadline-time.monotonic())),
                                  discovery_timeout=.2, client=clients[name], stopping=stopping)
            if result['success']:
                report['managers'].append({**result, 'queries': count})
                break
            rclpy.spin_once(node, timeout_sec=.1)
        else:
            report['reason'] = 'stopped' if stopping.is_set() else 'startup_timeout'
            report['last_manager'] = manager
            return report
    report['success'] = True
    return report


def _shutdown_stack(node, namespace, timeout, clients, stop_process):
    report = {'cancellations': [], 'managers': []}
    for action in ACTIONS:
        name = service_name(namespace, action, '_action/cancel_goal')
        # Zero UUID and stamp mean all goals on this action server in this namespace.
        report['cancellations'].append(call_service(
            node, CancelGoal, name, CancelGoal.Request(), timeout=2., discovery_timeout=.5,
            client=clients[name]))
    # CancelGoal only acknowledges the request. Wait for the terminal result
    # while the controller and BT action servers are still active.
    for action, canceled in zip(ACTIONS, report['cancellations']):
        canceled['results'] = []
        for uuid in canceled.get('goals_canceling', []):
            service_type = ACTIONS[action].Impl.GetResultService
            request = service_type.Request()
            request.goal_id.uuid = uuid
            name = service_name(namespace, action, '_action/get_result')
            result = call_service(node, service_type, name, request, timeout=min(timeout, 5.),
                                  client=clients[name])
            canceled['results'].append(result)
            if not result['success']:
                node.get_logger().warning(f'취소 완료 응답 없음, 노드 비활성화로 정지합니다: {name}')
    for manager in MANAGERS:
        node.get_logger().info(f'순서대로 종료: {manager}')
        name = service_name(namespace, manager, 'is_active')
        active = call_service(node, Trigger, name, Trigger.Request(), timeout,
                              client=clients[name])
        # Upstream SHUTDOWN assumes every managed node is active. After a failed
        # startup it otherwise deactivates/cleans up unconfigured nodes again.
        # In separate processes, the normal SIGINT preshutdown callbacks handle
        # each node's actual state. Never claim the standalone helper stopped them.
        if not active['success']:
            inactive = 'reason' not in active
            result = {'service': name, 'success': inactive and stop_process,
                      'reason': 'inactive_process_cleanup' if inactive else active['reason']}
            report['managers'].append(result)
            if result['success']:
                node.get_logger().info(f'{manager}: 현재 상태에 맞춰 프로세스 종료 시 정리합니다')
                continue
            node.get_logger().error(f'정상 종료 확인 실패: {result}')
            break
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        name = service_name(namespace, manager, 'manage_nodes')
        result = call_service(node, ManageLifecycleNodes, name, request, timeout,
                              client=clients[name])
        report['managers'].append(result)
        if not result['success']:
            node.get_logger().error(f'정상 종료 확인 실패: {result}')
            break  # Do not remove localization while navigation may still be active.
    report['success'] = len(report['managers']) == len(MANAGERS) and all(
        result['success'] for result in report['managers'])
    return report


def stop_launch(process, graceful_timeout=12., terminate_timeout=4.):
    """Notify the launch leader once; escalate only if that owned process is alive."""
    signals = []
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        signals.append('SIGINT_launch')
        try:
            process.wait(timeout=graceful_timeout)
        except subprocess.TimeoutExpired:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                signals.append('SIGTERM_group')
                try:
                    process.wait(timeout=terminate_timeout)
                except subprocess.TimeoutExpired:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
                        signals.append('SIGKILL_group')
                        process.wait(timeout=2.)
    return {'exit_code': process.returncode, 'signals': signals,
            'forced': any(s != 'SIGINT_launch' for s in signals)}


def write_report(path, report):
    if path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        temporary.replace(path)


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--namespace', default='')
    parser.add_argument('--shutdown-timeout', type=float, default=15.)
    parser.add_argument('--shutdown-report')
    return parser


def main(argv=None):
    parser = make_parser()
    parser.add_argument('--map', required=True, type=lambda p: str(Path(p).expanduser().resolve()))
    parser.add_argument('--params-file', default=os.path.join(
        get_package_share_directory('plant_observer_gazebo'), 'config', 'nav2_params.yaml'))
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--startup-timeout', type=float, default=90.)
    args = parser.parse_args(argv)
    if not Path(args.map).is_file() or not Path(args.params_file).expanduser().is_file():
        parser.error('지도와 Nav2 설정 파일 경로를 확인하세요')
    if not 0. < args.shutdown_timeout <= 60.:
        parser.error('--shutdown-timeout은 0초 초과, 60초 이하여야 합니다')
    if not 0. < args.startup_timeout <= 300.:
        parser.error('--startup-timeout은 0초 초과, 300초 이하여야 합니다')

    stopping = threading.Event()
    shutdown_phase = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopping.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('plant_nav2_session')  # Wall time for shutdown; never depend on Gazebo clock.
    process = None
    reader = None
    report = {'namespace': args.namespace, 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
              'shutdown_errors': []}
    code = 2
    try:
        clients = create_shutdown_clients(node, args.namespace)
        for manager in MANAGERS:
            client = clients[service_name(args.namespace, manager, 'manage_nodes')]
            if client.wait_for_service(timeout_sec=1.):
                raise RuntimeError('같은 ROS domain/namespace에 Nav2가 이미 실행 중입니다')
        if stopping.is_set():
            report.update(clean=True, requested_stop=True, reason='stopped_before_launch')
            return 0
        # This Jazzy installation crashed while destroying the composed container.
        # Separate processes plus lifecycle shutdown avoid that teardown path.
        command = ['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py', f'map:={args.map}',
                   f'params_file:={Path(args.params_file).expanduser()}',
                   f'use_sim_time:={str(args.use_sim_time).lower()}',
                   'use_composition:=False', 'autostart:=True']
        if args.namespace:
            command += [f'namespace:={args.namespace.strip("/")}', 'use_namespace:=true']
        process = subprocess.Popen(command, start_new_session=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)

        def forward_output():
            for line in process.stdout:
                print(line, end='', flush=True)
                if shutdown_phase.is_set() and any(s in line for s in ('[ERROR]', '[FATAL]', 'SIGSEGV', 'Segmentation Fault')):
                    report['shutdown_errors'].append(line.rstrip())

        reader = threading.Thread(target=forward_output, daemon=True)
        reader.start()
        report['startup'] = startup_stack(node, args.namespace, clients, args.startup_timeout, stopping)
        ready = report['startup']['success']
        node.get_logger().info('Nav2 준비 완료. Ctrl+C를 누르면 순서대로 종료합니다' if ready
                               else 'Nav2 시작 실패: 부분 시작된 노드를 정리합니다')
        while ready and process.poll() is None and not stopping.wait(.1):
            pass
        report['requested_stop'] = stopping.is_set()
        shutdown_phase.set()
        if process.poll() is None:
            report['lifecycle'] = shutdown_stack(
                node, args.namespace, args.shutdown_timeout, clients, stop_process=True)
        else:
            report['lifecycle'] = {'success': False, 'reason': 'launch_exited_before_shutdown'}
        report['process'] = stop_launch(process)
        reader.join(timeout=2.)
        report['clean'] = (report['lifecycle']['success'] and not report['process']['forced']
                           and report['process']['exit_code'] == 0 and not report['shutdown_errors'])
        code = 0 if report['clean'] and (ready or stopping.is_set()) else 2
        node.get_logger().info('Nav2 정상 종료 완료' if code == 0 else 'Nav2 종료 완료: 확인이 필요한 오류가 있습니다')
    except Exception as exc:
        report.update(clean=False, error=repr(exc))
        node.get_logger().error(str(exc))
    finally:
        if process is not None and process.poll() is None:
            report['process'] = stop_launch(process)
        if reader is not None:
            reader.join(timeout=2.)
        try:
            write_report(args.shutdown_report, report)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return code


def shutdown_main(argv=None):
    """Explicit helper for a Nav2 instance launched with the upstream command."""
    parser = make_parser()
    args = parser.parse_args(argv)
    if not 0. < args.shutdown_timeout <= 60.:
        parser.error('--shutdown-timeout은 0초 초과, 60초 이하여야 합니다')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('plant_nav2_shutdown')
    try:
        report = shutdown_stack(node, args.namespace, args.shutdown_timeout)
        write_report(args.shutdown_report, report)
        return 0 if report['success'] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
