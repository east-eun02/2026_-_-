"""B안 차동 구동 노드: /cmd_vel → XL430 두 개(속도 모드), 엔코더 → /odom + TF + /joint_states.

OpenRB-150에는 ROBOTIS usb_to_dynamixel(USB 패스스루) 펌웨어를 올리고,
이 노드가 Dynamixel SDK로 모터와 직접 통신한다(Protocol 2.0 / 57600bps).

모터 ID·방향 부호·전압 검사 기준은 팀 노션 「모터제어 — OpenRB-150 + XL430 실습 매뉴얼」의
실습 코드를 따른다: ID 1=왼쪽, ID 2=오른쪽, LEFT_FORWARD=+1, RIGHT_FORWARD=-1(실물 미검증),
토크 켜기 전 PING·입력 전압(8.5~11.5V)·Hardware Error 0 확인.

USB가 끊기거나 노드가 죽으면 XL430은 마지막 목표 속도로 계속 돈다(2026-10-27 실제 발생).
그래서 모터 자체의 Bus Watchdog을 켜 둔다: 통신이 bus_watchdog_ms 동안 없으면 모터가 스스로 멈춘다.

    ros2 run plant_observer_bringup base_driver --ros-args -p port:=/dev/openrb
"""
import math
import signal
import termios

import rclpy
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# XL430-W250-T 컨트롤 테이블 (Protocol 2.0)
ADDR_FIRMWARE_VERSION = 6
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_BUS_WATCHDOG = 98      # 1~127 × 20ms, 0 = 끔, 255(-1) = 시간 초과로 정지한 상태
ADDR_PROFILE_ACCELERATION = 108
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
OP_VELOCITY = 1
VELOCITY_UNIT_RPM = 0.229   # Goal Velocity 1단위
TICKS_PER_REV = 4096
BUS_WATCHDOG_TRIPPED = 255
MIN_FIRMWARE_FOR_WATCHDOG = 38
# USB를 뽑으면 pyserial이 읽기에서는 SerialException(OSError), 쓰기 flush에서는 termios.error를 낸다
USB_ERRORS = (OSError, termios.error)


def raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def to_signed32(value):
    return value - (1 << 32) if value & (1 << 31) else value


def wheel_rpms(linear, angular, wheel_radius, wheel_separation, max_rpm):
    """차동 구동 역기구학. 한쪽이라도 max_rpm을 넘으면 두 바퀴를 같은 비율로 줄여 곡률을 유지한다."""
    to_rpm = 60.0 / (2.0 * math.pi * wheel_radius)
    left = (linear - angular * wheel_separation / 2.0) * to_rpm
    right = (linear + angular * wheel_separation / 2.0) * to_rpm
    peak = max(abs(left), abs(right))
    if peak > max_rpm:
        left, right = left * max_rpm / peak, right * max_rpm / peak
    return left, right


class BaseDriver(Node):

    def __init__(self):
        super().__init__('base_driver')
        p = self.declare_parameter
        self.port_name = p('port', '/dev/openrb').value
        self.baudrate = p('baudrate', 57600).value
        self.left_id = p('left_id', 1).value
        self.right_id = p('right_id', 2).value
        # 바퀴가 로봇 앞쪽으로 굴러갈 때의 모터 회전 부호 (좌우 대칭 장착이라 한쪽이 반대)
        self.left_sign = p('left_forward_sign', 1).value
        self.right_sign = p('right_forward_sign', -1).value
        self.wheel_radius = p('wheel_radius', 0.033).value
        self.wheel_separation = p('wheel_separation', 0.287).value
        self.max_wheel_rpm = p('max_wheel_rpm', 30.0).value
        self.profile_acceleration = p('profile_acceleration', 10).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.min_voltage = p('min_voltage', 8.5).value
        self.max_voltage = p('max_voltage', 11.5).value
        self.max_comm_failures = p('max_comm_failures', 5).value
        # 20Hz 명령 주기(50ms)보다 넉넉하게. 20ms 단위로 반올림된다
        self.bus_watchdog = max(1, min(127, round(p('bus_watchdog_ms', 250).value / 20)))
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value
        rate = p('rate', 20.0).value
        self.ids = (self.left_id, self.right_id)

        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(2.0)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_VELOCITY, 4)
        self.sync_read = GroupSyncRead(self.port, self.packet, ADDR_PRESENT_POSITION, 4)
        for dxl_id in self.ids:
            self.sync_read.addParam(dxl_id)

        self.torque_on = False
        self.watchdog_armed = False
        self.fault = False
        self.comm_failures = 0
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = None
        self.x = self.y = self.yaw = 0.0
        self.start_ticks = None
        self.last_ticks = None
        self.last_time = None

        self.open_and_prepare()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / rate, self.update)
        self.create_timer(1.0, self.check_health)
        self.get_logger().info(
            f'준비 완료: {self.port_name} ID {self.left_id}(왼쪽)/{self.right_id}(오른쪽), '
            f'바퀴 최대 {self.max_wheel_rpm:.0f}RPM '
            f'(직진 {self.max_wheel_rpm * 2 * math.pi * self.wheel_radius / 60:.2f}m/s)')

    # ---------- 초기화·안전 ----------

    def open_and_prepare(self):
        try:
            opened = self.port.openPort()
        except USB_ERRORS:   # pyserial SerialException: 장치 없음·권한 없음
            opened = False
        if not opened:
            raise RuntimeError(f'{self.port_name}를 열 수 없습니다. OpenRB USB 연결과 dialout 권한을 확인하세요.')
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'통신 속도 {self.baudrate}bps 설정 실패')

        for dxl_id in self.ids:
            _, result, _ = self.packet.ping(self.port, dxl_id)
            if result != COMM_SUCCESS:
                raise RuntimeError(
                    f'ID {dxl_id} PING 실패. 패스스루 펌웨어·모터 배선·ID를 확인하세요.')
            firmware, result, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_FIRMWARE_VERSION)
            if result == COMM_SUCCESS and firmware < MIN_FIRMWARE_FOR_WATCHDOG:
                raise RuntimeError(
                    f'ID {dxl_id} 펌웨어 v{firmware}에는 Bus Watchdog이 없습니다. '
                    f'DYNAMIXEL Wizard 2.0으로 v{MIN_FIRMWARE_FOR_WATCHDOG} 이상으로 업데이트하세요.')
        problem = self.motor_problem()
        if problem:
            raise RuntimeError(problem + ' — 외부 전원(VIN(DXL) 점퍼)과 DC-DC 출력을 확인하세요.')

        # 운전 모드는 토크를 끈 상태에서만 바꿀 수 있다.
        # 지난 실행에서 걸린 Bus Watchdog 정지 상태도 0을 써서 풀어 둔다(전원을 끄기 전까지 남아 있음)
        for dxl_id in self.ids:
            self.write(1, dxl_id, ADDR_TORQUE_ENABLE, 0)
            self.write(1, dxl_id, ADDR_BUS_WATCHDOG, 0)
            self.write(1, dxl_id, ADDR_OPERATING_MODE, OP_VELOCITY)
            self.write(4, dxl_id, ADDR_PROFILE_ACCELERATION, self.profile_acceleration)
            self.write(4, dxl_id, ADDR_GOAL_VELOCITY, 0)
        for dxl_id in self.ids:
            self.write(1, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.torque_on = True

    def write(self, size, dxl_id, addr, value):
        fn = {1: self.packet.write1ByteTxRx, 4: self.packet.write4ByteTxRx}[size]
        result, error = fn(self.port, dxl_id, addr, value & 0xFFFFFFFF if size == 4 else value)
        if result != COMM_SUCCESS or error:
            raise RuntimeError(f'ID {dxl_id} 주소 {addr} 쓰기 실패: '
                               f'{self.packet.getTxRxResult(result)} {self.packet.getRxPacketError(error)}')

    def motor_problem(self):
        """입력 전압·Hardware Error를 읽어 문제가 있으면 설명 문자열을, 정상이면 None을 돌려준다."""
        for dxl_id in self.ids:
            volt, r1, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE)
            hw_err, r2, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_HARDWARE_ERROR)
            if r1 != COMM_SUCCESS or r2 != COMM_SUCCESS:
                return f'ID {dxl_id} 상태 읽기 실패'
            volt /= 10.0
            if not self.min_voltage <= volt <= self.max_voltage:
                return f'ID {dxl_id} 입력 전압 {volt:.1f}V (허용 {self.min_voltage}~{self.max_voltage}V)'
            if hw_err:
                return f'ID {dxl_id} Hardware Error {hw_err}'
        return None

    def enter_fault(self, reason):
        if self.fault:
            return
        self.fault = True
        self.get_logger().error(f'정지: {reason}. 원인을 해결한 뒤 노드를 다시 실행하세요.')
        self.stop_motors()

    def arm_watchdog(self):
        for dxl_id in self.ids:
            self.write(1, dxl_id, ADDR_BUS_WATCHDOG, self.bus_watchdog)
        self.watchdog_armed = True
        self.get_logger().info(f'Bus Watchdog 켜짐: 통신이 {self.bus_watchdog * 20}ms 끊기면 모터가 스스로 멈춥니다')

    def stop_motors(self):
        if not self.torque_on:
            return
        self.torque_on = False
        # USB가 이미 끊겼으면 여기서 보낼 수 없다. 그때는 모터의 Bus Watchdog이 멈춘다
        try:
            for dxl_id in self.ids:
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_GOAL_VELOCITY, 0)
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        except USB_ERRORS:
            pass

    def check_health(self):
        if self.fault:
            return
        try:
            problem = self.motor_problem()
            if problem:
                self.enter_fault(problem)
                return
            if self.watchdog_armed:
                self.rearm_tripped_watchdog()
        except USB_ERRORS as e:
            raise RuntimeError(f'OpenRB USB 연결이 끊겼습니다({e}). 모터는 Bus Watchdog으로 멈춥니다') from e

    def rearm_tripped_watchdog(self):
        """노드가 잠깐 멈춰 Watchdog이 걸렸으면(목표 속도 쓰기가 막힘) 풀고 다시 켠다."""
        for dxl_id in self.ids:
            value, result, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_BUS_WATCHDOG)
            if result == COMM_SUCCESS and value == BUS_WATCHDOG_TRIPPED:
                self.get_logger().warning(f'ID {dxl_id} Bus Watchdog 시간 초과로 정지했었음 — 다시 켭니다')
                self.write(1, dxl_id, ADDR_BUS_WATCHDOG, 0)
                self.write(1, dxl_id, ADDR_BUS_WATCHDOG, self.bus_watchdog)

    def close(self):
        self.stop_motors()
        try:
            self.port.closePort()
        except USB_ERRORS:
            pass

    # ---------- 주행 ----------

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def send_velocity(self, now):
        linear, angular = self.cmd
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            linear = angular = 0.0
        left_rpm, right_rpm = wheel_rpms(
            linear, angular, self.wheel_radius, self.wheel_separation, self.max_wheel_rpm)
        self.sync_write.clearParam()
        for dxl_id, rpm in ((self.left_id, left_rpm * self.left_sign),
                            (self.right_id, right_rpm * self.right_sign)):
            raw = round(rpm / VELOCITY_UNIT_RPM) & 0xFFFFFFFF
            self.sync_write.addParam(dxl_id, list(raw.to_bytes(4, 'little')))
        return self.sync_write.txPacket() == COMM_SUCCESS

    def read_ticks(self):
        if self.sync_read.txRxPacket() != COMM_SUCCESS:
            return None
        ticks = []
        for dxl_id in self.ids:
            if not self.sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 4):
                return None
            ticks.append(to_signed32(self.sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, 4)))
        # 로봇 전진 방향 기준 부호로 맞춘다
        return ticks[0] * self.left_sign, ticks[1] * self.right_sign

    def update(self):
        if self.fault:
            return
        now = self.get_clock().now()
        try:
            ticks = self.read_ticks()
            sent = self.send_velocity(now)
            # 주기 명령이 흐르기 시작한 뒤에 켜야 초기화 중에 Watchdog이 걸리지 않는다
            if sent and not self.watchdog_armed:
                self.arm_watchdog()
        except USB_ERRORS as e:
            raise RuntimeError(f'OpenRB USB 연결이 끊겼습니다({e}). 모터는 Bus Watchdog으로 멈춥니다') from e
        if ticks is None or not sent:
            self.comm_failures += 1
            if self.comm_failures >= self.max_comm_failures:
                self.enter_fault(f'모터 통신 {self.comm_failures}회 연속 실패')
            return
        self.comm_failures = 0

        if self.last_ticks is None:
            self.start_ticks = self.last_ticks = ticks
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        meters_per_tick = 2.0 * math.pi * self.wheel_radius / TICKS_PER_REV
        d_left = (ticks[0] - self.last_ticks[0]) * meters_per_tick
        d_right = (ticks[1] - self.last_ticks[1]) * meters_per_tick
        self.last_ticks, self.last_time = ticks, now

        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self.wheel_separation
        self.x += d_center * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d_center * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))
        self.publish(now, d_center / dt if dt > 0 else 0.0, d_yaw / dt if dt > 0 else 0.0,
                     d_left / dt if dt > 0 else 0.0, d_right / dt if dt > 0 else 0.0)

    def publish(self, now, linear, angular, v_left, v_right):
        stamp = now.to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        rad_per_tick = 2.0 * math.pi / TICKS_PER_REV
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['wheel_left_joint', 'wheel_right_joint']
        joints.position = [(t - s) * rad_per_tick for t, s in zip(self.last_ticks, self.start_ticks)]
        joints.velocity = [v_left / self.wheel_radius, v_right / self.wheel_radius]
        self.joint_pub.publish(joints)


def main():
    rclpy.init()
    # VS Code 터미널 창을 닫으면 SIGHUP이 온다. Ctrl-C와 똑같이 모터를 세우고 끝낸다
    signal.signal(signal.SIGHUP, raise_keyboard_interrupt)
    node = None
    try:
        node = BaseDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        if node is None:
            rclpy.logging.get_logger('base_driver').fatal(str(e))
        else:
            node.get_logger().fatal(str(e))
        raise SystemExit(1)
    finally:
        # launch와 터미널이 Ctrl-C를 두 번 보내도 모터 정지·정리가 끊기지 않게
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
