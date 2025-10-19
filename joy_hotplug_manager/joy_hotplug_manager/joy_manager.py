import os
import re
import signal
import subprocess
import time
import yaml
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# pyudev は apt: python3-pyudev
import pyudev


def match_device(mapping_match, dev_props):
    """mapping.match と udev のデバイス情報(dev_props)を突き合わせて一致可否を返す"""
    # 1) name_regex
    if 'name_regex' in mapping_match:
        pat = re.compile(mapping_match['name_regex'])
        name = dev_props.get('NAME') or dev_props.get('ID_MODEL_FROM_DATABASE') or dev_props.get('ID_MODEL') or ''
        if name.startswith('"') and name.endswith('"'):
            name = name[1:-1]
        if pat.search(name or ''):
            return True

    # 2) vid_pid
    if 'vid_pid' in mapping_match:
        want = mapping_match['vid_pid'].lower()
        vid = (dev_props.get('ID_VENDOR_ID') or '').lower()
        pid = (dev_props.get('ID_MODEL_ID') or '').lower()
        if vid and pid and f'{vid}:{pid}' == want:
            return True

    return False


class JoyManager(Node):
    def __init__(self):
        super().__init__('joy_manager')
        self.declare_parameter('mapping_yaml', rclpy.parameter.Parameter.Type.STRING)
        self.declare_parameter('joy_linux_pkg', 'joy_linux')
        self.declare_parameter('joy_linux_exec', 'joy_linux_node')  # 環境により 'joy_linux_node' でない場合あり
        self.declare_parameter('namespace', '')
        self.declare_parameter('extra_ros_args', [])  # ['--ros-args', '-p', 'deadzone:=0.05'] のように追加可

        mapping_yaml = self.get_parameter('mapping_yaml').get_parameter_value().string_value
        if not mapping_yaml:
            mapping_yaml = os.path.join(
                os.getenv('COLCON_PREFIX_PATH', '').split(':')[0] or '/',
                'share', 'joy_hotplug_manager', 'config', 'mappings.yaml'
            )
        with open(mapping_yaml, 'r') as f:
            all_cfg = yaml.safe_load(f)
        self.mappings = all_cfg.get('mappings', [])
        self.fallback = all_cfg.get('fallback', {'id': 'generic'})

        self.active_mapping_id = None
        self.joy_proc = None
        self.proc_lock = threading.Lock()

        self.pub_active = self.create_publisher(String, 'active_joystick', 10)

        # 既存接続のスキャンと起動
        self.select_and_start()

        # udev 監視
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

    # udev 監視ループ
    def monitor_loop(self):
        ctx = pyudev.Context()
        mon = pyudev.Monitor.from_netlink(ctx)
        mon.filter_by(subsystem='input')

        for dev in iter(mon.poll, None):
            # ジョイスティックのみ注目: ID_INPUT_JOYSTICK=1 or "js"デバイス
            if dev is None:
                continue
            if dev.action not in ('add', 'remove', 'change'):
                continue

            props = dict(dev.properties)
            is_js = props.get('ID_INPUT_JOYSTICK') == '1'
            if not is_js and not (dev.device_node and 'js' in (dev.device_node or '')):
                continue

            self.get_logger().info(f'UDEV event: {dev.action} {dev.device_node} {props.get("NAME","")}')
            # 少し待ってから再探索（デバイスノード確定待ち）
            time.sleep(0.4)
            self.select_and_start()

    def list_connected_joysticks(self):
        """現在接続されているジョイスティックの udev properties を列挙"""
        ctx = pyudev.Context()
        out = []
        for dev in ctx.list_devices(subsystem='input'):
            props = dict(dev.properties)
            if props.get('ID_INPUT_JOYSTICK') == '1' or (dev.device_node and 'js' in dev.device_node):
                out.append((dev.device_node, props))
        return out

    def select_and_start(self):
        """接続中ジョイスティックと mappings を突き合わせ、最初にマッチしたものを選び joy_linux を再起動"""
        js_list = self.list_connected_joysticks()

        chosen = None
        for m in self.mappings:
            for node, props in js_list:
                if match_device(m.get('match', {}), props):
                    chosen = (m, node, props)
                    break
            if chosen:
                break

        if not chosen:
            # 見つからなければフォールバック
            m = self.fallback
            node = None
            props = {}
            self.start_or_switch(m, node, props)
            return

        m, node, props = chosen
        self.start_or_switch(m, node, props)

    def start_or_switch(self, mapping, device_node, props):
        mapping_id = mapping.get('id', 'unknown')
        # 既にこのマッピングなら何もしない（joy_linux devが変わるケースは再起動）
        need_restart = (mapping_id != self.active_mapping_id)

        # joy_linux のパラメータ決定
        joy_args = []
        jl = mapping.get('joy_linux', {})
        device_name = jl.get('device_name')
        dev_fixed = jl.get('dev')

        # 優先: device_name 指定、次に dev 指定、最後に検出された device_node
        if device_name:
            # 多くの環境では '-p device_name:=<name>' で対象選択可能
            joy_args += ['--ros-args', '-p', f'device_name:={device_name}']
        elif dev_fixed:
            joy_args += ['--ros-args', '-p', f'dev:={dev_fixed}']
        elif device_node:
            joy_args += ['--ros-args', '-p', f'dev:={device_node}']
        else:
            # 何もない場合は joy_linux のデフォルト探索に委ねる
            pass

        extra = self.get_parameter('extra_ros_args').get_parameter_value().string_array_value
        if extra:
            joy_args += list(extra)

        with self.proc_lock:
            if (not need_restart) and self.joy_proc and (self.joy_proc.poll() is None):
                # 既に同マッピングで起動済み。dev が変わっていれば再起動する。
                # 簡易対応：常に再起動でも OK。
                need_restart = True

            if need_restart:
                self.stop_joy_linux_locked()
                self.start_joy_linux_locked(joy_args)

                self.active_mapping_id = mapping_id
                msg = String()
                msg.data = mapping_id
                self.pub_active.publish(msg)
                self.get_logger().info(f'Active joystick mapping: {mapping_id}')

    def start_joy_linux_locked(self, joy_args):
        pkg = self.get_parameter('joy_linux_pkg').get_parameter_value().string_value
        exe = self.get_parameter('joy_linux_exec').get_parameter_value().string_value
        ns  = self.get_parameter('namespace').get_parameter_value().string_value
        cmd = ['ros2', 'run', pkg, exe]
        if ns:
            cmd = ['ros2', 'run', pkg, exe, '--ros-args', '-r', f'__ns:={ns}']
            # 以降の --ros-args は続けて渡して問題ありません
        cmd += joy_args
        self.get_logger().info(f'Starting joy_linux: {" ".join(cmd)}')
        # 新しいセッションで起動（kill しやすい）
        self.joy_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

    def stop_joy_linux_locked(self):
        if self.joy_proc and (self.joy_proc.poll() is None):
            self.get_logger().info('Stopping existing joy_linux...')
            try:
                # セッションごと SIGTERM
                os.killpg(os.getpgid(self.joy_proc.pid), signal.SIGTERM)
                try:
                    self.joy_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.joy_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.joy_proc = None

    def destroy_node(self):
        with self.proc_lock:
            self.stop_joy_linux_locked()
        super().destroy_node()


def main():
    rclpy.init()
    node = JoyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
