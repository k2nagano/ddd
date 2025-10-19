import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class JoyMapper(Node):
    def __init__(self):
        super().__init__('joy_mapper')
        self.declare_parameter(
            'mapping_yaml', rclpy.parameter.Parameter.Type.STRING)

        mapping_yaml = self.get_parameter(
            'mapping_yaml').get_parameter_value().string_value
        if not mapping_yaml:
            mapping_yaml = os.path.join(
                os.getenv('COLCON_PREFIX_PATH', '').split(':')[0] or '/',
                'share', 'joy_hotplug_manager', 'config', 'mappings.yaml'
            )
        with open(mapping_yaml, 'r') as f:
            all_cfg = yaml.safe_load(f)

        self.mapping_table = {m['id']: m for m in all_cfg.get('mappings', [])}
        fb = all_cfg.get('fallback', {'id': 'generic'})
        self.fallback = fb
        self.active_id = fb.get('id', 'generic')

        self.sub_joy = self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.sub_active = self.create_subscription(
            String, 'active_joystick', self.on_active, 10)

        self.pub_cmd = self.create_publisher(String, 'joy_cmd', 10)
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(
            f'JoyMapper ready. active mapping: {self.active_id}')

    def on_active(self, msg: String):
        self.active_id = msg.data
        self.get_logger().info(f'Switched mapping -> {self.active_id}')

    def get_active_mapping(self):
        return self.mapping_table.get(self.active_id, self.fallback)

    def on_joy(self, joy: Joy):
        m = self.get_active_mapping()
        buttons = m.get('buttons', {})
        axes = m.get('axes', {})
        twist_cfg = m.get('twist', {})

        # 例: ARM/DISARM/MODE ボタンを String に
        for name, idx in buttons.items():
            try:
                if idx < len(joy.buttons) and joy.buttons[idx] == 1:
                    s = String()
                    s.data = name
                    self.pub_cmd.publish(s)
            except Exception:
                pass

        # 例: 指定軸を Twist に
        axis_map = {}
        if axes:
            inv = set(axes.get('invert', []))

            def get_axis(label):
                i = axes.get(label, None)
                if i is None or i >= len(joy.axes):
                    return 0.0
                v = joy.axes[i]
                if label in inv:
                    v = -v
                return float(v)

            # 汎用的に取り出し
            for k in ['LX', 'LY', 'RX', 'RY', 'LT', 'RT']:
                if k in axes:
                    axis_map[k] = get_axis(k)

        if twist_cfg:
            tw = Twist()
            lin_label = twist_cfg.get('linear_x')
            ang_label = twist_cfg.get('angular_z')
            scale = twist_cfg.get('scale', {})
            if lin_label and lin_label in axis_map:
                tw.linear.x = axis_map[lin_label] * \
                    float(scale.get('linear_x', 1.0))
            if ang_label and ang_label in axis_map:
                tw.angular.z = axis_map[ang_label] * \
                    float(scale.get('angular_z', 1.0))
            self.pub_twist.publish(tw)


def main():
    rclpy.init()
    node = JoyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
