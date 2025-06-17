#!/usr/bin/env python3
import sys
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton
)
from PySide2.QtCore import QProcess


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Process Controller (via Terminal)")

        # プロセス用オブジェクト（端末ごとにトラッキング）
        self.run_term = QProcess(self)
        self.launch_term = QProcess(self)
        self.bag_term = QProcess(self)

        # ボタン
        self.run_btn = QPushButton("▶ ros2 run talker")
        self.launch_btn = QPushButton("▶ ros2 launch listener")
        self.bag_btn = QPushButton("▶ ros2 bag record -a")

        w = QWidget()
        layout = QVBoxLayout(w)
        layout.addWidget(self.run_btn)
        layout.addWidget(self.launch_btn)
        layout.addWidget(self.bag_btn)
        self.setCentralWidget(w)

        # シグナル接続
        self.run_btn.clicked.connect(self.toggle_run)
        self.launch_btn.clicked.connect(self.toggle_launch)
        self.bag_btn.clicked.connect(self.toggle_bag)

    def _make_cmd(self, ros_cmd: str) -> list:
        """
        端末エミュレータを起動して、bash -ic "...; exec bash" で
        ROS 2 の環境を source してからコマンド実行、終了後にも端末を残す
        """
        # Ubuntu のデフォルト端末切替候補
        # term = "/usr/bin/gnome-terminal"
        # term = "terminator"
        term = "/usr/bin/xterm"
        # 他の環境なら '/usr/bin/x-terminal-emulator' を使ってもよい
        # if not QProcess().executable():  # dummy check; 実際はどちらかが必ずある想定
        #     # term = "/usr/bin/x-terminal-emulator"
        bash_cmd = f"source /opt/ros/humble/setup.bash && {ros_cmd}; exec bash"
        # return [term, "--", "bash", "-ic", bash_cmd]
        return [term, "-e", "bash", "-ic", bash_cmd]

    def toggle_run(self):
        if self.run_term.state() == QProcess.NotRunning:
            args = self._make_cmd("ros2 run demo_nodes_cpp talker")
            self.run_term.start(args[0], args[1:])
            self.run_btn.setText("■ Stop talker")
        else:
            self.run_term.terminate()
            if not self.run_term.waitForFinished(3000):
                self.run_term.kill()
            self.run_btn.setText("▶ ros2 run talker")

    def toggle_launch(self):
        if self.launch_term.state() == QProcess.NotRunning:
            args = self._make_cmd(
                "ros2 launch demo_nodes_cpp listener.launch.py")
            self.launch_term.start(args[0], args[1:])
            self.launch_btn.setText("■ Stop launch")
        else:
            self.launch_term.terminate()
            if not self.launch_term.waitForFinished(3000):
                self.launch_term.kill()
            self.launch_btn.setText("▶ ros2 launch listener")

    def toggle_bag(self):
        if self.bag_term.state() == QProcess.NotRunning:
            args = self._make_cmd("ros2 bag record -a")
            self.bag_term.start(args[0], args[1:])
            self.bag_btn.setText("■ Stop recording")
        else:
            self.bag_term.terminate()
            if not self.bag_term.waitForFinished(3000):
                self.bag_term.kill()
            self.bag_btn.setText("▶ ros2 bag record -a")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(400, 150)
    win.show()
    sys.exit(app.exec_())
