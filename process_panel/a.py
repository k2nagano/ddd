#!/usr/bin/env python3
import sys
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton, QTextEdit
)
from PySide2.QtCore import QProcess

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Process Controller")

        # プロセス用オブジェクトを用意
        self.run_proc    = QProcess(self)
        self.launch_proc = QProcess(self)
        self.bag_proc    = QProcess(self)

        # ボタンとログ表示
        self.run_btn    = QPushButton("▶ ros2 run talker")
        self.launch_btn = QPushButton("▶ ros2 launch listener")
        self.bag_btn    = QPushButton("▶ ros2 bag record -a")
        self.log = QTextEdit()
        self.log.setReadOnly(True)

        # レイアウト
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.addWidget(self.run_btn)
        layout.addWidget(self.launch_btn)
        layout.addWidget(self.bag_btn)
        layout.addWidget(self.log)
        self.setCentralWidget(w)

        # シグナル接続
        self.run_btn.clicked.connect(self.toggle_run)
        self.launch_btn.clicked.connect(self.toggle_launch)
        self.bag_btn.clicked.connect(self.toggle_bag)

        # プロセス出力をログに流す
        for proc in (self.run_proc, self.launch_proc, self.bag_proc):
            proc.readyReadStandardOutput.connect(self.handle_stdout)
            proc.readyReadStandardError.connect(self.handle_stderr)

    def handle_stdout(self):
        proc = self.sender()
        text = proc.readAllStandardOutput().data().decode()
        self.log.append(text)

    def handle_stderr(self):
        proc = self.sender()
        text = proc.readAllStandardError().data().decode()
        self.log.append(f"<span style='color:red;'>{text}</span>")

    def toggle_run(self):
        if self.run_proc.state() == QProcess.NotRunning:
            cmd = "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"
            # bash -c でまとめて実行
            self.run_proc.start("bash", ["-c", cmd])
            self.run_btn.setText("■ Stop talker")
        else:
            self.run_proc.terminate()
            if not self.run_proc.waitForFinished(3000):
                self.run_proc.kill()
            self.run_btn.setText("▶ ros2 run talker")

    def toggle_launch(self):
        if self.launch_proc.state() == QProcess.NotRunning:
            cmd = "source /opt/ros/humble/setup.bash && ros2 launch demo_nodes_cpp listener.launch.py"
            self.launch_proc.start("bash", ["-c", cmd])
            self.launch_btn.setText("■ Stop launch")
        else:
            self.launch_proc.terminate()
            if not self.launch_proc.waitForFinished(3000):
                self.launch_proc.kill()
            self.launch_btn.setText("▶ ros2 launch listener")

    def toggle_bag(self):
        if self.bag_proc.state() == QProcess.NotRunning:
            cmd = "source /opt/ros/humble/setup.bash && ros2 bag record -a"
            self.bag_proc.start("bash", ["-c", cmd])
            self.bag_btn.setText("■ Stop recording")
        else:
            self.bag_proc.terminate()
            if not self.bag_proc.waitForFinished(3000):
                self.bag_proc.kill()
            self.bag_btn.setText("▶ ros2 bag record -a")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(600, 400)
    window.show()
    sys.exit(app.exec_())

