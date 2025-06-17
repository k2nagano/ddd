import sys
import subprocess
from threading import Thread
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton, QTextEdit
)
from PySide2.QtCore import Signal, QObject


class ProcessWorker(QObject):
    stdout_ready = Signal(str)
    stderr_ready = Signal(str)
    finished = Signal(int)

    def __init__(self, cmd_list):
        super().__init__()
        self._cmd = cmd_list
        self._proc = None

    def start(self):
        # 別スレッドでプロセス起動＆出力読み出し
        def run():
            self._proc = subprocess.Popen(
                self._cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=1,
                text=True,
            )
            # 非同期に標準出力／エラーを逐次読み
            for line in self._proc.stdout:
                self.stdout_ready.emit(line.rstrip())
            for line in self._proc.stderr:
                self.stderr_ready.emit(line.rstrip())
            code = self._proc.wait()
            self.finished.emit(code)

        Thread(target=run, daemon=True).start()

    def stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("subprocess + QThread Sample")

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.btn = QPushButton("▶ Start ros2 run")
        self.btn.clicked.connect(self.on_button)

        w = QWidget()
        layout = QVBoxLayout(w)
        layout.addWidget(self.btn)
        layout.addWidget(self.log)
        self.setCentralWidget(w)

        self.worker = None
        self.is_running = False

    def on_button(self):
        if not self.is_running:
            # コマンドをリストで指定
            cmd = [
                "bash", "-ic", "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"]
            self.worker = ProcessWorker(cmd)
            self.worker.stdout_ready.connect(self.log.append)
            self.worker.stderr_ready.connect(lambda t: self.log.append(
                f"<span style='color:red;'>{t}</span>"))
            self.worker.finished.connect(self.on_finished)
            self.worker.start()

            self.btn.setText("■ Stop")
            self.is_running = True
        else:
            self.worker.stop()
            self.btn.setEnabled(False)  # 終了完了まで重複クリック防止

    def on_finished(self, return_code):
        self.log.append(f"<i>Process exited with {return_code}</i>")
        self.btn.setText("▶ Start ros2 run")
        self.btn.setEnabled(True)
        self.is_running = False


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(600, 400)
    win.show()
    sys.exit(app.exec_())
