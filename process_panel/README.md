あなた:
ubuntu22.04 ros2 humbleを使っています
PySide2のqt5 python programで、ボタンをクリックすると ros2 run や ros2 launch, ros2 bag recordで、proccessを実行し、再度そのボタンをクリックするとそのprocessを終了するsample codeを生成してください


ChatGPT:

思考時間: 7 秒
以下は PySide2（Qt5）で「ボタンを押すと ROS 2 のプロセス（ros2 run／ros2 launch／ros2 bag record）を起動し、もう一度押すと同じプロセスを終了する」サンプルです。
※あらかじめターミナルなどで source /opt/ros/humble/setup.bash を実行した環境から起動してください。

python
コピーする
編集する
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
ポイント解説

QProcess を使うことで、外部コマンドの起動・終了が簡単に行えます。

start("bash", ["-c", "..."]) とすることで、事前に ROS 2 の環境を source したうえでコマンドを実行。

toggle メソッド内で state() をチェックし、未起動なら start()、起動中なら terminate()→kill() で確実に終了。

プロセスの標準出力・標準エラーを受け取り、GUI のログ部に表示するようにしています。

このサンプルをもとに、必要なパッケージ名や launch ファイル名、bag のオプションなどを変更してご利用ください。








あなた:
標準出力が同じところに表示されています
これを各プロセスをterminalで実行することはできませんか


ChatGPT:

思考時間: 2 ～ 3 秒
以下のようにすると、バックグラウンドで直接 ros2 コマンドを叩くのではなく、各プロセスを新しい端末ウィンドウ（GNOME Terminal や /usr/bin/x-terminal-emulator）で起動できます。ターミナル上で実行されるので、標準出力・標準エラーもそのまま表示されます。

python
コピーする
編集する
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
        self.run_term    = QProcess(self)
        self.launch_term = QProcess(self)
        self.bag_term    = QProcess(self)

        # ボタン
        self.run_btn    = QPushButton("▶ ros2 run talker")
        self.launch_btn = QPushButton("▶ ros2 launch listener")
        self.bag_btn    = QPushButton("▶ ros2 bag record -a")

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
        term = "/usr/bin/gnome-terminal"
        # 他の環境なら '/usr/bin/x-terminal-emulator' を使ってもよい
        if not QProcess().executable():  # dummy check; 実際はどちらかが必ずある想定
            term = "/usr/bin/x-terminal-emulator"
        bash_cmd = f"source /opt/ros/humble/setup.bash && {ros_cmd}; exec bash"
        return [term, "--", "bash", "-ic", bash_cmd]

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
            args = self._make_cmd("ros2 launch demo_nodes_cpp listener.launch.py")
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
ポイント

/usr/bin/gnome-terminal -- bash -ic "…; exec bash" の形式で起動すると、コマンド実行後も端末が残るのでログを確認できます。

exec bash を末尾に入れることで、コマンド終了後もシェルプロンプトが残ります。

端末を閉じることで、その中で動いている ROS 2 プロセスも同時に終了します（terminate() → kill() は端末プロセス自体を終了させる動作です）。

他の端末エミュレータ（konsole、xterm、tilix など）を使う場合は、_make_cmd の先頭部分を変更してください。








あなた:
QProcessを使わず　subprocessを使うとどうなりますか


ChatGPT:
QProcess の代わりに標準ライブラリの subprocess を使うと、以下のような違い・注意点があります。

1. ノンブロッキング＆イベント駆動の統合
QProcess

Qt のイベントループとシームレスに連携して非同期起動・終了や標準入出力の読み書きができる。

シグナル（readyReadStandardOutput／finished など）で通知を受け取れる。

subprocess

標準ではブロッキング呼び出し（subprocess.run()）か、自前でスレッドや非同期フレームワークを使って非同期化する必要がある。

GUI スレッドで直接プロセスを起動すると、標準出力を待つ間に UI が固まる。

2. 標準出力・標準エラーの取り扱い
QProcess

readAllStandardOutput()／readAllStandardError() で Qt 型のバッファから簡単に読み取れる。

subprocess

stdout=PIPE, stderr=PIPE を指定することでパイプに飛ばせるが、

GUI スレッドで p.stdout.read() するとブロックするので、

別スレッドで読み出し → Qt のシグナルでメインスレッドに渡す、など工夫が必要。

3. プロセスの終了と後始末
QProcess

terminate() → waitForFinished() → kill() といった制御が Qt 標準で手軽に行える。

subprocess

Popen.terminate()／Popen.kill() はあるが、

「終了待ち」は自分で Popen.wait(timeout=…) を呼ぶか、

タイマーでポーリングする必要がある。

4. サンプルコード：subprocess＋QThread を使う場合
python
コピーする
編集する
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
    finished     = Signal(int)

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
            cmd = ["bash", "-ic", "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"]
            self.worker = ProcessWorker(cmd)
            self.worker.stdout_ready.connect(self.log.append)
            self.worker.stderr_ready.connect(lambda t: self.log.append(f"<span style='color:red;'>{t}</span>"))
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
このやり方のポイント
QThread を使わずに Python 標準の threading.Thread

軽量で十分ですが、Qt オブジェクトのスレッド安全性には注意が必要。

stdout=PIPE, text=True で逐次読み込み

ブロックせずデータが来たらシグナルで GUI に渡す。

終了制御

proc.terminate() を呼んで、プロセスが残ったら proc.kill() を追加で行うのがおすすめ。

まとめ
シンプルに書きたい なら subprocess + threading、ただし GUI フリーズ防止や出力の非同期受け取りを自前で実装する必要があります。

Qt と自然に連携したい 場合はやはり QProcess を使う方がコード量も少なく、シグナル/スロットで書けるぶんメンテナンスもしやすいです。









