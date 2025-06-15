import notify2
import time

def send_notification(title, message):
    notify2.init("Python通知テスト")
    n = notify2.Notification(title, message)
    n.set_timeout(3000)  # ミリ秒（3秒）
    n.show()
    time.sleep(3)  # 通知表示のために少し待つ（オプション）

# 使用例
send_notification("通知タイトル", "これはnotify2で送った通知です。")

