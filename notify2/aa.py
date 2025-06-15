import notify2

def send_error_notification(title, message):
    notify2.init("Error Notifier")
    n = notify2.Notification(title, message)
    n.set_urgency(notify2.URGENCY_CRITICAL)  # ⚠️ ここでエラー通知として設定
    n.set_timeout(notify2.EXPIRES_DEFAULT)
    n.show()

# 使用例
send_error_notification("エラー発生", "ファイルの読み込みに失敗しました。")

