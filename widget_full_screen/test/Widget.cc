#include "Widget.hh"

// explicit
Widget::Widget(QWidget *pParent) : QWidget(pParent), isFullScreenMode(false)
{
    toggleButton = new QPushButton("切り替え: FullScreen", this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(toggleButton);
    setLayout(layout);

    connect(toggleButton, &QPushButton::clicked, this, &Widget::toggleFullScreen);
}

// virtual
Widget::~Widget()
{
}

void Widget::toggleFullScreen()
{
    if (isFullScreenMode)
    {
        showNormal(); // 通常表示に戻す
        toggleButton->setText("切り替え: FullScreen");
    }
    else
    {
        showFullScreen(); // フルスクリーンにする
        toggleButton->setText("切り替え: 通常表示");
    }
    isFullScreenMode = !isFullScreenMode;
}