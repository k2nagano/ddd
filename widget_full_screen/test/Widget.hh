#if !defined(WIDGET_HH)
#define WIDGET_HH

#include <QtWidgets>

class Widget : public QWidget
{
	Q_OBJECT
public:
	explicit Widget(QWidget *pParent = nullptr);
	virtual ~Widget();
private slots:
	void toggleFullScreen();

private:
	bool isFullScreenMode;
	QPushButton *toggleButton;
};

#endif // #if !defined(WIDGET_HH)
