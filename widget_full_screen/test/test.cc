#include <QtWidgets>
#include "Widget.hh"

int main(int argc, char *argv[])
{
   QApplication a(argc, argv);
   Widget w;
   w.show();
   a.exec();
   return 0;
}
