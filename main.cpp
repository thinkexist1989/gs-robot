#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL); // 禁用缓冲区，所有printf即时输出
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(":/icons/pictures/app_icon.png"));
    MainWindow w;
    w.show();
    return a.exec();
}
