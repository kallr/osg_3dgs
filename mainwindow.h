#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
class GraphicsWindowQt;

//qt main wnd
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public:
	void createToolBar();
private:
	GraphicsWindowQt* pOSGWnd= nullptr;
public Q_SLOTS:
    void fileOpen();
};
#endif // MAINWINDOW_H
