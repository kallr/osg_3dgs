#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
class GraphicsWindowQt;
class MI_TreeItem;
class QTreeWidgetItem;
class QTreeWidget;

//qt main wnd
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public:
 private:
    GraphicsWindowQt* pOSGWnd= nullptr;
 
 
     void createToolBar();

public Q_SLOTS:
    void fileNew();
    void fileOpen();
};
#endif // MAINWINDOW_H
