#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
class GraphicsWindowQt;
class QAction;
class QLabel;
class QDragEnterEvent;
class QDropEvent;

//qt main wnd
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
	void loadFile(const QString& fileName);
	bool captureFramebuffer(const QString& fileName);
protected:
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void changeEvent(QEvent* event) override;
private:
    void createActions();
    void fileOpen();
    void saveModelAs();
    void saveScreenshot();
    void updateViewStatus();
    GraphicsWindowQt* pOSGWnd = nullptr;
    QLabel* _viewStatus = nullptr;
    QAction* _fitAction = nullptr;
    QAction* _boundsAction = nullptr;
    QAction* _resetAction = nullptr;
    QAction* _captureAction = nullptr;
    QAction* _saveModelAction = nullptr;
    QAction* _fullscreenAction = nullptr;
    QAction* _selectAction = nullptr;
    QAction* _clearSelectionAction = nullptr;
    QAction* _deletePointsAction = nullptr;
    QAction* _undoDeletionAction = nullptr;
    QString _currentFile;
};
#endif // MAINWINDOW_H
