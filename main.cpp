#include "mainwindow.h"

#include <QApplication>
#include <QTimer>
#include <QString>
#include <QByteArray>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <iostream>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
	w.resize(1200, 800);
	if (qEnvironmentVariableIntValue("OSG_3DGS_CAPTURE_LOCK_SIZE") != 0)
		w.setFixedSize(1200, 800);
	if (argc > 1) {
		QString fileName = QString::fromLocal8Bit(argv[1]);
		QTimer::singleShot(0, &w, [fileName, &w]() { w.loadFile(fileName); });
	}

	QByteArray capturePath = qgetenv("OSG_3DGS_CAPTURE_PATH");
	if (!capturePath.isEmpty()) {
		int captureDelayMs = qEnvironmentVariableIntValue("OSG_3DGS_CAPTURE_DELAY_MS");
		if (captureDelayMs <= 0)
			captureDelayMs = 8000;
		const int sequenceMs = qEnvironmentVariableIntValue("OSG_3DGS_CAPTURE_SEQUENCE_MS");
		if (sequenceMs > 0) {
			// Optional visual regression capture. Readback/encoding affects timing,
			// so leave this disabled for performance measurements.
			const QFileInfo destination(QString::fromLocal8Bit(capturePath.constData()));
			const QString directory = destination.absolutePath() + "/" + destination.completeBaseName() + "-frames";
			if (QDir().mkpath(directory)) {
				auto* sequence = new QTimer(&w);
				QElapsedTimer clock;
				clock.start();
				QObject::connect(sequence, &QTimer::timeout, &w, [&w, sequence, directory, clock]() {
					const QString frame = directory + "/" + QString::number(clock.elapsed()).rightJustified(8, '0') + ".bmp";
					if (!w.captureFramebuffer(frame)) {
						sequence->stop();
						std::cerr << "Framebuffer sequence capture failed" << std::endl;
					}
				});
				sequence->start(qBound(33, sequenceMs, 5000));
			}
		}
		QTimer::singleShot(captureDelayMs, &w, [&w, capturePath]() {
			QString path = QString::fromLocal8Bit(capturePath.constData());
			bool saved = w.captureFramebuffer(path);
			std::cout << "OSG_3DGS_CAPTURE_PATH=" << path.toLocal8Bit().constData()
			          << ", saved=" << (saved ? 1 : 0) << std::endl;
			QApplication::quit();
		});
	}
    return a.exec();
}
