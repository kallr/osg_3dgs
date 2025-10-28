#include "tools.h"


#include "QCoreApplication"

////////////
namespace osg_tools {

	std::string getAppDir()
	{
		QString exeFilePath = QCoreApplication::applicationDirPath(); 
		return exeFilePath.toLocal8Bit().constData();
	}
}

