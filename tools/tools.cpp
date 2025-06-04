#include "tools.h"

#include "osg/Geode"
#include "osg/Group"

#include "QCoreApplication"

////////////
namespace osg_tools {

	std::string getAppDir()
	{
		QString exeFilePath = QCoreApplication::applicationDirPath(); 
		return exeFilePath.toLocal8Bit().constData();
	}
}

