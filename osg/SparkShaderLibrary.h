#ifndef SPARK_SHADER_LIBRARY_H
#define SPARK_SHADER_LIBRARY_H

#include <osg/Shader>

#include <string>

namespace SparkShaderLibrary
{
    osg::Shader* readShaderTextFile(osg::Shader::Type type, const std::string& filename);
    std::string readFileToString(const std::string& filename);
    std::string insertShaderPrelude(const std::string& source, const std::string& prelude);
}

#endif
