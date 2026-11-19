#include "SparkShaderLibrary.h"

#include <osg/ref_ptr>

#include <fstream>
#include <iostream>
#include <iterator>

namespace SparkShaderLibrary
{

osg::Shader* readShaderTextFile(osg::Shader::Type type, const std::string& filename)
{
    std::string finalPath = filename;
    std::ifstream file(finalPath, std::ios::in | std::ios::binary);
    if (!file) {
        std::string basename = filename.substr(filename.find_last_of("/\\") + 1);
        std::string fallback = "shader/" + basename;
        file.open(fallback, std::ios::in | std::ios::binary);
        if (!file) {
            std::cerr << "Failed to open shader file: " << filename << " and fallback " << fallback << std::endl;
            return nullptr;
        }
        finalPath = fallback;
    }

    osg::ref_ptr<osg::Shader> shader = new osg::Shader(type);
    shader->loadShaderSourceFromFile(finalPath);
    if (shader->getShaderSource().empty()) {
        std::cerr << "Failed to read shader source: " << finalPath << std::endl;
        return nullptr;
    }
    return shader.release();
}

std::string readFileToString(const std::string& filename)
{
    std::ifstream file(filename, std::ios::in | std::ios::binary);
    if (!file) {
        std::string basename = filename.substr(filename.find_last_of("/\\") + 1);
        std::string fallback = "shader/" + basename;
        file.open(fallback, std::ios::in | std::ios::binary);
        if (!file) {
            std::cerr << "readFileToString failed for: " << filename << " and fallback " << fallback << std::endl;
            return "";
        }
    }
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

std::string insertShaderPrelude(const std::string& source, const std::string& prelude)
{
    if (prelude.empty()) return source;

    std::size_t insertPos = source.find("\nvoid main");
    if (insertPos != std::string::npos) {
        std::string combined = source;
        combined.insert(insertPos + 1, prelude + "\n");
        return combined;
    }
    insertPos = source.find("void main");
    if (insertPos == std::string::npos)
        return prelude + "\n" + source;

    std::string combined = source;
    combined.insert(insertPos, prelude + "\n");
    return combined;
}

} // namespace SparkShaderLibrary
