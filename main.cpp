#include <iostream>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
 

static const char* gl3_VertexShader = {
    "#version 330 core\n"
    "in vec4 osg_Vertex;\n"
    "in vec4 osg_Color;\n"
    "in vec4 osg_MultiTexCoord0;\n"
    "uniform mat4 osg_ModelViewProjectionMatrix;\n"
    "out vec2 texCoord;\n"
    "out vec4 vertexColor;\n"
    "void main(void)\n"
    "{\n"
    "    gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;\n"
    "    texCoord = osg_MultiTexCoord0.xy;\n"
    "    vertexColor = osg_Color; \n"
    "}\n"
};
 
static const char* gl3_FragmentShader = {
    "#version 330 core\n"
    "uniform sampler2D baseTexture;\n"
    "in vec2 texCoord;\n"
    "in vec4 vertexColor;\n"
    "out vec4 color;\n"
    "void main(void)\n"
    "{\n"
    "    color = vertexColor;\n"
    "}\n"
};
 
void setSS(osg::StateSet* stateset){
      
        osg::ref_ptr<osg::Program> program = new osg::Program;
        program->addShader(new osg::Shader(osg::Shader::VERTEX,   gl3_VertexShader));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, gl3_FragmentShader));
        stateset->setAttribute(program.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);        
}
     

int main(int argc ,const char** argv){

   std::cout<<"my osg 3sgs renderer"<<std::endl;

    osgViewer::Viewer viewer;

    if(argc >1 ) { // try to load model
        osg::ref_ptr<osg::Node> pModel = osgDB::readNodeFile(argv[1]);
        if(pModel){
            std::cout<<"load model successed"<<std::endl;
            osg::StateSet* ss = pModel->getOrCreateStateSet();
         //   setSS(ss);
            viewer.setSceneData(pModel);
        } 
        
     //   viewer.getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
    }
    
    viewer.run();

    return 0;
}