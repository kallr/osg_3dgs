#include <iostream>

#include <osgViewer/Viewer>

int main(){
    std::cout<<"my osg 3sgs renderer"<<std::endl;

    osgViewer::Viewer viewer;
    viewer.run();

    return 0;
}