## 2024-04-25
# 使用osg渲染3dgs ,支持ply格式， splat格式
# require glm qt5 osg>=3.6.5


# 使用方法：
# 1: 设置相机
# 2：创建GaussianDrawObj对象，传入文件路径
# 3：调用GaussianDrawObj的getNode()函数返回osg节点， 并将节点加入场景中即可

# 示例：
# std::string file = "C:\\Users\\Administrator\\Desktop\\1.splat";
# GaussianDrawObj*pObj = new GaussianDrawObj(file);
# osg::ref_ptr<osg::Node> pGsModul  =  pObj->getNode();
# getRoot()->addChild(pGsModul);
