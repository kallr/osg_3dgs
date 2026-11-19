#include "SparkOsgCallbacks.h"

#include <osg/GLExtensions>
#include <osg/State>

MemoryBarrierDrawable::MemoryBarrierDrawable()
    : _barriers(GL_SHADER_STORAGE_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT)
{
    setCullingActive(false);
    setUseDisplayList(false);
}

MemoryBarrierDrawable::MemoryBarrierDrawable(GLbitfield barriers)
    : _barriers(barriers)
{
    setCullingActive(false);
    setUseDisplayList(false);
}

MemoryBarrierDrawable::MemoryBarrierDrawable(const MemoryBarrierDrawable& rhs, const osg::CopyOp& copyop)
    : osg::Drawable(rhs, copyop), _barriers(rhs._barriers)
{
}

osg::BoundingBox MemoryBarrierDrawable::computeBoundingBox() const
{
    return osg::BoundingBox();
}

void MemoryBarrierDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State* state = renderInfo.getState();
    if (!state) return;
    const osg::GLExtensions* ext = state->get<osg::GLExtensions>();
    if (ext && ext->glMemoryBarrier)
        ext->glMemoryBarrier(_barriers);
}
