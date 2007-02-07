#ifndef SOFA_COMPONENTS_GL_RAII_H
#define SOFA_COMPONENTS_GL_RAII_H

#include <GL/gl.h>
/* Opengl Resource Acquisition Is Initialisation */
/* with this tool, we know at any moment what is the state of the openGL machine */

namespace Sofa
{

namespace Components
{

namespace GL
{

template <GLenum Flag>
struct Enable
{
    GLboolean state;

    Enable ()
    {
        state = glIsEnabled(Flag);
        // state contains the state of the Flag
        // if this flag is activated, we haven't to reactivate it
        if (!state)
            glEnable (Flag);
    };

    ~Enable ()
    {
        if (!state)
            glDisable (Flag);
    };
};

template <GLenum Flag>
struct Disable
{

    GLboolean state;

    Disable ()
    {
        state = glIsEnabled(Flag);
        // state contains the state of the Flag
        // if this flag is activated, we haven't to reactivate it
        if (state)
            glDisable (Flag);
    };

    ~Disable ()
    {
        if (state)
            glEnable (Flag);
    };
};

} // namespace GL

} // namespace Components

} // namespace Sofa

#endif
