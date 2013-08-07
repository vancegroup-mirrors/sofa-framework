#include "labelboximagetoolbox.h"

#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace engine
{

SOFA_DECL_CLASS(LabelBoxImageToolBox)

int LabelBoxImageToolBox_Class = core::RegisterObject("LabelBoxImageToolBox")
.add< LabelBoxImageToolBox >()
.addLicense("LGPL")
.addAuthor("Vincent Majorczyk");

}}}
