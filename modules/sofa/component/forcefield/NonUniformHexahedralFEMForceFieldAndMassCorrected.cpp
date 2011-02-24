#include "NonUniformHexahedralFEMForceFieldAndMassCorrected.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/Vec.h>


namespace sofa{
  namespace component{
    namespace forcefield{
      using namespace sofa::defaulttype;

SOFA_DECL_CLASS(NonUniformHexahedralFEMForceFieldAndMassCorrected)
// Register in the Factory
int NonUniformHexahedralFEMForceFieldAndMassCorrectedClass = core::RegisterObject("Non uniform Hexahedral finite elements")
#ifndef SOFA_FLOAT
.add< NonUniformHexahedralFEMForceFieldAndMassCorrected<Vec3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
.add< NonUniformHexahedralFEMForceFieldAndMassCorrected<Vec3fTypes> >()
#endif
.addAlias("NonUniformHexaFEMMassCorrected")
;

#ifndef SOFA_FLOAT
template class NonUniformHexahedralFEMForceFieldAndMassCorrected<Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class NonUniformHexahedralFEMForceFieldAndMassCorrected<Vec3fTypes>;
#endif
    }
  }
}

