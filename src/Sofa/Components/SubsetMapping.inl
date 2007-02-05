#ifndef SOFA_COMPONENTS_SUBSETMAPPING_INL
#define SOFA_COMPONENTS_SUBSETMAPPING_INL

#include "SubsetMapping.h"

namespace Sofa
{

namespace Components
{


template <class BaseMapping>
SubsetMapping<BaseMapping>::SubsetMapping(In* from, Out* to)
    : Inherit(from, to)
    , f_indices( this->dataField(&f_indices, "indices", "list of input indices"))
    , f_first( this->dataField(&f_first, -1, "first", "first index (use if indices are sequential)"))
    , f_last( this->dataField(&f_last, -1, "last", "last index (use if indices are sequential)"))
{
}

template <class BaseMapping>
SubsetMapping<BaseMapping>::~SubsetMapping()
{
}

template <class BaseMapping>
void SubsetMapping<BaseMapping>::init()
{
    unsigned int inSize = this->fromModel->getX()->size();
    if (f_indices.getValue().empty() && f_first.getValue() != -1)
    {
        IndexArray& indices = *f_indices.beginEdit();
        unsigned int first = (unsigned int)f_first.getValue();
        unsigned int last = (unsigned int)f_last.getValue();
        if (first >= inSize)
            first = 0;
        if (last >= inSize)
            last = inSize-1;
        indices.resize(last-first+1);
        for (unsigned int i=0; i<indices.size(); ++i)
            indices[i] = first+i;
        f_indices.endEdit();
    }
}

template <class BaseMapping>
void SubsetMapping<BaseMapping>::apply( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const IndexArray& indices = f_indices.getValue();
    //out.resize(in.size());
    for(unsigned int i = 0; i < out.size(); ++i)
    {
        out[i] = in[ indices[i] ];
    }
}

template <class BaseMapping>
void SubsetMapping<BaseMapping>::applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
    const IndexArray& indices = f_indices.getValue();
    //out.resize(in.size());
    for(unsigned int i = 0; i < out.size(); ++i)
    {
        out[i] = in[ indices[i] ];
    }
}

template <class BaseMapping>
void SubsetMapping<BaseMapping>::applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const IndexArray& indices = f_indices.getValue();
    for(unsigned int i = 0; i < in.size(); ++i)
    {
        out[indices[i]] += in[ i ];
    }
}


} // namespace Components

} // namespace Sofa

#endif
