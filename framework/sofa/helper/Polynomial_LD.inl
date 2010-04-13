/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_HELPER_POLYNOMIAL_LD_INL
#define SOFA_HELPER_POLYNOMIAL_LD_INL

#include <sofa/helper/Polynomial_LD.h>
#include <sstream>
#include <iterator>


namespace sofa
{
namespace helper
{


using namespace sofa::defaulttype;
using namespace std;
/**
 * Tools used in FEM computing
 */

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N>::Monomial_LD()
{
    coef=(Real) 0;
    for(unsigned int i=0; i<N; i++)
    {
        powers[i]=0;
        ostringstream oss; oss << 'l' << i ;
        variables[i]=oss.str();
    }
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N>::Monomial_LD(const Monomial_LD<Real,N> & b)
{
    coef=b.coef;
    for(unsigned int i=0; i<N; i++)
    {
        powers[i]=b.powers[i];
        variables[i]=b.variables[i];
    }
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N>::Monomial_LD(const Real & m_coef, ...)
{
    coef=m_coef;
    va_list vl;
    va_start(vl,m_coef);
    for(unsigned int i=0; i<N; i++)
    {
        powers[i]=va_arg(vl,int);
        ostringstream oss; oss << 'l' << i ;
        variables[i]=oss.str();
    }
    va_end(vl);
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N>& Monomial_LD<Real,N>::operator=(const Monomial_LD<Real,N> & b)
{
    coef=b.coef;
    for( unsigned int i=0; i<N; i++)
    {
        powers[i]=b.powers[i];
        variables[i]=b.variables[i];
    }
    return *this;
}
////////////////////////////////

template<typename Real, unsigned int N>
void Monomial_LD<Real,N>::Set(const Real & m_coef, ...)
{
    coef=m_coef;
    va_list vl;
    va_start(vl,m_coef);
    for(unsigned int i=0; i<N; i++)
    {
        powers[i]=va_arg(vl,int);
    }
    va_end(vl);
}
////////////////////////////////
template<typename Real, unsigned int N>
int Monomial_LD<Real,N>::degree()
{
    int degree=0;
    for(unsigned int i=0; i<N; i++)
        degree+=powers[i];
    return degree;
}
////////////////////////////////
template<typename Real, unsigned int N>
bool Monomial_LD<Real,N>::operator==(const Monomial_LD<Real,N> & b) const
{
    bool compare=true;
    if ((coef != b.coef) || (variables.size() != b.variables.size())) compare=false;
    for(unsigned int i=0; i<N; i++)
    {
        if (powers[i] != b.powers[i]) compare=false;
    }
    return compare;
}
////////////////////////////////
template<typename Real, unsigned int N>
bool Monomial_LD<Real,N>::isSamePowers(const Monomial_LD<Real,N> & b) const
{
    bool compare=true;
    if ( variables.size() != b.variables.size() ) compare=false;
    for(unsigned int i=0; i<N; i++)
    {
        if (powers[i] != b.powers[i]) compare=false;
    }
    return compare;
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N> & Monomial_LD<Real,N>::operator+=(const Monomial_LD<Real,N> & b)
{
    if (this->isSamePowers(b))
    {
        this->coef+=b.coef;
    }
    else
    {
        cout<<"WARNING : "<<(*this)<<" + "<<b
            <<"   Not permissed with different powers"<<endl;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N> & Monomial_LD<Real,N>::operator-=(const Monomial_LD<Real,N> & b)
{
    if (this->isSamePowers(b))
    {
        this->coef-=b.coef;
    }
    else
    {
        cout<<"WARNING : "<<(*this)<<" - "<<b
            <<"   Not permissed with different powers"<<endl;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N> & Monomial_LD<Real,N>::operator*=(const Monomial_LD<Real,N> & b)
{
    coef*=b.coef;
    for(unsigned int i=0; i<N; i++)
        powers[i] += b.powers[i];
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Real Monomial_LD<Real,N>::operator()(const vector<Real> & x) const
{
    if (x.size()!= N) cout<<"WARNING : value assigned to the monome has not the good number of variable"<<endl;
    Real value= coef;
    for( unsigned int i=0; i<N; i++)
    {
        value*=(Real) pow(x[i],powers[i]);
    }
    return value;
}
////////////////////////////////
template<typename Real, unsigned int N>
Real Monomial_LD<Real,N>::operator()(const RNpoint & x) const
{
    if (x.size()!= N) cout<<"WARNING : value assigned to the monome has not the good number of variable"<<endl;
    Real value= coef;
    for( unsigned int i=0; i<N; i++)
    {
        value*=(Real) pow(x[i],powers[i]);
    }
    return value;
}
////////////////////////////////
template<typename Real, unsigned int N>
Real Monomial_LD<Real,N>::operator()(const vector<Real> & x,unsigned int ideriv) const
{
    //assert( (x.size()==N) && (ideriv < N) );
    if (x.size()!= N) cout<<"WARNING : value assigned to the monome has not the good number of variable"<<endl;
    Real value= coef;

    if (ideriv >= N)
    {
        cout<<"WARNING : "<<ideriv<<"-th derivative couldn't take place for the monomial of:"<<N<<"-variables"<<endl
            <<(*this)<<endl
            <<"CONDITION: id_derivative = { 0,1... (NbVariable-1) }"<<endl<<endl;
    }
    else
    {
        if(ideriv==0)
        {
            value=this->operator()(x);
        }
        else
        {
            for(unsigned int i=0; i<N; i++)
            {
                if (i==ideriv)
                {
                    value*=(Real) powers[i];//derivate
                    value*=(Real) pow(x[i],(powers[i]-1));
                }
                else
                {
                    value*=(Real) pow(x[i],powers[i]);
                }
            }
        }
    }
    return value;
}
////////////////////////////////
template<typename Real, unsigned int N>
Real Monomial_LD<Real,N>::operator()(const RNpoint & x,unsigned int ideriv) const
{
    //assert( (x.size()==N) && (ideriv < N) );
    if (x.size()!= N) cout<<"WARNING : value assigned to the monome has not the good number of variable"<<endl;
    Real value= coef;

    if (ideriv >= N)
    {
        cout<<"WARNING : "<<ideriv<<"-th derivative couldn't take place for the monomial of:"<<N<<"-variables"<<endl
            <<(*this)<<endl
            <<"CONDITION: id_derivative = { 0,1... (NbVariable-1) }"<<endl<<endl;
    }
    else
    {
        if(ideriv==0)
        {
            value=this->operator()(x);
        }
        else
        {
            for(unsigned int i=0; i<N; i++)
            {
                if (i==ideriv)
                {
                    value*=(Real) powers[i];//derivate
                    value*=(Real) pow(x[i],(powers[i]-1));
                }
                else
                {
                    value*=(Real) pow(x[i],powers[i]);
                }
            }
        }
    }
    return value;
}
////////////////////////////////
template<typename Real, unsigned int N>
Monomial_LD<Real,N> Monomial_LD<Real,N>::d(const unsigned int & ideriv) const
{
    Monomial_LD<Real,N> r(*this);
    if (ideriv >= N)
    {
        cout<<"WARNING : "<<ideriv<<"-th derivative couldn't take place for the monomial of:"<<N<<"-variables"<<endl
            <<r<<endl
            <<"CONDITION: id_derivative = { 0,1... (NbVariable-1) }"<<endl<<endl;
    }
    else
    {
        r.coef*=(Real) r.powers[ideriv];
        if (powers[ideriv] != 0)
        {
            (r.powers[ideriv])--;
        }
    }
    return r;
}
////////////////////////////////
template<typename Real, unsigned int N>
void Monomial_LD<Real,N>::writeToStream(ostream & stream) const
{
    stream<<coef<<"*"<<variables[0]<<"^"<<powers[0];
    for(unsigned int i=1; i<N; i++) stream<<"."<<variables[i]<<"^"<<powers[i];
}
////////////////////////////////
template<typename Real, unsigned int N>
void Monomial_LD<Real,N>::readFromStream(std::istream & stream)
{
    Real t_coef; int t_power;

    if (stream >> t_coef ) coef=t_coef;

    for(unsigned int i=0; i<N; ++i)
    {
        if (stream >> t_power) powers[i]=t_power;
    }
    //std::cout<<"Monomial : "<<*this<<std::endl;/////////////////////////////////////////////
}

template< typename FReal, unsigned int FN > //For comutativity of operator *: Monomial_LD*Real || Real*Monomial_LD.
Monomial_LD< FReal, FN > & operator*(const FReal & alpha,Monomial_LD< FReal, FN > & r)
{
    r *= alpha;
    return r;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>::Polynomial_LD()
{
    Monomial_LD<Real,N> monomialnull;
    listOfMonoMial.push_back(monomialnull);
    nbOfMonomial=1;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>::Polynomial_LD(const Polynomial_LD<Real,N> & a)
{
    listOfMonoMial=a.listOfMonoMial;
    nbOfMonomial=a.nbOfMonomial;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>::Polynomial_LD(const Monomial_LD<Real,N> & a)
{
    listOfMonoMial.push_back(a);
    nbOfMonomial=1;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>::Polynomial_LD(const unsigned int & nbofTerm,...)
{
    nbOfMonomial=nbofTerm;
    va_list vl;
    va_start(vl,nbofTerm);
    for (unsigned int iterm=0; iterm<nbofTerm; iterm++)
    {
        Monomial_LD<Real,N> mi;
        vector<int> powermonomiali(N,0);

        Real coefi=va_arg(vl,Real);
        for(unsigned int jvar=0; jvar<N; jvar++)
        {
            powermonomiali[jvar]=va_arg(vl,int);
            //mi.powers[jvar]=va_arg(vl,int);
        }
        mi.SetCoef(coefi); mi.SetPower(powermonomiali);
        listOfMonoMial.push_back(mi);
    }
    va_end(vl);
}
////////////////////////////////
template<typename Real, unsigned int N>
int Polynomial_LD<Real,N>::degree()
{
    int deg=0;
    for(MonomialIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
    {
        deg = ( (*it).degree() > deg ) ? (*it).degree() : deg;
    }
    return deg;
}
////////////////////////////////
template<typename Real, unsigned int N>
bool Polynomial_LD<Real,N>::operator ==(const Polynomial_LD<Real,N> & b) const
{
    bool result=true;
    if ( this->listOfMonoMial.size() != b.listOfMonoMial.size() )
        result=false;
    else
    {
        result = ( this->listOfMonoMial == b.listOfMonoMial ) ? result : false;
    }
    return result;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>  & Polynomial_LD<Real,N>::operator*=(const Real & alpha)
{
    for(MonomialIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
    {
        (*it)*=alpha;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>  & Polynomial_LD<Real,N>::operator/=(const Real & alpha)
{
    for(MonomialIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
    {
        //*it->operator/=(alpha);
        (*it)/=alpha;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>  & Polynomial_LD<Real,N>::operator+=(const Monomial_LD<Real,N> & b)
{
    bool added=false;
    for(MonomialIterator ita=listOfMonoMial.begin(); ita != listOfMonoMial.end(); ++ita)
    {
        if ( (*ita).isSamePowers(b) )
        {
            (*ita)+=b;
            added=true;
            break;
        }
    }
    if (!added) listOfMonoMial.push_back(b); nbOfMonomial++;
    return *this;
}
////////////////////////////////
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>  & Polynomial_LD<Real,N>::operator+=(const Polynomial_LD<Real,N> & b)
{
    for(MonomialConstIterator itb=b.listOfMonoMial.begin(); itb != b.listOfMonoMial.end(); ++itb)
    {
        bool added=false;
        for(MonomialIterator ita=listOfMonoMial.begin(); ita != listOfMonoMial.end(); ++ita)
        {
            if ( (*ita).isSamePowers(*itb) )
            {
                (*ita)+=(*itb);
                added=true;
                break;
            }
        }
        if (!added) listOfMonoMial.push_back((*itb)); nbOfMonomial++;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>  & Polynomial_LD<Real,N>::operator-=(const Polynomial_LD<Real,N> & b)
{
    for(MonomialConstIterator itb=b.listOfMonoMial.begin(); itb != b.listOfMonoMial.end(); ++itb)
    {
        bool added=false;
        for(MonomialIterator ita=listOfMonoMial.begin(); ita != listOfMonoMial.end(); ++ita)
        {
            if ( (*ita).isSamePowers(*itb) )
            {
                (*ita)-=(*itb);
                added=true;
                break;
            }
        }
        if (!added) listOfMonoMial.push_back(-(*itb)); nbOfMonomial++;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N>  & Polynomial_LD<Real,N>::operator*=(const Polynomial_LD<Real,N> & b)
{
    MonomialIterator ita=listOfMonoMial.begin();
    while(ita != listOfMonoMial.end())
    {
        for(MonomialConstIterator itb=b.listOfMonoMial.begin(); itb != b.listOfMonoMial.end(); ++itb)
        {
            Monomial_LD<Real,N> multipSimple=(*ita)*(*itb);
            listOfMonoMial.insert(ita,multipSimple); nbOfMonomial++;
        }
        ita=listOfMonoMial.erase(ita);
        //++ita;
    }
    return *this;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N> Polynomial_LD<Real,N>::operator-() const
{
    Polynomial_LD<Real,N> r(*this);
    for(MonomialIterator it=r.listOfMonoMial.begin(); it != r.listOfMonoMial.end(); ++it)
    {
        (*it).coef*=(Real) -1.;
    }
    return r;
}
////////////////////////////////

template<typename Real, unsigned int N>
Real Polynomial_LD<Real,N>::operator()(const sofa::helper::vector<Real> & x) const
{
    Real result=(Real) 0.;
    for(MonomialConstIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
    {
        result += (*it).operator()(x);
    }
    return result;
}

////////////////////////////////
template<typename Real, unsigned int N>
Real Polynomial_LD<Real,N>::operator()(const RNpoint & x) const
{
    Real result=(Real) 0.;
    for(MonomialConstIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
    {
        result += (*it).operator()(x);
    }
    return result;
}
////////////////////////////////
template<typename Real, unsigned int N>
Real Polynomial_LD<Real,N>::operator()(const sofa::helper::vector<Real> & x,unsigned int iderive) const
{
    Real result=(Real) 0.;
    if (iderive >= N)
    {
        cout<<"WARNING : "<<iderive<<"-th derivative couldn't take place for the polynomial of:"<<N<<"-variables"<<endl
            <<(*this)<<endl
            <<"CONDITION: id_derivative = { 0,1... (NbVariable-1) }"<<endl<<endl;
    }
    else
    {
        if(iderive==0)
        {
            result=this->operator()(x);
        }
        else
        {
            for(MonomialConstIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
            {
                result += (*it).operator()(x,iderive);
            }
        }

    }
    return result;
}

////////////////////////////////
template<typename Real, unsigned int N>
Real Polynomial_LD<Real,N>::operator()(const RNpoint & x,unsigned int iderive) const
{
    Real result=(Real) 0.;
    if (iderive >= N)
    {
        cout<<"WARNING : "<<iderive<<"-th derivative couldn't take place for the polynomial of:"<<N<<"-variables"<<endl
            <<(*this)<<endl
            <<"CONDITION: id_derivative = { 0,1... (NbVariable-1) }"<<endl<<endl;
    }
    else
    {
        if(iderive==0)
        {
            result=this->operator()(x);
        }
        else
        {
            for(MonomialConstIterator it=listOfMonoMial.begin(); it != listOfMonoMial.end(); ++it)
            {
                result += (*it).operator()(x,iderive);
            }
        }

    }
    return result;
}
////////////////////////////////
template<typename Real, unsigned int N>
Polynomial_LD<Real,N> Polynomial_LD<Real,N>::d(const unsigned int & iderive) const
{
    Polynomial_LD<Real,N> result(*this);
    if (iderive >=N)
    {
        cout<<"WARNING : "<<iderive<<"-th derivative couldn't take place for the polynomial of:"<<"-variables"<<endl
            <<result<<endl
            <<"CONDITION: id_derivative = { 0,1... (NbVariable-1) }"<<endl<<endl;
    }
    else
    {
        for(MonomialIterator it=result.listOfMonoMial.begin(); it != result.listOfMonoMial.end(); ++it)
        {
            (*it).coef*=(Real) (*it).powers[iderive];
            if ((*it).powers[iderive] != 0)
            {
                ((*it).powers[iderive])--;
            }
        }
    }
    return result;
}
////////////////////////////////
////////////////////////////////
template<typename Real, unsigned int N>
void Polynomial_LD<Real,N>::setnbOfMonomial(int m_nbofmonomial)
{
    listOfMonoMial.clear();
    nbOfMonomial=m_nbofmonomial;
    Monomial_LD<Real,N> monomialNULL;
    for(int i=0; i<nbOfMonomial; i++)
    {
        listOfMonoMial.push_back(monomialNULL);
    }
}
////////////////////////////////
template<typename Real, unsigned int N>
void Polynomial_LD<Real,N>::writeToStream(std::ostream & stream) const
{
    MonomialConstIterator it=listOfMonoMial.begin();
    stream<< *it; ++it;
    while(it != listOfMonoMial.end() )
    {
        stream << "  +  "<<*it;
        ++it;
    }
    stream<<std::endl;
}
////////////////////////////////
template<typename Real, unsigned int N>
void Polynomial_LD<Real,N>::readFromStream(std::istream & stream)
{
    int nbofmonomialtempo;

    if (stream >> nbofmonomialtempo) nbOfMonomial=nbofmonomialtempo;

    listOfMonoMial.resize(nbOfMonomial);
    MonomialIterator it=listOfMonoMial.begin();
    for(int monomialcounter=0; monomialcounter<nbOfMonomial; ++monomialcounter)
    {
        Monomial_LD<Real,N> tempo;
        if (stream >> tempo) (*it)=tempo;
        ++it;
    }
    //std::cout<<"     Polynomial :"<<*this<<std::endl;/////////////////////////////////////////////
}
////////////////////////////////
template<typename Real, unsigned int N>
std::string  Polynomial_LD<Real,N>::getString() const
{
    std::ostringstream toto;
    toto<<(*this);
    return toto.str();
}
////////////////////////////////
template< typename FReal, unsigned int FN >
Polynomial_LD< FReal, FN > & operator*(const FReal & alpha, Polynomial_LD< FReal, FN> & r)
{
    r *= alpha;
    return r;
}
////////////////////////////////
template< typename FReal, unsigned int FN >
Polynomial_LD< FReal, FN > & operator*(const Monomial_LD< FReal, FN >   & a, Polynomial_LD< FReal, FN> & r)
{
    r *= a;
    return r;
}


} // namespace helper

} // namespace sofa

#endif

