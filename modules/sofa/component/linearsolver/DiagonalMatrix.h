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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_LINEARSOLVER_DIAGONALMATRIX_H
#define SOFA_COMPONENT_LINEARSOLVER_DIAGONALMATRIX_H

#include "MatrixExpr.h"
#include "NewMatMatrix.h"

namespace sofa {

namespace component {

namespace linearsolver {

/// Simple full matrix container
template<typename T>
class DiagonalMatrix : public defaulttype::BaseMatrix {
public:
    typedef T Real;
    typedef int Index;

    typedef DiagonalMatrix<T> Expr;
    typedef DiagonalMatrix<double> matrix_type;
    enum { category = MATRIX_DIAGONAL };
    enum { operand = 1 };

protected:
    FullVector<T> data;

public:

    DiagonalMatrix()
    {
    }

    DiagonalMatrix(int nbRow, int nbCol)
    : data(new T[nbRow])
    {
    }

    DiagonalMatrix(Real* p, int nbRow, int nbCol)
    : data(p)
    {
    }

    ~DiagonalMatrix() {}

    Real* ptr() { return data.ptr(); }
    const Real* ptr() const { return data.ptr(); }

    Real* operator[](Index i) {
        return data+i;
    }

    const Real* operator[](Index i) const {
        return data+i;
    }

    void resize(int nbRow, int /*nbCol*/) {
	data.resize(nbRow);
    }

    unsigned int rowSize(void) const {
        return data.size();
    }

    unsigned int colSize(void) const{
        return data.size();
    }

    SReal element(int i, int j) const {
        if (i!=j) return (Real)0;
        return data[i];
    }

    void set(int i, int j, double v) {
        if (i==j) data[i] = (Real)v;
    }

    void add(int i, int j, double v) {
        if (i==j) data[i] += (Real)v;
    }

    void clear(int i, int j) {
        if (i==j) data[i] = (Real)0;
    }

    void clearRow(int i) {
        data[i] = (Real)0;
    }

    void clearCol(int j) {
        data[j] = (Real)0;
    }

    void clearRowCol(int i) {
        data[i] = (Real)0;
    }

    void clear() {
	data.clear();
    }

    // operators similar to vectors

    void resize(int nbRow) {
	data.resize(nbRow);
    }

    unsigned int size() const {
        return data.size();
    }

    void swap(DiagonalMatrix<T>& v) {
        data.swap(v.data);
    }

    const SReal& element(int i) const {
        return data[i];
    }

    SReal& element(int i) {
        return data[i];
    }

    void set(int i, double v) {
        data[i] = (Real)v;
    }

    void add(int i, double v) {
        data[i] += (Real)v;
    }

    void clear(int i) {
        data[i] = (Real)0;
    }

    template<class Real2>
    FullVector<Real2> operator*(const FullVector<Real2>& v) const {
        FullVector<Real2> res;
	for (int i=0;i<rowSize();i++) res[i] = data[i] * v[i];
        return res;
    }

    // methods for MatrixExpr support

    template<class M2>
    bool hasRef(const M2* m) const
    {
        return (const void*)this == (const void*)m;
    }

    std::string expr() const
    {
        return std::string(Name());
    }

    bool valid() const
    {
        return true;
    }

    template<class Dest>
    void doCompute(Dest* dest) const
    {
        int ny = rowSize();
        for (int y=0;y<ny;++y)
            dest->add(y,y,element(y));
    }

protected:

    template<class M>
    void compute(const M& m, bool add = false)
    {
        if (m.hasRef(this))
        {
            DiagonalMatrix<T> tmp;
            tmp.resize(m.rowSize(), m.colSize());
            m.doCompute(&tmp);
            if (add)
                tmp.doCompute(this);
            else
                swap(tmp);
        }
        else
        {
            if (!add)
                resize(m.rowSize(), m.colSize());
            m.doCompute(this);
        }
    }
public:

    template<class Real2>
    void operator=(const DiagonalMatrix<Real2>& m)
    {
        if (&m == this) return;
        resize(m.rowSize(), m.colSize());
        m.doCompute(this);
    }

    template<class Real2>
    void operator+=(const DiagonalMatrix<Real2>& m)
    {
        compute(m, true);
    }

    template<class Real2>
    void operator-=(const DiagonalMatrix<Real2>& m)
    {
        compute(MatrixExpr< MatrixNegative< DiagonalMatrix<Real2> > >(MatrixNegative< DiagonalMatrix<Real2> >(m)), true);
    }

    template<class Expr2>
    void operator=(const MatrixExpr< Expr2 >& m)
    {
        compute(m, false);
    }

    template<class Expr2>
    void operator+=(const MatrixExpr< Expr2 >& m)
    {
        compute(m, true);
    }

    template<class Expr2>
    void operator-=(const MatrixExpr< Expr2 >& m)
    {
        compute(MatrixExpr< MatrixNegative< Expr2 > >(MatrixNegative< Expr2 >(m)), true);
    }

    MatrixExpr< MatrixTranspose< DiagonalMatrix<T> > > t() const
    {
        return MatrixExpr< MatrixTranspose< DiagonalMatrix<T> > >(MatrixTranspose< DiagonalMatrix<T> >(*this));
    }

    MatrixExpr< MatrixInverse< DiagonalMatrix<T> > > i() const
    {
        return MatrixExpr< MatrixInverse< DiagonalMatrix<T> > >(MatrixInverse< DiagonalMatrix<T> >(*this));
    }

    MatrixExpr< MatrixNegative< DiagonalMatrix<T> > > operator-() const
    {
        return MatrixExpr< MatrixNegative< DiagonalMatrix<T> > >(MatrixNegative< DiagonalMatrix<T> >(*this));
    }

    MatrixExpr< MatrixScale< DiagonalMatrix<T>, double > > operator*(const double& r) const
    {
        return MatrixExpr< MatrixScale< DiagonalMatrix<T>, double > >(MatrixScale< DiagonalMatrix<T>, double >(*this, r));
    }

    friend std::ostream& operator << (std::ostream& out, const DiagonalMatrix<T>& v ) {
        int ny = v.rowSize();
        out << "[";
        for (int y=0;y<ny;++y) out << " " << v.element(y);
        out << " ]";
        return out;
    }

    static const char* Name() { return "DiagonalMatrix"; }
};


/// Simple full matrix container
template<int LC>
class BlockDiagonalMatrix : public defaulttype::BaseMatrix {
public:
    typedef double Real;
    typedef int Index;
    typedef NEWMAT::Matrix SubMatrixType;
    typedef NEWMAT::InvertedMatrix InvertedMatrix;

    enum { BSIZE = LC };

protected:
    std::vector< SubMatrixType > data;
    unsigned cSize;
    
public:
    
    BlockDiagonalMatrix() {
	cSize = 0;
    }

    ~BlockDiagonalMatrix() {}

    void resize(int nbRow, int ) {
	cSize = nbRow;
	data.resize((cSize+LC-1) / LC);
	for (unsigned i=0;i<data.size();i++) data[i].ReSize(LC,LC);
    }

    unsigned rowSize(void) const {
        return cSize;
    }

    unsigned colSize(void) const{
        return cSize;
    }

    Real element(int i, int j) const {
        if (i/LC!=j/LC) return (Real)0;
        return data[i/LC].element(i%LC,j%LC);
    }

    void set(int i, int j, double v) {
	if (i/LC==j/LC) data[i/LC].element(i%LC,j%LC) = (Real)v; 
    }

    void add(int i, int j, double v) {
	if (i/LC==j/LC) data[i/LC].element(i%LC,j%LC) += (Real)v;
    }

    void clear() {
	for (unsigned b=0;b<data.size();b++) 
	  for (int j=0;j<LC;j++) 
	    for (int i=0;i<LC;i++) 
	      data[b].element(i,j) = 0.0;;
    }
    
    void clear(int i, int j) {
        if (i/LC==j/LC) data[i/LC].element(i%LC,j%LC) = 0.0;
    }

    void clearRow(int i) {
	  for (int j=0;j<LC;j++) 
	    data[i/LC].element(i,j) = 0.0;;
    }

    void clearCol(int j) {
	  for (int i=0;i<LC;i++) 
	    data[j/LC].element(i,j) = 0.0;;

    }

    void clearRowCol(int i) {
        clearRow(i); 
	clearCol(i);
    }
    
    SubMatrixType sub(int l,int ,int ,int ) {
	SubMatrixType m=data[l/LC];
	return m;
    }
    
    void setSubMatrix(unsigned l,unsigned ,unsigned ,unsigned ,InvertedMatrix m) {
	data[l/LC] = m;
    }
    
    void i() {
	for (unsigned b=0;b<data.size();b++) data[b] = data[b].i();
    }
    

    template<class Real2>
    FullVector<Real2> operator*(const FullVector<Real2>& v) const {
        FullVector<Real2> res;
	res.resize(cSize);
	for (unsigned b=0;b<data.size()-1;b++) {
	  for (int j=0;j<LC;j++) {
	    res[b*LC+j] = 0;
	    for (int i=0;i<LC;i++) {
		res[b*LC+j] += data[b].element(i,j) * v[b*LC+i];
	    }
	  }
	}

	int last_block = (data.size()-1)*LC;
	for (int j=0;last_block+j<(int) cSize;j++) {
	    res[last_block+j] = 0;
	    for (int i=0;i<LC;i++) {
		res[j] += data[data.size()-1].element(i,j) * v[last_block+i];
	    }
	}

	return res;
    }
    
    template<class Real2>
    void mult(FullVector<Real2>& res, const FullVector<Real2>& v) {
	for (unsigned b=0;b<data.size()-1;b++) {
	  for (int j=0;j<LC;j++) {
	    res[b*LC+j] = 0;
	    for (int i=0;i<LC;i++) {
		res[b*LC+j] += data[b].element(i,j) * v[b*LC+i];
	    }
	  }
	}

	int last_block = (data.size()-1)*LC;
	for (int j=0;last_block+j<(int) cSize;j++) {
	    res[last_block+j] = 0;
	    for (int i=0;i<LC;i++) {
		res[last_block+j] += data[data.size()-1].element(i,j) * v[last_block+i];
	    }
	}
    }
    
    friend std::ostream& operator << (std::ostream& out, const BlockDiagonalMatrix<LC>& v ) {
	out << "[";
        for (unsigned i=0;i<v.data.size();i++) out << " " << v.data[i];
        out << " ]";
        return out;
    }

    static const char* Name(); // { return "BlockDiagonalMatrix"; }
};

typedef BlockDiagonalMatrix<3> BlockDiagonalMatrix3;
typedef BlockDiagonalMatrix<6> BlockDiagonalMatrix6;
typedef BlockDiagonalMatrix<9> BlockDiagonalMatrix9;
typedef BlockDiagonalMatrix<12> BlockDiagonalMatrix12;

template<> inline const char* BlockDiagonalMatrix3::Name() { return "BlockDiagonalMatrix3"; }
template<> inline const char* BlockDiagonalMatrix6::Name() { return "BlockDiagonalMatrix6"; }
template<> inline const char* BlockDiagonalMatrix9::Name() { return "BlockDiagonalMatrix9"; }
template<> inline const char* BlockDiagonalMatrix12::Name() { return "BlockDiagonalMatrix12"; }

// trivial product and inverse operations for diagonal matrices

template<class R1, class M2>
class MatrixProductOp<DiagonalMatrix<R1>, M2>
{
protected:
    template<class Dest>
    class MyDest
    {
    public:
        const DiagonalMatrix<R1>& m1;
        Dest* d;
        MyDest(const DiagonalMatrix<R1>& m1, Dest* d) : m1(m1), d(d) {}
        void add(int l, int c, double v) { d->add(l,c,m1.element(l)*v); }
    };
public:
    typedef typename M2::matrix_type matrix_type;
    enum { category = M2::category };

    template<class Dest>
    void operator()(const DiagonalMatrix<R1>& m1, const M2& m2, Dest* d)
    {
        MyDest<Dest> myd(m1,d);
        std::cout << "EXPR using diagonal pre-product: " << m1.expr() << " * " << m2.expr() << std::endl;
        m2.doCompute(&myd);
    }
};

template<class M1, class R2>
class MatrixProductOp<M1, DiagonalMatrix<R2> >
{
protected:
    template<class Dest>
    class MyDest
    {
    public:
        const DiagonalMatrix<R2>& m2;
        Dest* d;
        MyDest(const DiagonalMatrix<R2>& m2, Dest* d) : m2(m2), d(d) {}
        void add(int l, int c, double v) { d->add(l,c,v*m2.element(c)); }
    };
public:
    typedef typename M1::matrix_type matrix_type;
    enum { category = M1::category };

    template<class Dest>
    void operator()(const M1& m1, const DiagonalMatrix<R2>& m2, Dest* d)
    {
        MyDest<Dest> myd(m2,d);
        std::cout << "EXPR using diagonal post-product: " << m1.expr() << " * " << m2.expr() << std::endl;
        m1.doCompute(&myd);
    }
};

template<class R1, class R2>
class MatrixProductOp<DiagonalMatrix<R1>, DiagonalMatrix<R2> >
{
public:
    typedef DiagonalMatrix<R1> M1;
    typedef DiagonalMatrix<R2> M2;
    typedef typename type_selector<(sizeof(R2)>sizeof(R1)),M1,M2>::T matrix_type;
    enum { category = matrix_type::category };

    template<class Dest>
    void operator()(const DiagonalMatrix<R1>& m1, const DiagonalMatrix<R2>& m2, Dest* d)
    {
        unsigned int n = m1.size();
        std::cout << "EXPR using diagonal product: " << m1.expr() << " * " << m2.expr() << std::endl;
        for (unsigned int i=0;i<n;++i)
            d->add(i,i,m1.element(i)*m2.element(i));
    }
};

template<class R1>
class MatrixInvertOp<DiagonalMatrix<R1> >
{
public:
    typedef DiagonalMatrix<double> matrix_type;
    enum { category = matrix_type::category };

    template<class Dest>
    void operator()(const DiagonalMatrix<R1>& m1, Dest* d)
    {
        unsigned int n = m1.size();
        for (unsigned int i=0;i<n;++i)
            d->add(i,i,1.0/m1.element(i));
    }
};

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
