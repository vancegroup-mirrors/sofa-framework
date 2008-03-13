#include <stdlib.h>

template<class T>
inline Dynamic_Vector<T>::Dynamic_Vector(){
      reserved_size=length=0;
      v=0;
}


template<class T>
inline Dynamic_Vector<T>::Dynamic_Vector(unsigned int length){
      reserved_size=Dynamic_Vector::length=length;
      create();
}


template<class T>
inline Dynamic_Vector<T>::Dynamic_Vector(const Dynamic_Vector<T> & other){
      length=other.length;
      create();
      copy(other);
}

template<class T>
inline Dynamic_Vector<T>::~Dynamic_Vector(){
      destroy();
}


template<class T>
inline unsigned int Dynamic_Vector<T>::size() const {
      return length;
}


template<class T>
inline bool Dynamic_Vector<T>::is_empty() const {
      return length==0;
}

template<class T>
inline void Dynamic_Vector<T>::empty()
{
  length=0;
}


template<class T>
inline unsigned int Dynamic_Vector<T>::capacity(){
      return reserved_size;
}

template<class T>
inline void Dynamic_Vector<T>::reserve(unsigned int size){
      if(reserved_size<size) {
	  reserved_size=size+1;
	  if(v!=0)
	      v=(T*)realloc(v,sizeof(T)*reserved_size);
	  else
	      v=(T*)malloc(sizeof(T)*reserved_size);
      }
}


template<class T>
inline void Dynamic_Vector<T>::resize(unsigned int size){
      length=size;
      if(v!=0) v=(T*)realloc(v,sizeof(T)*length);
      else v=(T*)malloc(sizeof(T)*length);
      reserved_size=length;
}

template<class T>
inline void Dynamic_Vector<T>::push_back(const T &other){ 
      if(length>=reserved_size) reserve(length*2+4);
      v[length]=other;
      length++;
}


template<class T>
inline T Dynamic_Vector<T>::pop_back(){
      if(length!=0) length--;
      return v[length];
}


template<class T>
inline Dynamic_Vector<T> & Dynamic_Vector<T>::operator = (const Dynamic_Vector<T> & other){
    copy(other);
    return *this;
}


template<class T>
inline T & Dynamic_Vector<T>::operator[](const unsigned int index){
    return v[index];
}

template<class T>
inline const T & Dynamic_Vector<T>::operator[](const unsigned int index) const{
    return v[index];
}

template<class T>
inline void Dynamic_Vector<T>::create(){
      v=(T*)calloc(length + 1, sizeof(T));
      reserved_size=length;
}

template<class T>
inline void Dynamic_Vector<T>::copy(const Dynamic_Vector<T> & other){
    if(&other !=this) {
	    if (length!=other.length) {
		      resize(other.length);
		}
	    unsigned int i;
	    for(i=0; i<length; i++)
		v[i]=other[i];
	}
}

template<class T>
inline void Dynamic_Vector<T>:: destroy(){
      reserved_size=length=0;
      if(v!=0)
	    free(v);
      v=0;
}

template<class T>
ostream & operator <<(ostream & out, const Dynamic_Vector<T> & other){
      return vector_put(out,other.length,other);
}

template<class T> unsigned int vector_length(const Dynamic_Vector<T> & other) {
      return other.size();
}

