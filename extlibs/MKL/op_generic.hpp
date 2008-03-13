
template <class T> inline T generic_vector_scalar_product1(unsigned int size,T *a, T *b){
	unsigned int i;
	T sum=0;
	for(i=0;i<size;++i) {
		sum+=a[i]*b[i];
	}
	return sum;
}

template<class T> inline T generic_vector_scalar_product2(unsigned int size,T *a, T*b){
	unsigned int i,t;
	T sum=0;
	for(i=0;i<size-3;i+=4) {
		for(t=0;t<4;++t)
			sum+=a[i+t]*b[i+t];
	}
	for(;i<size;++i) 
		sum+=a[i]*b[i];
	return sum;
}

template <class T> inline  T generic_vector_scalar_product3(unsigned int size,T *a, T *b){
	unsigned int i,t;
	T sum=0;
	for(i=0;i<size-3;i+=4) {
		T *A=&a[i];
		T *B=&b[i];
		for(t=0;t<4;++t)
			sum+=A[t]*B[t];
	}
	for(;i<size;++i) 
		sum+=a[i]*b[i];
	return sum;
}

template <class T> inline T generic_vector_scalar_product4(unsigned int size,T *a, T *b){
	unsigned int i,t;
	T cum[4]={0,0,0,0};
	T sum=0;
	for(i=0;i<size-3;i+=4) {
		T *A=&a[i];
		T *B=&b[i];
		for(t=0;t<4;++t) {
			cum[t]+=A[t]*B[t];
		}
	}
	sum=cum[0]+cum[1]+cum[2]+cum[3];
	for(;i<size;++i) 
		sum+=a[i]*b[i];
	return sum;
}

template<class T> inline T generic_vector_norm_square(unsigned int size, T *a) {
	T n=0;
	unsigned int i=0;
	for(;i<size;++i) n+=a[i]*a[i];
	return n;
}

template<class T> inline void generic_vector_copy(unsigned int size, T *a, T *b) {
	unsigned int i=0;
	for(;i<size;++i) b[i]=a[i];
}

template<class T> inline void generic_vector_mul_k(unsigned int size, T*a, T k, T *b){
	unsigned int i=0;
	for(;i<size;++i) b[i]=a[i]*k;
}

template<class T> inline void generic_vector_add_mul_k(unsigned int size, T*a, T k, T*b, T*c){
	unsigned int i=0;
	for(;i<size;++i) 
		c[i]=a[i]+k*b[i];
}

template<class T> void generic_vector_neg(unsigned int size, T*a, T*b) {
	for(unsigned int i=0; i<size; ++i )
		b[i]= -a[i];
}

template<class T, unsigned int size>  inline void generic_vector_add_const_size(T*a, T*b, T*c) {
	for(unsigned int i=0; i<size; ++i )
		c[i]=a[i]+b[i];		
}

template<class T, unsigned int size>  inline void generic_vector_sub_const_size(T*a, T*b, T*c) {
	for(unsigned int i=0; i<size; ++i)
		c[i]=a[i]-b[i];
}

template<class T, unsigned int size>  inline void generic_vector_mul_const_size(T*a, T*b, T*c) {
	for(unsigned int i=0; i<size; ++i)
		c[i]=a[i]*b[i];
}
	
template<class T> inline void generic_add(T*a, T*b, T*c) {
	*c=*a+*b;	
}

template<class T> inline void generic_vector2_add(T*a, T*b, T*c) {
	c[0]=a[0]+b[0];
	c[1]=a[1]+b[1];
}

template<class T> inline void generic_vector3_add(T*a, T*b, T*c){
	c[0]=a[0]+b[0];
	c[1]=a[1]+b[1];
	c[2]=a[2]+b[2];
}

template<class T> inline void generic_vector4_add(T*a, T*b, T*c){
	c[0]=a[0]+b[0];
	c[1]=a[1]+b[1];
	c[2]=a[2]+b[2];
	c[3]=a[3]+b[3];
}

template<class T> inline void generic_sub(T*a, T*b, T*c) {
	*c=*a-*b;	
}

template<class T> inline void generic_vector2_sub(T*a, T*b, T*c){
	c[0]=a[0]-b[0];
	c[1]=a[1]-b[1];
}

template<class T> inline void generic_vector3_sub(T*a, T*b, T*c){
	c[0]=a[0]-b[0];
	c[1]=a[1]-b[1];
	c[2]=a[2]-b[2];
}

template<class T> inline void generic_vector4_sub(T*a, T*b, T*c){
	c[0]=a[0]-b[0];
	c[1]=a[1]-b[1];
	c[2]=a[2]-b[2];
	c[3]=a[3]-b[3];
}
	
template<class T> inline void generic_mul(T*a, T*b, T*c) {
	*c=*a * *b;	
}

template<class T> inline void generic_vector2_mul(T*a, T*b, T*c) {
	c[0]=a[0]*b[0];
	c[1]=a[1]*b[1];
}

template<class T> inline void generic_vector3_mul(T*a, T*b, T*c) {
	c[0]=a[0]*b[0];
	c[1]=a[1]*b[1];
	c[2]=a[2]*b[2];
}

template<class T> inline void generic_vector4_mul(T*a, T*b, T*c) {
	c[0]=a[0]*b[0];
	c[1]=a[1]*b[1];
	c[2]=a[2]*b[2];
	c[3]=a[3]*b[3];
}

template<class T> inline void generic_vector_add(unsigned int size, T*a, T*b, T*c) {
	unsigned int i=0;
	for(;i<size;++i) c[i]=a[i]+b[i];
}

template<class T> inline void generic_vector_sub(unsigned int size, T*a, T*b, T*c) {
	unsigned int i=0;
	for(;i<size;++i) c[i]=a[i]-b[i];
}

template<class T> inline void generic_vector_mul(unsigned int size, T*a, T*b, T*c) {
	unsigned int i=0;
	for(;i<size;++i) c[i]=a[i]*b[i];
}
