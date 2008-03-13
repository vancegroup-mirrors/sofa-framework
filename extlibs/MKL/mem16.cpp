#include "mem16.h"

#include <stdlib.h>
#include "portable_iostream.h"

struct mem16_info {
	char *magic_end;
	void* real_addr;
	unsigned int size;
	char magic[4];
};

#define MEMSIZE_END (16)
#define MEMSIZE_OVERHEAD (16+sizeof(mem16_info)+MEMSIZE_END)

inline void set_magic(char *ptr) {
	ptr[0]='M';
	ptr[1]='A';
	ptr[2]='G';
	ptr[3]='I';
}

inline bool is_magic(char *ptr) {
	return ptr[0]=='M' 
		&& ptr[1]=='A'
		&& ptr[2]=='G'
		&& ptr[3]=='I';
}


inline void show_mem16_info(void *ptr) {
	mem16_info * ptr16   =((mem16_info *)ptr-1);
	cout << "ptr16   " << ptr16 << endl
		<< "  ptr    " << ptr16->real_addr << endl
		<< "  size   " << ptr16->size << endl
		<< "  magic_end* " << (void*)ptr16->magic_end << endl
		<< "  size   " << (void *) ((char *)ptr16->magic_end - (char *) ptr) << endl
		<< "  offset " << (void *) ((char *)ptr 			- (char *)ptr16->real_addr) << endl;
}

inline void * align16(void *ptr, unsigned int __size) {
	mem16_info* ptr16=((mem16_info *)( ((unsigned int)ptr+MEMSIZE_OVERHEAD-MEMSIZE_END)&0xFFFFFFF0 ))-1;
	ptr16->real_addr=ptr;
	ptr16->size=__size;
	ptr16->magic_end=((char *)(ptr16+1))+__size;
	set_magic(ptr16->magic);
	set_magic(ptr16->magic_end);
	return ptr16+1;
}	

inline void * getrealaddr16(void *ptr) {
	return ((mem16_info *)ptr-1)->real_addr;
}

inline unsigned int getsize16(void *ptr) {
	return ((mem16_info *)ptr-1)->size;
}

inline bool test_lower_bound_magic(void *ptr) {
	return is_magic(((mem16_info *)ptr-1)->magic);
}

inline bool test_upper_bound_magic(void *ptr) {
	return is_magic(((mem16_info *)ptr-1)->magic_end);
}

inline void test_memory_integrity(void *ptr) {
	if(!test_lower_bound_magic(ptr)) {
		cout << "memory integrity violation: Lower bound" << endl;
	}
	if(!test_upper_bound_magic(ptr)) {
		cout << "memory integrity violation: Upper bound" << endl;
		show_mem16_info(ptr);
	}
}

void * malloc16(unsigned int __size) {
	//cout << "malloc16 " << __size << endl;
	return align16(malloc(__size+MEMSIZE_OVERHEAD),__size);
}

void free16(void *__ptr) {
	test_memory_integrity(__ptr);
	free(getrealaddr16(__ptr));
}

void * calloc16(unsigned int __nmemb, unsigned int __size) {
	//cout << "calloc16 " << __nmemb << " x " << __size << endl;
	return align16(calloc(1,__nmemb*__size+MEMSIZE_OVERHEAD),__nmemb*__size);
}

void * realloc16(void *__ptr, unsigned int __size) {
	//cout << "realloc16 " << __size << endl;
	test_memory_integrity(__ptr);
	return align16(realloc(getrealaddr16(__ptr),__size+MEMSIZE_OVERHEAD),__size);
	
}
