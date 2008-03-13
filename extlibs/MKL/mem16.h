#ifndef __MEM16_H_
#define __MEM16_H_

void * malloc16(unsigned int __size); 
void free16(void *__ptr);
void * calloc16(unsigned int __nmemb, unsigned int __size);
void * realloc16(void *__ptr, unsigned int __size);

#endif
