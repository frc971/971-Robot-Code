#ifndef _BinHeap_H
#define _BinHeap_H

#include <stdint.h>

typedef uint8_t ElementType;
struct HeapStruct;
typedef struct HeapStruct *PriorityQueue;

struct HeapStruct
{
	int Capacity;
	int Size;
	ElementType *Elements;
};

// Elements is the number allocated at H->Elements
void Initialize( int Elements, PriorityQueue H );
// 0 if successful, -1 if full
int Insert( ElementType X, PriorityQueue H );
void Remove( ElementType X, PriorityQueue H );
ElementType DeleteMin( PriorityQueue H );
ElementType GetMin( PriorityQueue H );
int IsEmpty( PriorityQueue H );
int IsFull( PriorityQueue H );
int GetSize( PriorityQueue H );

#endif

