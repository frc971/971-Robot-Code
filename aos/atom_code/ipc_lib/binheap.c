#include "binheap.h"
#include <stdlib.h>
#include <stdio.h>

#ifndef TESTING_ASSERT
#define TESTING_ASSERT(...)
#endif
#define Error(x) TESTING_ASSERT(0, x)

#define MinData (0)

void Initialize( int Elements, PriorityQueue H )
{
	H->Capacity = Elements - 1;
	H->Size = 0;
	H->Elements[ 0 ] = MinData;
}

int Insert( ElementType X, PriorityQueue H )
{
	int i;

	if( IsFull( H ) )
	{
		return -1;
	}

	for( i = ++H->Size; H->Elements[ i / 2 ] > X; i /= 2 )
		H->Elements[ i ] = H->Elements[ i / 2 ];
	H->Elements[ i ] = X;
	return 0;
}

void Remove( ElementType X, PriorityQueue H )
{
	int i, Child, removed = 0;
	ElementType LastElement;

	for ( i = 1; i <= H->Size; ++i )
	{
		if( H->Elements[ i ] == X )
		{
			removed = i;
			break;
		}
	}
	if( removed == 0 )
	{
		fprintf(stderr, "could not find element %d to remove. not removing any\n", X);
		return;
	}

	LastElement = H->Elements[ H->Size-- ];

	for( i = removed; i * 2 <= H->Size; i = Child )
	{
		/* Find smaller child */
		Child = i * 2;
		if( Child != H->Size && H->Elements[ Child + 1 ]
				< H->Elements[ Child ] )
			Child++;

		/* Percolate one level */
		if( LastElement > H->Elements[ Child ] )
			H->Elements[ i ] = H->Elements[ Child ];
		else
			break;
	}
	H->Elements[ i ] = LastElement;
}

ElementType DeleteMin( PriorityQueue H )
{
	int i, Child;
	ElementType MinElement, LastElement;

	if( IsEmpty( H ) )
	{
		Error( "Priority queue is empty" );
		return H->Elements[ 0 ];
	}
	MinElement = H->Elements[ 1 ];
	LastElement = H->Elements[ H->Size-- ];

	for( i = 1; i * 2 <= H->Size; i = Child )
	{
		/* Find smaller child */
		Child = i * 2;
		if( Child != H->Size && H->Elements[ Child + 1 ]
				< H->Elements[ Child ] )
			Child++;

		/* Percolate one level */
		if( LastElement > H->Elements[ Child ] )
			H->Elements[ i ] = H->Elements[ Child ];
		else
			break;
	}
	H->Elements[ i ] = LastElement;
	return MinElement;
}

ElementType GetMin( PriorityQueue H )
{
	if( !IsEmpty( H ) )
		return H->Elements[ 1 ];
	Error( "Priority Queue is Empty" );
	return H->Elements[ 0 ];
}

int IsEmpty( PriorityQueue H )
{
	return H->Size == 0;
}

int IsFull( PriorityQueue H )
{
	return H->Size == H->Capacity;
}

int GetSize( PriorityQueue H )
{
	return H->Size;
}

