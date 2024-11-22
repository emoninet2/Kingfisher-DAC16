/*
 * fifo.h
 *
 *  Created on: Nov 21, 2024
 *      Author: habiburrahman
 */

#ifndef FIFO_QUEUE_H
#define FIFO_QUEUE_H

#include <stdbool.h>

// Define the maximum size of the queue
#define MAX_QUEUE_SIZE 1024

// FIFOQueue structure definition
typedef struct {
    void **data;     // Array of void pointers to hold different data types
    int front;       // Index of the front element
    int rear;        // Index of the rear element
    int size;        // Current size of the queue
} FIFOQueue;

// Function declarations
FIFOQueue* createQueue();
bool isFull(FIFOQueue *queue);
bool isEmpty(FIFOQueue *queue);
bool enqueue(FIFOQueue *queue, void *value);
void* dequeue(FIFOQueue *queue);
void* peek(FIFOQueue *queue);
void clearQueue(FIFOQueue *queue);
void freeQueue(FIFOQueue *queue);
void printQueue(FIFOQueue *queue, void (*printFunc)(void*));

#endif  // FIFO_QUEUE_H
