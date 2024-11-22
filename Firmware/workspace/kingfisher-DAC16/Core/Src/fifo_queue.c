#include <stdio.h>
#include <stdlib.h>
#include "fifo_queue.h"

// Function to create and initialize the FIFO queue
FIFOQueue* createQueue() {
    FIFOQueue *queue = (FIFOQueue *)malloc(sizeof(FIFOQueue));
    if (!queue) {
        //printf("Memory allocation failed.\n");
        return NULL;
    }
    queue->data = (void **)malloc(sizeof(void*) * MAX_QUEUE_SIZE);
    if (!queue->data) {
        printf("Memory allocation failed.\n");
        free(queue);
        return NULL;
    }
    queue->front = 0;
    queue->rear = 0;
    queue->size = 0;
    return queue;
}

// Function to check if the queue is full
bool isFull(FIFOQueue *queue) {
    return queue->size == MAX_QUEUE_SIZE;
}

// Function to check if the queue is empty
bool isEmpty(FIFOQueue *queue) {
    return queue->size == 0;
}

// Function to enqueue an element into the queue
bool enqueue(FIFOQueue *queue, void *value) {
    if (isFull(queue)) {
        //printf("Queue is full. Cannot enqueue value.\n");
        return false;
    }
    queue->data[queue->rear] = value;
    queue->rear = (queue->rear + 1) % MAX_QUEUE_SIZE;
    queue->size++;
    return true;
}

// Function to dequeue an element from the queue
void* dequeue(FIFOQueue *queue) {
    if (isEmpty(queue)) {
        //printf("Queue is empty. Cannot dequeue.\n");
        return NULL;  // Return NULL to indicate an error (empty queue)
    }
    void *value = queue->data[queue->front];
    queue->front = (queue->front + 1) % MAX_QUEUE_SIZE;
    queue->size--;
    return value;
}

// Function to peek at the front element of the queue without dequeuing
void* peek(FIFOQueue *queue) {
    if (isEmpty(queue)) {
        //printf("Queue is empty. Cannot peek.\n");
        return NULL;  // Return NULL to indicate an error (empty queue)
    }
    return queue->data[queue->front];
}

// Function to clear the queue
void clearQueue(FIFOQueue *queue) {
    queue->front = queue->rear = queue->size = 0;
}

// Function to free the allocated memory for the queue
void freeQueue(FIFOQueue *queue) {
    free(queue->data);
    free(queue);
}

// Function to print the elements in the queue
void printQueue(FIFOQueue *queue, void (*printFunc)(void*)) {
    if (isEmpty(queue)) {
        //printf("Queue is empty.\n");
        return;
    }
    int i = queue->front;
    //printf("Queue: ");
    for (int j = 0; j < queue->size; j++) {
        printFunc(queue->data[i]);
        //printf(" ");
        i = (i + 1) % MAX_QUEUE_SIZE;
    }
    //printf("\n");
}
