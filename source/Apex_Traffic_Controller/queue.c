#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define MAX 6

uint8_t intArray[MAX];
int front = 0;
int rear = -1;
int itemCount = 0;


bool isFull() {
   return itemCount == MAX;
}


void push(uint8_t data) {

   if(!isFull()) {

      if(rear == MAX-1) {
         rear = -1;
      }

      intArray[++rear] = data;
      itemCount++;
   }
}

uint8_t pop() {
   uint8_t data = intArray[front++];

   if(front == MAX) {
      front = 0;
   }

   itemCount--;
   return data;
}
