/*********************************************************************/
/* <Your program description here>                                   */
/* Name:  <Your name here>                                           */
/* Date:  <Date completed>                                           */
/* Class:  CMPE 250                                                  */
/* Section:  <Your section here>                                     */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            March 30, 2018                                         */
/*********************************************************************/
#include "MKL46Z4.h"
#include "Exercise12_C.h"
#include <string.h>
#include <stdlib.h>

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)
#define MAX_GATES  (8)

#define SCREEN_X   (40)
#define SCREEN_Y   (40)

#define MIN_VALID 0x31
#define MAX_VALID 0x35

#define ARRAY_DIGITS (3)

// static variable, lord help me
Game  theGame;
UInt8 cont;

//void accelerateCar(short *int value)
void movePointLeft(Point *objPoint, Int8 value) {
    objPoint->X -= value;
}

void movePointRight(Point *objPoint, Int8 value) {
    objPoint->X += value;
}

/* Function which replaces the first "num" items in "dest" starting at "start" with "src" */
void strReplace(Int8 dest[], Int8 src[], Int8 start, Int8 num) {
  UInt8 index = start;
  UInt8 max   = start + num;
  UInt8 i     = 0;
  for (; i < num; i++) {
    index = start + i;
    dest[index] = src[i];
  }
}

void fillArray(Int16 value, Int8 dest[], Int8 arraySize) {
    Int8  index = 0;
    Int16 mod   = 10;
    Int8  div   = 1;
    for (index = 0; index < arraySize; index++) {
      mod = 10;
      Int8  g = ((value % pow(mod, index + 1)) / (pow(div, index))) + 0x30;
      dest[index] = g;
      div = 10;
    }
}

/* code only works if string has size 3. bite me */
void strReverse3(Int8 array[]) {
  Int8 val1 = array[2];
  array[2] = array[0];
  array[0] = val1;
}

Int32 pow(Int32 base, Int32 exp) {
  Int32 orig = base;
  for (; exp > 1; exp--) { base *= orig; }
  return base;
}

void moveCursorToLocation(Int8 x, Int8 y) {
    Int8 escape[10] = "0[000;000H";
    Int8 xs[ARRAY_DIGITS];
    Int8 ys[ARRAY_DIGITS];
    fillArray(x, xs, ARRAY_DIGITS);
    strReverse3(xs);
    fillArray(y, ys, ARRAY_DIGITS);
    strReverse3(ys);
    /* escape[0] = 0x1B; */
    /* PutStringSB(escape, 10); */
    strReplace(escape, ys, 2, 3);
    strReplace(escape, xs, 6, 3);
    PutStringSB(escape, 10);
}

void printScreen(Game *game) {
    // for each gate
    //   printGate(gate);
    // print car
  UInt8 i          = 0;
  UInt8 yOver      = 0;
  UInt8 escape1[10] = "0[000;000H";
  escape1[0]        = 0x1B;
  PutStringSB(escape1, 10);
  UInt8 escape2[4] = "\033[2J";
  PutStringSB(escape2, 4);
  Entity *car = &(game->car);
  for (; yOver < SCREEN_Y; yOver++) {
    for (; i < game->currentGates; i++) {
      if (game->gates[i].location.Y == yOver) {
        PutChar('G');
      }
    }
    if (car->location.Y == yOver) {
      UInt8 x = car->location.X - 1;
      while ( x > 0) { 
        x--; 
        PutChar(' '); 
      }
      PutStringSB("o^o\n\b\b\b|o|\n\b\b\bo-o", 17);
    }
    PutChar('\n');
    PutChar('\r');
  }
}

void moveGates() {
  UInt8 i = 0;
  for (; i < theGame.currentGates; i++) {
    theGame.gates[i].location.Y += 1;
  }    
}

void tick(Game *game) {
    Int8 c = GetChar();
    Int8 gotChar = CheckCFlag();
    Int8 print = 0;
    if (gotChar == 1) {
        if (c == 'a') {
            print = 1;
            game->car.location.X -= 1;
            if (game->car.location.X == 255) {
              game->car.location.X = SCREEN_X;
            }
        } else if (c == 'd') {
            print = 1;
            game->car.location.X += 1;
            if (game->car.location.X == SCREEN_X) {
              game->car.location.X = 0;
            }
        } if (c == 'w') {
            print = 1;
            game->car.location.Y -= 1;
            if (game->car.location.Y == 255) {
              game->car.location.Y = SCREEN_Y;
            }
        } else if (c == 's') {
            print = 1;
            game->car.location.Y += 1;
            if (game->car.location.Y == SCREEN_Y) {
              game->car.location.Y = 0;
            }
        } else if (c == 'p') {
            print = 1;
            cont = 0;
        }
        
        // update game
        moveGates();
        
    } else {
        /* do nothing */
    }
    if (print == 1) {
      //PutChar('P');
      printScreen(game);
    }
    PutChar(c);
}

int main (void) {

  __asm("CPSID   I");  /* mask interrupts */

  /* Perform all device initialization here */
  Init_UART0_IRQ();
    
  /* Before unmasking interrupts            */
  __asm("CPSIE   I");  /* unmask interrupts */
  
  UInt8 input;
  cont = 1;
  theGame.car.location.Y = 15;
  theGame.car.location.X = 5;
  theGame.gates[0].location.Y = 5;
  theGame.gates[0].location.X = 5;
  theGame.currentGates =   1;
  theGame.totalScore =     0;
  
  while (cont == 1) { /* do forever */
    tick(&theGame);
    //PutChar(GetChar());
  } /* do forever */

  return (0);
} /* main */

/* end of file */




