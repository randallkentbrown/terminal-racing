/*********************************************************************/
/* Lab Exercise Eleven                                               */
/* Adjusts a servo to one of five positions [1, 5] using  mixed C    */
/* and assembly language.  Prompts user to enter a number from 1 to  */
/* 5, generates a voltage in the range (0, 3.3] V proportional to    */
/* the user's number, converts the voltage to a 10-bit number, and   */
/* set's the servo position [1, 5] based on the magnitude of the 10- */
/* bit digital value.                                                */
/* Name:  R. W. Melton                                               */
/* Date:  April 9, 2018                                              */
/* Class:  CMPE 250                                                  */
/* Section:  All sections                                            */
/*********************************************************************/
#define MAX_GATES (8)

typedef int Int32;
typedef short int Int16;
typedef char Int8;
typedef unsigned int UInt32;
typedef unsigned short int UInt16;
typedef unsigned char UInt8;

typedef struct {
    Int8 X, Y;
} Point;

typedef struct {
    Point location;
    Point speed;
} Entity;

typedef struct {
  Entity car;
  Entity gates[MAX_GATES];
  Int16  totalScore;
  Int8   currentGates;
} Game;

/* assembly language ROM table entries */
extern UInt16 DAC0_table_0;
extern UInt16 PWM_duty_table_0;

/* C functions */
extern void moveObjectLeft(Point *objPos, char value);
extern void moveObjectRight(Point *objPos, char value);

/* assembly language subroutines */
char GetChar        (void);
char GetCharTick    (void);
void GetStringSB    (char String[], int StringBufferCapacity);
void Init_UART0_IRQ (void);
void PutChar        (char Character);
void PutNumHex      (UInt32);
void PutNumU        (UInt32);
void PutNumUB       (UInt8);
void PutStringSB    (char String[], int StringBufferCapacity);
char CheckCFlag     (void);
