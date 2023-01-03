/*
 
 * kickMoters4.c
 
 *
 
 *  Created on: Nov 1, 2022
 
 *      Author: james
 
 */

/*
 
 
 
 This program is intended to kick the tires on using the dual H-bridge and two DC motors.
 
 
 
 Connection for power:
 
 - 5V from battery pack into the 5V input block on proto-board only powers the servos.
 
 - The jumper J1 must be in place tpo power the servos
 
 - A wire jumper from Vin to 5V reg will power the whole board with 5V.  This allows for
 
 power to the whole car either from the USB or from 5V battery pack
 
 
 
 - Power motors with the 5v battery pack.
 
 
 
 The maps for the 4 connectors are shown below.  Since we are only testing the motors today
 
 we only need to be concerned with IN11,IN2,IN3,IN4.
 
 
 
 
 
 6x2 pin header
 
 -------
 
 PTB0  ---------         PTE20
 
 B1          |     |     |           E21               *** ping trigger GPIO E21
 
 B2          |     |     |           E22               *** ping echo T2, channel 0
 
 B3          |     |     |           E23         7A  *** opto sensor 1 ADC7A         
 
 C2          |     |     |           E29         4B  *** opto sensor 2 ADC4B         
 
 C1          |     |     |           E30               *** IN4 motor control T0, channel 3
 
 ---------
 
 10x2 pin header               
 
 PTD7  ---------         PTE1
 
 D6*         |     |     |           E0        
 
 NC          |     |     |           AREF
 
 E31         |     |     |           GND       
 
 A17         |     |     |           D1*       
 
 A16         |     |     |           D3        
 
 C17*  |     |     |           D2        
 
 C16*  |     |     |           D0*       
 
 C13*  |     |     |           D5*            
 
 C12*  |     |     |           A13       
 
 ---------    
 
 
 
 8x2 pin header    (left side)  
 
 
 
 SDA_D5      ---------         PTB8*
 
 3V3         |     |     |           B9*
 
 RST_A20     |     |     |           B10*
 
 3V3         |     |     |           B11*      
 
 JMPVIN            5V_Reg      |     |     |           E2*
 
 GND         |     |     |           E3*
 
 GND         |     |     |           E4*
 
 JMP5V_Reg   VIN         |     |     |           E5*
 
 ---------      
 
 8x2 pin header    (right side)          
 
 
 
 PTC11 ---------         PTC9*
 
 C10         |     |     |           C8*
 
 C6*         |     |     |           A5*  *** IN1 motor control T0, channel 2
 
 C5*         |     |     |           A4*         *** servo1 motor control T0, channel 1. Internal to proto board
 
 C4*         |     |     |           A12*  *** IN3 motor control T1, channel 0
 
 C3          |     |     |           D4*   *** IN2 motor control T0, channel 4
 
 C0          |     |     |           A1**
 
 C7*         |     |     |           A2**
 
 ---------      
 
 *   Connected to a feature on the proto or FRDM board.  CHECK SCHEMATICS!!!
 
 **
 
 
 
 
 
 // *****  Local Motor Routines   *********
 
 void init_motors(void)
 
 // A5:  Connected to timer T0, channel 2
 
 // D4:  Connected to timer T0, channel 4
 
 // A12:  Connected to timer T1, channel 0
 
 // E30:  Connected to timer T0, channel 3    
 
 
 // Drive routines:  Use duty cycle as argument.  Ex, 100% use 100
 
 * Seems to work ok until the argument is 20% 
 
 void driveForward(uint16_t,uint16_t);
 
 void driveReverse(uint16_t,uint16_t);
 
 
 
 
 
 */

#include "derivative.h" /* include peripheral declarations */

#include "mcg.h" /* using XTAL and PLL does not work returns errors*/

#include "my_timers.h"

#include "my_gpio.h"

#include "my_interruptHandlers.h"

#include "my_PWM.h"

#include "my_pBsLeds.h"

#include "my_ssd1306.h"

#include "my_adc.h"

//**********************************************************/

/**************Defines Here************************/

//**********************************************************/
//**********************************************************/
/******* Prototype Function declarations ******/

//**********************************************************/
void init_motors(void);

void driveForward(uint16_t, uint16_t);

void driveReverse(uint16_t, uint16_t);

void LCD_numchar_out(uint8_t);

void LCD_num16_out(uint16_t);

void LCD_num32_out(uint32_t);

void LCD_mem_out(uint32_t);

uint32_t bcdConv(uint32_t);

uint32_t pingglobal;

int adcValX;
int adcValY;
int adcValXfin;
int adcValYfin;

void init_ping_interrupt(void);

int ping(void);

int counter = 0;
int count = 0;

void init_ping(void);

int ping2(void);

int ping_checker;

void FSM_drive( datStruct);

void FSM_drive2( datStruct);

typedef struct

{

uint8_t pB1_;

uint8_t pB2_;

uint8_t pB3_;

uint8_t pB4_;

uint8_t Lsense;

uint8_t Rsense;

int ping_;

} datStruct;

//**********************************************************/

/**************Global Variables Here************************/

int optoR;

int optoL;

uint8_t prev_LS;

uint8_t numbuff[16];

//**********************************************************/

//uint8_t pB1_, pB2_, pB3_, pB4_;
//0x4004B010
// 0x00000103

int main(void) {

//**********************************************************/

//*******Local Variable Declarations for main *****************

//**********************************************************/

uint32_t T1 = 0;

uint32_t T2 = 0;

datStruct myDat;

//**********************************************************/

//******Initial Chip Settings******************

//**********************************************************/

//  The student user should not make changes here unless

//explicitly instructed to do so

pll_init(8000000, 0, 1, 4, 24, 1); //Use the crystal oscillator to drive the PLL

//******User chip module initializations***************************************

init_motors();

/*   Initialize the ports for output LEDs C12,13,16,17 */

/*   Initialize the ports for input PBs C4,5,6,7 */

initpBsLeds();

init_adc0();

init_ping_interrupt();

// init_ping();

//  IMPORTANT NOTE!!!  Modify the RVR value in my_interruopts.c so systicks is increments every 1ms, not every 5 ms

init_sysTicks();

delay_ms(1000);

//**********************************************************/      

//*************The infinte loop.  Could use while(1);

//**********************************************************/

for (;;) {

//driveForward(0,0);

//  Test your sensors first before the car.  Use the debugger to see if state_opt values are being set.
//  IMPORTANT NOTE!!!  Modify the pBx() routines in mypBsLeds.c to return a 1 if pushed and a 0 if not pushed!!!

optoR = gpi_c(5);
optoL = gpi_c(4);

myDat.pB1_ = pB1(); // These functions pB1() are in the included file mypBsLeds.h
myDat.pB2_ = pB2();   // They return a 1 of button pushed or a 0 if not
myDat.pB3_ = pB3();
myDat.pB4_ = pB4();
myDat.Lsense = optoL;
myDat.Rsense = optoR;

if (sysTicks > counter + 5) {
static int movAvg;
ping();

movAvg *= 7;    // x7/8  
movAvg = (movAvg >> 3);
movAvg += (T2Ticks >> 3);
counter = sysTicks;
myDat.ping_ = movAvg;
ping_checker = movAvg;

}

//myDat.ping_=ping2();   //make sure it works.  What distance corresponds to what number?

//pingglobal = ping2();

//myDat.ping_=ping();   //make sure it works.  What distance corresponds to what number?

FSM_drive(myDat);

while (sysTicks < T1)
;
T1 = sysTicks + 10;

if (sysTicks > T2) {
//setLocation(0,0);                                        
T2 = sysTicks + 500;
//LCD_num32_out(myDat.ping_);
}

}

return 0;

}

void FSM_drive(datStruct mD) {

typedef enum {

reset, idle, right, straight, left, stopT, pingStop, pause, FreeDrive

} states;

static states state = reset;

static int count_track;

static int StateTracker;

static uint16_t count;

static uint16_t count1;

static int increment;

switch (state) {

case reset: {

count = 0;

state = idle;

}

break;

case idle: {       //Wait for pB input

count_track = 0;

StateTracker = 0;

driveForward(0, 0);

driveReverse(0, 0);

gpo_c(13, 0);
gpo_c(16, 0);

gpo_c(12, 1);
gpo_c(17, 1);

if (mD.pB4_ == 0)
state = straight;
else if (mD.pB3_ == 0)
state = FreeDrive;

}

break;

case straight: {

StateTracker = 1;

count = 0;

count1 = 0;

gpo_c(13, 1);
gpo_c(16, 1);
gpo_c(12, 0);
gpo_c(17, 0);

if (mD.ping_ < 8000) {
count = 0;
state = pingStop;
}

if (mD.Lsense == 1 && mD.Rsense == 1) {
count_track++;
state = stopT;
}

else if (mD.Lsense == 1) { //ALTERNATE: if (LS1 == 0) {

increment = increment + 15;

driveForward(15, 50 + increment); //increment right wheel Power

}

else if (mD.Rsense == 1) { //ALTERNATE: if (LS2 == 0) {

increment = increment + 15;

driveForward(50 + increment, 15); //increment left wheel Power

}

else if (mD.pB3_ == 0) {

state = idle;

}

else {

driveForward(30, 30);

increment = 0;

}

}

break;

/*case stopT: { // use this to turn, we want to count how many times our LS1 or LS2 changes to know when to stop turning
 
int countLS = 0; //move this to outside states
 
//driveForward(0, 0);
 
gpo_c(12, 1);
 
gpo_c(13, 1);
 
PWM_1_0_pw(0 * 100);   //A12
 
PWM_0_2_pw(40 * 100);  //A5 wheel 1 forward
 
PWM_0_3_pw(40 * 100);
 
PWM_0_4_pw(0 * 100);
 
//driveForward(0,25);
 
//driveReverse(25,0);
 
//E30 wheel 2 reverse
 
// exit condition
 
if (prev_LS != mD.Lsense) { //when we enter this state, mD.Lsense == 1, so can't we just say if mD.Lsense == 1 count++
 
countLS++;
 
prev_LS = mD.Lsense;
 
}
 
else if (countLS == 3) {
 
state = straight;
 
}
 
if (mD.pB4_ == 1) {
 
state = straight;
 
}
 
}*/

//break;
//test
case stopT: {

StateTracker = 2;

gpo_c(12, 0);
gpo_c(13, 0);
gpo_c(16, 1);
gpo_c(17, 1);

//if (mD.Lsense == 1 && mD.Rsense == 1)

//driveReverse(50,50);

if (count_track >= 2) {
state = idle;
}

else if (count < 50) {
driveReverse(30, 30);
count++;

}

else if (mD.Rsense == 1) {
StateTracker = 10;
state = left;

}

else if (mD.pB3_ == 0)
state = idle;

else {
PWM_1_0_pw(0 * 100);   //A12

PWM_0_2_pw(30 * 100);  //A5 wheel 1 forward

PWM_0_3_pw(30 * 100);

PWM_0_4_pw(0 * 100);
}
}

break;

case left: {

if (mD.Lsense == 1) {

StateTracker = 3;
//increment = increment + 15;
//driveForward(0, 0);
PWM_1_0_pw(30 * 100);   //A12

PWM_0_2_pw(30 * 100);  //A5 wheel 1 forward

PWM_0_3_pw(30 * 100);

PWM_0_4_pw(30 * 100);
//increment left wheel Power
count1++;
if (count1 > 100) {
state = straight;
}
} else {
PWM_1_0_pw(0 * 100);   //A12

PWM_0_2_pw(30 * 100);  //A5 wheel 1 forward

PWM_0_3_pw(30 * 100);

PWM_0_4_pw(0 * 100);
}
}

break;

case pingStop: {
PWM_1_0_pw(40 * 100);   //A12

PWM_0_2_pw(40 * 100);  //A5 wheel 1 forward

PWM_0_3_pw(40 * 100);

PWM_0_4_pw(40 * 100);

gpo_c(12, 1);
gpo_c(13, 1);
gpo_c(16, 0);
gpo_c(17, 0);
if (count < 1000) //wait 2 seconds??
{
count++;
}
if (mD.pB3_ == 0)
state = idle;
else if (count >= 500) {
if (mD.ping_ < 8000) {
count_track++;
state = stopT;
} else
state = straight;
}
}
break;

case FreeDrive: {
gpo_c(12, 0);
gpo_c(13, 0);
gpo_c(16, 0);
gpo_c(17, 0);
adcValX = adc0_value(7, 'a') - 2500;
adcValY = adc0_value(4, 'b') - 2600;

if (abs(adcValX) < 600) {
adcValXfin = 0;
} else if (adcValX < 0) {
adcValXfin = adcValX / 3;
} else {
adcValXfin = adcValX;
}
if (abs(adcValY) < 600){
adcValYfin = 0;
} else if (adcValY < 0) {
adcValYfin = adcValY / 3;
} else {
adcValYfin = adcValY;
}

if (adcValYfin < 0) {
driveReverse(abs(adcValYfin), abs(adcValYfin));
} else if (adcValXfin > 0) {
driveForward(abs(adcValYfin + adcValXfin), abs(adcValYfin));
} else if (adcValXfin < 0) {
driveForward(abs(adcValYfin), abs(adcValYfin + adcValXfin));
}else if(adcValXfin == 0){
driveForward(adcValYfin,adcValYfin);
}else
driveForward(0,0);

if (mD.pB3_ == 0)
state = idle;

}
break;

default:
break;

}

}

/*     init motors 
 
 *
 
 * This function will initialize 2 DC motors driven by an h-bridge
 
 *
 
 *Motor 1:  attached to A5,A12
 
 *Motor 2:  attached to D4,E30
 
 
 Using timers T0 and T1
 
 
 A5:  Connected to timer T0, channel 2
 
 A12:  Connected to timer T1, channel 0
 
 D4:  Connected to timer T0, channel 4
 
 E30:  Connected to timer T0, channel 3
 
 
 
 
 */

void init_motors(void) {

init_PWM_0_2(10000);  // A5:  Connected to timer T0, channel 2
init_PWM_0_4(10000);  // D4:  Connected to timer T0, channel 4
init_PWM_1_0(10000);  // A12:  Connected to timer T1, channel 0
init_PWM_0_3(10000);   // E30:  Connected to timer T0, channel 3

}

void driveReverse(uint16_t w1, uint16_t w2) {
//Wheel 1
PWM_0_2_pw(0);          //A5
PWM_0_4_pw(w1 * 10);   //D4

//Wheel 2
PWM_1_0_pw(0);          //A12
PWM_0_3_pw(w2 * 10);   //E30
}

void driveForward(uint16_t w1, uint16_t w2) {
//Wheel 1
PWM_0_2_pw(w1 * 10);  //A5
PWM_0_4_pw(0);       //D4

//Wheel 2       
PWM_1_0_pw(w2 * 10);   //A12
PWM_0_3_pw(0);       //E30
}

void init_ping_interrupt(void) {

init_gpio_e(21, 1);  //E21 for trigger

gpo_e(22, 0);  //

init_inputCapture_2_0(1);  //E22 for echo

}

int ping() {
static int movAvg;
gpo_e(21, 1);  //E21 for trigger
delay_10us();
delay_10us();
gpo_e(21, 0);  //E21

}

void init_ping(void) {

init_gpio_e(21, 1);  //E21 for trigger

gpo_e(22, 0);  //

init_gpio_e(22, 0);    //E22 for echo

}

int ping2() {

int Techo = 0;

static int moveAvg = 0;

gpo_e(21, 1);  //E21 for trigger

delay_10us();

delay_10us();

gpo_e(21, 0);  //E21

while (gpi_e(22) == 0)
;

while (gpi_e(22) == 1) {

delay_10us();

Techo++;

}

moveAvg *= 7;

moveAvg = (moveAvg >> 3);

moveAvg += (T2Ticks >> 3);

return Techo;

}

void LCD_numchar_out(uint8_t ch) {

uint8_t tmp;

tmp = ch;

tmp = tmp >> 4;

printChar(numbuff[tmp]);

ch = ch & 0x0f;

printChar(numbuff[ch]);

return;

}

void LCD_num32_out(uint32_t ch) {

uint32_t tmp;

tmp = ch;

tmp = tmp >> 24;

LCD_numchar_out(tmp);

tmp = ch;

tmp = tmp >> 16;

tmp = tmp & 0x00ff;

LCD_numchar_out(tmp);

tmp = ch;

tmp = tmp >> 8;

tmp = tmp & 0x00ff;

LCD_numchar_out(tmp);

tmp = ch;

tmp = tmp & 0x00ff;

LCD_numchar_out(tmp);

return;

}

void LCD_num16_out(uint16_t ch) {

uint16_t tmp;

tmp = ch;

tmp = tmp >> 8;

tmp = tmp & 0x00ff;

LCD_numchar_out(tmp);

tmp = ch;

tmp = tmp & 0x00ff;

LCD_numchar_out(tmp);

return;

}

void LCD_mem_out(uint32_t loc) {

uint32_t x;

x = *(uint32_t *) loc;      //prints out the location = value

//    LCD_numint_out(loc);

//    LCD_write_char('=');

LCD_num32_out(x);

return;

}

uint32_t bcdConv(uint32 num) {

uint32_t bcdResult = 0;

uint32_t shift = 0;

while (num > 0) {

bcdResult |= (num % 10) << (shift++ << 2);

num /= 10;

}

return bcdResult;

}


