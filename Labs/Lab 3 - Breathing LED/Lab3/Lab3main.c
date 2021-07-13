#include <stdint.h>
#include <stdlib.h>
// Constant declarations to access port registers
// using symbolic names instead of addresses
#define GPIO_PORTE_DATA_R     (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R      (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R    (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R      (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R      (*((volatile unsigned long *)0x4002451C))

#define GPIO_PORTE_AMSEL_R    (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R     (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R        (*((volatile unsigned long *)0x400FE108))
	
// Global variables
volatile unsigned long switch_1;

// Function prototypes
void PortEInit(void);             // port E initial fuction
void delay(int sec);							// delay function
void red(void);	// red led on-off dunction
void yellow(void);					// yellow led on-off dunction
void green(void);						// green led on-off dunction
volatile int k = 1;					// delay time controller variable
int cond;													// condition checker variable
volatile int st = 0;				// condition counter for toggle

int  main(void) {
    // Setup
    // initializes the real board grader for lab 4
    PortEInit();        
    // Loop
    while (1) {
        switch_1 = GPIO_PORTE_DATA_R & 0x10;  // read PE4 into SW1
				if(cond == 0){	// cond = 0 red on
					red();
				}
				
		}
		return 0;
}
			

 // Initializes port F pins for input and output.
void PortEInit(void) {
    SYSCTL_RCGC2_R    |= 0x00000010;     // 1) E clock
    GPIO_PORTE_AMSEL_R = 0x00;           // 3) disable analog function
    GPIO_PORTE_PCTL_R  = 0x00;           // 4) GPIO clear bit PCTL
    GPIO_PORTE_DIR_R  |= 0x0E;           // 5) PE4, PE0 inputs. PE3, PE2, PE1 outputs
    GPIO_PORTE_AFSEL_R = 0x00;           // 6) no alternate function
    GPIO_PORTE_PUR_R  |= 0x11;           // enable pullup resistors on PE4, PE0
    GPIO_PORTE_DEN_R  |= 0x1F;           // 7) enable digital pins PE4-PE0
}
	
void delay(int sec){
	int c = 1, d = 1;
	for( c = 1; c <= sec; c++)
		for( d = 1; d <= 40; d++){}//000---    0.001 saniye periyod 0,0001
		}

int time = 1200;
int op = 1200;
int cl=0;
int ctrl = 0;
int diff =5;
void red(void){	// red led control func.
	if(!switch_1){// if p4 pressed enter this loop and increase delay by +1
			GPIO_PORTE_DATA_R = 0x0A; 
			delay(20000);
			GPIO_PORTE_DATA_R = 0x00; 
			delay(80000);
	}
	else{
		while(switch_1){
			GPIO_PORTE_DATA_R = 0x06; 
			delay(op);
			if(ctrl%2 ==0){
			op = op -diff;
			}
			GPIO_PORTE_DATA_R = 0x00; 
			delay(cl);
			if(ctrl%2 ==0){
			cl = cl+diff;
			}
			if(cl>=time){
				delay(200000);
				break;
			}
			ctrl++;
		}
		while(switch_1){
			GPIO_PORTE_DATA_R = 0x06; 
			delay(op);
			if(ctrl%2 ==0){
			op = op +diff;
			}
			GPIO_PORTE_DATA_R = 0x00; 
			delay(cl);
			if(ctrl%2 ==0){
			cl = cl-diff;
			}
			if(op>=time){
				break;
			}
			ctrl++;
		}
	}
}
