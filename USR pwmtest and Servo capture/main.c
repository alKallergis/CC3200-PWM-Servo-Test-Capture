//Copyright (c) 2019 Alex Kallergis

/* This project uses timerA2 edge time mode to capture 2 servo signals on P15(TIMERA) and P5(TIMERB),
 * and timerA3 PWM mode to output 2 conjugate PWM servo signals on P1(TIMERA) and P2(TIMERB).
 * The capture from TIMERA2A is used to feed the PWM signals.
 *
 * This program uses a 20ms servo period and assumes max possible pulse for input capture =3ms!
 *
 * Connections for CC3200LAUNCH-XL:
 * P15--SERVO SIGNAL FOR TIMERA2A
 * P5--     -//-         TIMERA2B
 * P1--SERVO OUTPUT FROM TIMARA3A
 * P2--     -//-         TIMERA3B
 *
 * */

#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "rom.h"
#include "rom_map.h"
#include "timer.h"
#include "utils.h"
#include "prcm.h"
#include "gpio.h"
#include "timer_if.h"
#include "hw_timer.h"
#include "pin_mux_config.h"

#define Period_ticks 0x186a00   //TimerA2 ticks for a 20ms period
#define TICKS_FOR_3MS   240000  //need this to define a valid pulse
#define TICKS_FOR_1point5MS   120000

unsigned int i,x,c,n,samplesA[2],samplesB[2],invalid_pulses,valid_pulses;
unsigned int L,H;
float ms;
tBoolean above;

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static void BoardInit(void);
void TimA2AIntHandler(void);
void TimA2BIntHandler(void);
void configTA2(void);
void configTA3(void);

//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

static void BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void TimA2AIntHandler(void){
//    if (GPIOPinRead(GPIOA0_BASE,GPIO_PIN_0)&0x1){ //CCP INPUT       //!! DOESN'T WORK
    if (TimerIntStatus(TIMERA2_BASE,true)==TIMER_CAPA_EVENT){   //  has happened
        TimerIntClear(TIMERA2_BASE,TIMER_CAPA_EVENT);
        samplesA[1]=samplesA[0];
        samplesA[0]=TimerValueGet(TIMERA2_BASE, TIMER_A);
        if ((samplesA[1]-samplesA[0])<TICKS_FOR_3MS){
            i=samplesA[1]-samplesA[0];    //update pwm match registers with new captured pulse
            valid_pulses++;
            ms=i*12.5/1000000;  //in milliseconds
        }
        else{
            invalid_pulses++;
        }
        //L=HWREG(TIMERA2_BASE + TIMER_O_TAR);    //read LSBS of capture value as per datasheet. Actually captures all 32 bits.
                                                //Also, actually the TnR doesn't capture the edge ,and rolls along with TnV instead..
        //H=HWREG(TIMERA2_BASE + TIMER_O_TAPR);   //read MSBS -//-.. Doesn't work,is always read 255 here
    }
}
void TimA2BIntHandler(void){
    if (TimerIntStatus(TIMERA2_BASE,true)==TIMER_CAPB_EVENT){   //  has happened
        TimerIntClear(TIMERA2_BASE,TIMER_CAPB_EVENT);
        samplesB[1]=samplesB[0];
        samplesB[0]=TimerValueGet(TIMERA2_BASE, TIMER_B);
        if ((samplesB[1]-samplesB[0])<TICKS_FOR_3MS){
            c=samplesB[1]-samplesB[0];
            ms=c*12.5/1000000;  //in milliseconds
            if (c > TICKS_FOR_1point5MS){     //signal more than half,assert boolean
                above=true;
            }
            else{above=false;}
        }
    }
}

void configTA2(void){
    Timer_IF_Init(PRCM_TIMERA2,TIMERA2_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME|TIMER_CFG_B_CAP_TIME),
                  TIMER_BOTH,0xff); //TIMER0 A&B CAPTURE, T=209.7152 ms
    TimerControlEvent(TIMERA2_BASE,TIMER_BOTH,TIMER_EVENT_BOTH_EDGES);       //capture both edges in both timers
    TimerIntRegister(TIMERA2_BASE, TIMER_A, TimA2AIntHandler);
    TimerIntRegister(TIMERA2_BASE, TIMER_B, TimA2BIntHandler);
    TimerIntEnable(TIMERA2_BASE,TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);      //INT capture event enable
    TimerControlStall(TIMERA2_BASE, TIMER_BOTH,true);
    TimerLoadSet(TIMERA2_BASE,TIMER_BOTH,0xffff);  //for T=209.7152 ms

}
void configTA3(void){
    i=0x1869f0;     //initial match value
    TimerConfigure(TIMERA3_BASE,(TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM));
    TimerPrescaleSet(TIMERA3_BASE,TIMER_A,0x18);    //209.7152 ms max T,here 20ms
    TimerLoadSet(TIMERA3_BASE,TIMER_A,0x6a00); //20ms
    TimerMatchSet(TIMERA3_BASE,TIMER_A,0x69f0);    //~0% dc
    TimerPrescaleMatchSet(TIMERA3_BASE,TIMER_A,0x18);
    TimerConfigure(TIMERA3_BASE,(TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM));
    TimerPrescaleSet(TIMERA3_BASE,TIMER_B,0x18);    //209.7152 ms max T,here 20ms
    TimerLoadSet(TIMERA3_BASE,TIMER_B,0x69f0); //20ms
    TimerMatchSet(TIMERA3_BASE,TIMER_B,10);    //~100% dc
    TimerPrescaleMatchSet(TIMERA3_BASE,TIMER_B,0x0);
}

void main()
{
    //
    // Board Initialisation
    //
    BoardInit();
    
    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();
    configTA2();        //timer for capture
    configTA3();        //timer for PWM signals

    while(x!=0x20){     //wait for button to start
    x=GPIOPinRead(GPIOA1_BASE,GPIO_PIN_5);
    }
    TimerEnable(TIMERA2_BASE,TIMER_BOTH);
    TimerEnable(TIMERA3_BASE,TIMER_BOTH);
    while(1)
    {   //create 2 conjugate/inverted signals:
        TimerMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-i)&0xffff));
        TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-i)&0xff0000)>>16);
        TimerMatchSet(TIMERA3_BASE, TIMER_B,(Period_ticks-TICKS_FOR_3MS+i)&0xffff);
        TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B,((Period_ticks-TICKS_FOR_3MS+i)&0xff0000)>>16);
        MAP_UtilsDelay(300);
        //c=TimerValueGet(TIMERA3_BASE, TIMER_A);   //see the pwm timer values
                //n=TimerValueGet(TIMERA3_BASE, TIMER_B);

        n++;    //keep track of cycles
    }
}
