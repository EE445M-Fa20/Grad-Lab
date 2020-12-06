/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "xil_types.h"
#include "Xscugic.h"
#include "Xil_exception.h"
#include "xscutimer.h"
#include "math.h"
#include "xgpiops.h"
#include "OS.h"

//timer info/
#define TIMER_DEVICE_ID     XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID      XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_INTR     XPAR_SCUTIMER_INTR

//gpio
#define  Gpio_device_ID		XPAR_PS7_GPIO_0_DEVICE_ID
#define SPWM_IO1 54    //emio
#define SPWM_IO2 55

//#define TIMER_LOAD_VALUE  1s = 0x13D92D3F sin 50Hz time = 1s/50Hz/256 spwm_cycyle = 2B81(78us)
#define TIMER_LOAD_VALUE    0x33333 //ÓÃ0.1Ãë²âÊÔ´®¿Ú´òÓ¡

static XScuGic Intc; //GIC
static XScuTimer Timer;//timer
static XGpioPs GpioPs;
static XGpioPs_Config *GpioCfg;

static void SetupInterruptSystem(XScuGic *GicInstancePtr,XScuTimer *TimerInstancePtr, u16 TimerIntrId);
static void TimerIntrHandler(void *CallBackRef);

//SPWM
u8 cnt_78us;
u8 spwm_high[] 	= {16,47,78,106,132,155,174,188,198,203,203,198,188,174,155,132,106,78,47,16};
u8 spwm_cycle[] = {241,241,242,243,244,246,248,250,252,255,255,252,250,248,246,244,243,242,241,241};

void gen_spwm(void){
	static u8 i = 0;
	static u8 test_H=0,test_L=0;

	if(spwm_high[i] >= cnt_78us){
		test_H++;
		test_L = 0;
		XGpioPs_WritePin(&GpioPs,SPWM_IO1,0x01);  //EMIO ¸ù¾ÝÐèÒªÈ¡Ïû×¢ÊÍ£¬·ÅÉÏÈ¥
		//printf(" H%d\n\r",test_H);

	}
	else if(spwm_high[i] < cnt_78us){
		test_H = 0;
		test_L++;
		XGpioPs_WritePin(&GpioPs,SPWM_IO1,0x0);
		//printf(" L%d\n\r",test_L);
	}

	if(cnt_78us >= 255){
		i++;
		if(i >= 20)
			i = 0;
	}
	OS_Sleep(100);
}

int IdleCount;
void Idle(){
	IdleCount = 0;
	while (1){
		IdleCount ++;
		XGpioPs_WritePin(&GpioPs,SPWM_IO1,0x01);
		XGpioPs_WritePin(&GpioPs,SPWM_IO1,0x00);
	}
}
int IdleCount2;
void Idle2(){
	IdleCount2 = 0;
	while (1){
		IdleCount2 ++;
		//XGpioPs_WritePin(&GpioPs,SPWM_IO2,0x01);
		//XGpioPs_WritePin(&GpioPs,SPWM_IO2,0x00);
	}
}

int Gpio_init(u16 Device_ID){

	int status;

	GpioCfg = XGpioPs_LookupConfig(Device_ID);
	status = XGpioPs_CfgInitialize(&GpioPs,GpioCfg,GpioCfg->BaseAddr);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}
	XGpioPs_SetDirectionPin(&GpioPs,SPWM_IO1,0x01);
	XGpioPs_SetOutputEnablePin(&GpioPs,SPWM_IO1,0x01);
	XGpioPs_SetDirectionPin(&GpioPs,SPWM_IO2,0x01);
	XGpioPs_SetOutputEnablePin(&GpioPs,SPWM_IO2,0x01);

	return XST_SUCCESS;
}


int main()
{
     XScuTimer_Config *TMRConfigPtr;     //timer config
     DisableInterrupts();
     printf("------------START-------------\n");
     Gpio_init(Gpio_device_ID);
     //Ë½ÓÐ¶¨Ê±Æ÷³õÊ¼»¯
     TMRConfigPtr = XScuTimer_LookupConfig(TIMER_DEVICE_ID);
     XScuTimer_CfgInitialize(&Timer, TMRConfigPtr,TMRConfigPtr->BaseAddr);
     XScuTimer_SelfTest(&Timer);

     //¼ÓÔØ¼ÆÊýÖÜÆÚ£¬Ë½ÓÐ¶¨Ê±Æ÷µÄÊ±ÖÓÎªCPUµÄÒ»°ã£¬Îª333MHZ,Èç¹û¼ÆÊý1S,¼ÓÔØÖµÎª1sx(333x1000x1000)(1/s)-1=0x13D92D3F
     XScuTimer_LoadTimer(&Timer, TIMER_LOAD_VALUE);

     //×Ô¶¯×°ÔØ
     XScuTimer_EnableAutoReload(&Timer);

     //Æô¶¯¶¨Ê±Æ÷
     XScuTimer_Start(&Timer);

     //set up the interrupts
     SetupInterruptSystem(&Intc,&Timer,TIMER_IRPT_INTR);

     OS_Init();
     OS_AddThread(&Idle2,128,1);
     NumCreated++;
     OS_AddThread(&Idle,128,5);
     NumCreated++;
     OS_Launch(2000);
     while(1){
//    	 gen_spwm();
     }

     return 0;

}

void SetupInterruptSystem(XScuGic *GicInstancePtr,XScuTimer *TimerInstancePtr, u16 TimerIntrId)
{
        XScuGic_Config *IntcConfig; //GIC config
        Xil_ExceptionInit();
        //initialise the GIC
        IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
        XScuGic_CfgInitialize(GicInstancePtr, IntcConfig,IntcConfig->CpuBaseAddress);

        //connect to the hardware
        Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler)XScuGic_InterruptHandler,
        		GicInstancePtr);

        //set up the timer interrupt
        XScuGic_Connect(GicInstancePtr, TimerIntrId,
                        (Xil_ExceptionHandler)TimerIntrHandler,
                        (void *)TimerInstancePtr);

        //enable the interrupt for the Timer at GIC
        XScuGic_Enable(GicInstancePtr, TimerIntrId);

        //enable interrupt on the timer
        XScuTimer_EnableInterrupt(TimerInstancePtr);

        // Enable interrupts in the Processor.
        Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

}

static void TimerIntrHandler(void *CallBackRef)
{
//    static int sec = 0;   //¼ÆÊý

    XScuTimer *TimerInstancePtr = (XScuTimer *) CallBackRef;
    XScuTimer_ClearInterruptStatus(TimerInstancePtr);
    cnt_78us++;
//    if(cnt_78us % 2 == 0){
//    	XGpioPs_WritePin(&GpioPs,SPWM_IO1,0x01);
//    }
//    else{
//    	XGpioPs_WritePin(&GpioPs,SPWM_IO1,0x0);
//    }
//    gen_spwm();
    SysTick_Handler();
    if(cnt_78us > 255)
    	cnt_78us = 0;
//    printf(" %d Second\n\r",cnt_78us);  //Ã¿Ãë´òÓ¡Êä³öÒ»´Î

}
