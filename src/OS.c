// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "OS.h"


#define PD3                    (*((volatile uint32_t *)0x40007020))
#define NVIC_SYSPRI14_R        (*((volatile uint32_t *)0xE000ED22))  //PendSV priority register (position 14).
#define NVIC_SYSPRI15_R        (*((volatile uint32_t *)0xE000ED23))  //Systick priority register (position 15).
#define NVIC_LEVEL14               0xEF                              //Systick priority value (second lowest).
#define NVIC_LEVEL15               0xFF                              //PendSV priority value (lowest).
#define NVIC_PENDSVSET             0x10000000                        //Value to trigger PendSV exception.



// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};

#define MAX_TCB_SIZE 13
#define MAX_STACK_SIZE 512
#define MAX_FIFO_SIZE 64

uint32_t OStime_round;												//variables defined for Lab 1, used to count rounds for timer5A,can be reset
uint32_t OStime_Task1;                        //variables defined for Lab 1, used to count Jitters,cannot be reset
uint32_t OStime_ms;														//variables defined for Lab 1, used to remember time in ms

Sema4Type LCDFree;
Sema4Type TxRoomLeft;
Sema4Type RxDataAvailable;

void(*SW_task)(void);				//hook for SW interrupt task
void(*PeriodicTask4)(void);	//hook for TIMER0, used for add period task

/*	FIFO variables	*/
struct FIFO{
	long queue[MAX_FIFO_SIZE];
	Sema4Type RoomRemain;
	Sema4Type DataAvailable;
	long max;
};
struct FIFO OS_FIFO;

/*	FIFO variables	*/
struct MailBox{
	long data;
	Sema4Type BoxFree;
	Sema4Type DataValid;
};
struct MailBox OS_MailBox;

uint32_t NumCreated;
/*	TCB variables	*/
struct TCB{
	long *SavedSP;
	struct TCB *Next;
	struct TCB *Previous;
	int32_t Id;
	uint32_t Sleep;
	uint32_t Priority;
	uint32_t Block;
};
typedef struct TCB TCBtype;

TCBtype TCB_pool[MAX_TCB_SIZE];
TCBtype *RunPt;
TCBtype *SchPt;
long STACK_pool[MAX_TCB_SIZE][MAX_STACK_SIZE];


void WDTimer0A_task(){
	//long temp = StartCritical();
	for (int i = 0;i<MAX_TCB_SIZE;i++){
		if (TCB_pool[i].Sleep>0)
			TCB_pool[i].Sleep--;
	}
	//EndCritical(temp);
}

/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 2 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/
void SysTick_Handler(void) {
	WDTimer0A_task();
	SchPt = RunPt->Next;
	while(SchPt->Sleep != 0){
		SchPt = SchPt->Next;
	}
	ContextSwitch();
  
} // end SysTick_Handler

unsigned long OS_LockScheduler(void){
  // lab 4 might need this for disk formating
  return 0;// replace with solution
}
void OS_UnLockScheduler(unsigned long previous){
  // lab 4 might need this for disk formating
}


void SysTick_Init(unsigned long period){

}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */

void OSTtask(void){  // runs at 1Hz in background
  OStime_round += 1;
	OStime_Task1 += 1;
}

void OS_Init(void){
  // put Lab 2 (and beyond) solution here
	OS_InitSemaphore(&LCDFree, 1);
	OS_InitSemaphore(&TxRoomLeft, 1024);
	OS_InitSemaphore(&RxDataAvailable, 0);
}; 


// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  // put Lab 2 (and beyond) solution here
	semaPt->Value = value;
}; 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
  DisableInterrupts();
	while(semaPt->Value <= 0){
		EnableInterrupts();
		DisableInterrupts();
	}
	semaPt->Value--;
	EnableInterrupts();
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long temp = StartCritical();
	semaPt->Value++;
	EndCritical(temp);
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	while(semaPt->Value == 0){
		EnableInterrupts();
		DisableInterrupts();
	}
	semaPt->Value = 0;
	EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	semaPt->Value = 1;
}; 



//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority){
  // put Lab 2 (and beyond) solution here
		 if (NumCreated < MAX_TCB_SIZE){

			 TCB_pool[NumCreated].SavedSP = &STACK_pool[NumCreated][MAX_STACK_SIZE-1];			//initiate SP information in TCB
			 
			 *(TCB_pool[NumCreated].SavedSP) = 0x01000000;
			 *(--TCB_pool[NumCreated].SavedSP) = (int) task;				//initiate stack
			 *(--TCB_pool[NumCreated].SavedSP) = (int) task;
			 *(--TCB_pool[NumCreated].SavedSP) = (int) task;
			 *(--TCB_pool[NumCreated].SavedSP) = (int) task;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x10101010;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x09090909;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x08080808;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x07070707;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x06060606;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x05050505;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x04040404;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x03030303;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x02020202;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x01010101;
			 *(--TCB_pool[NumCreated].SavedSP) = 0x00000000;
			 
			 //long temp = StartCritical();
			 if (NumCreated == 0){
				 TCB_pool[NumCreated].Next = &TCB_pool[NumCreated];
				 TCB_pool[NumCreated].Previous = &TCB_pool[NumCreated];
			 }
			 else{
				 TCB_pool[NumCreated].Next = &TCB_pool[0];
				 TCB_pool[NumCreated].Previous = &TCB_pool[NumCreated - 1];
				 TCB_pool[NumCreated].Previous->Next = &TCB_pool[NumCreated];
				 TCB_pool[0].Previous = &TCB_pool[NumCreated];
			 }
			 //EndCritical(temp);
			 return 1;
/*				 
			 switch (NumCreated){
				 case 0:{
					 TCB_pool[NumCreated].Previous = &TCB_pool[2];
					 TCB_pool[NumCreated].Next = &TCB_pool[1];
					 return 1;
				 }
				 case 1:{
					 TCB_pool[NumCreated].Previous = &TCB_pool[0];
					 TCB_pool[NumCreated].Next = &TCB_pool[2];
					 return 1;
				 }
				 case 2:{
					 TCB_pool[NumCreated].Previous = &TCB_pool[1];
					 TCB_pool[NumCreated].Next = &TCB_pool[0];
					 return 1;
				 }
				 default:
					 return 0;
			 };
*/		 
		 }
		 else
		 return 0;

};

//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority){
  // put Lab 5 solution here

     
  return 0; // replace this line with Lab 5 solution
}


//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  
  return 0; // replace this line with solution
};

//******** Timer0 Interrupt *************** 
//used for add period background thread
void Timer4A_Handler(void){

}
//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority){
	return 0;
};


/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
void GPIOPortF_Handler(void){

}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
	return 0;
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
    
  return 0; // replace this line with solution
};


// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // put Lab 2 (and beyond) solution here
	long temp = StartCritical();
  RunPt->Sleep = sleepTime;
	SchPt = RunPt->Next;
	while(SchPt->Sleep != 0){
		SchPt = SchPt->Next;
	}
//	NVIC_INT_CTRL_R = NVIC_PENDSVSET; //NVIC_PENDSVSET = 0x10000000
	EndCritical(temp);
};  

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	RunPt->Previous->Next = RunPt->Next;
	RunPt->Next->Previous = RunPt->Previous;
	NumCreated --;
	SchPt = RunPt->Next;
	while(SchPt->Sleep != 0){
		SchPt = SchPt->Next;
	}
//	NVIC_INT_CTRL_R = NVIC_PENDSVSET; //NVIC_PENDSVSET = 0x10000000
  EnableInterrupts();   // end of atomic section 
  for(;;){};        // can not return
    
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  // put Lab 2 (and beyond) solution here
  ContextSwitch();
};
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(uint32_t size){
  // put Lab 2 (and beyond) solution here
	int i;
	for (i=size;i>=1;i--){
		OS_FIFO.queue[size-1] = 15;
	}
  OS_FIFO.RoomRemain.Value = size;
	OS_FIFO.DataAvailable.Value = 0;
	OS_FIFO.max = size;
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  // put Lab 2 (and beyond) solution here
	if(OS_FIFO.RoomRemain.Value == 0){
		return 0;
	}
	OS_Wait(&OS_FIFO.RoomRemain);
	DisableInterrupts();
	OS_FIFO.queue[OS_FIFO.RoomRemain.Value] = data;
	EnableInterrupts();
	OS_Signal(&OS_FIFO.DataAvailable);
	return 1; // replace this line with solution
};  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  // put Lab 2 (and beyond) solution here
	int data_temp;int i;
	OS_Wait(&OS_FIFO.DataAvailable);
	DisableInterrupts();
	data_temp = OS_FIFO.queue[OS_FIFO.max-1];
	for (i=OS_FIFO.max;i>OS_FIFO.RoomRemain.Value;i--){
		OS_FIFO.queue[i-1] = OS_FIFO.queue[i-2];
	}
	EnableInterrupts();
	OS_Signal(&OS_FIFO.RoomRemain);

  return data_temp; // replace this line with solution
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
   
  return 0; // replace this line with solution
};


// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  // put Lab 2 (and beyond) solution here
  OS_MailBox.data = 0;
	OS_MailBox.BoxFree.Value = 1;
	OS_MailBox.DataValid.Value = 0;
  // put solution here
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  // put Lab 2 (and beyond) solution here
  // put solution here
	OS_bWait(&OS_MailBox.BoxFree);
	OS_MailBox.data = data;
	OS_bSignal(&OS_MailBox.DataValid);
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  // put Lab 2 (and beyond) solution here
	int data_temp;
	OS_bWait(&OS_MailBox.DataValid);
	data_temp = OS_MailBox.data;
	OS_bSignal(&OS_MailBox.BoxFree);
  return data_temp; // replace this line with solution
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  // put Lab 2 (and beyond) solution here
	uint32_t OStime_us=0;
  return OStime_us; // replace this line with solution
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  // put Lab 2 (and beyond) solution here
  uint32_t diff_ns = (stop - start)*80 ;   //time in 12.5ns units, 0 to 4294967295
  return diff_ns; // output difference in 12.5ns units
                  // replace this line with solution
};


// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  // put Lab 1 solution here

};

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  // put Lab 1 solution here
  return OStime_ms; // replace this line with solution
};


//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
  // put Lab 2 (and beyond) solution here
	RunPt = &TCB_pool[NumCreated-1];
	StartOS(RunPt->SavedSP);
};

long StartCritical(){
//	asm (	"mrs    R0 , I"	);
	asm (	"cpsid		I"	);
	asm (	"bx			lr"		);
};
void EndCritical(int temp){
//	asm (	"msr    I, R0"	);
	asm (	"cpsie		I"	);
	asm (	"bx			lr"		);
};
void EnableInterrupts(){
	asm (	"cpsie		I"	);
	asm (	"bx			lr"		);
};
void DisableInterrupts(){
	asm (	"cpsid		I"	);
	asm (	"bx			lr"		);
};

void StartOS(){
    asm("LDR     R0, =RunPt");
    asm("LDR     R1, [R0]");
    asm("LDR     SP, [R1]");
    asm("POP     {R4-R11}");
    asm("POP     {R0-R3}");
    asm("POP     {R12}");
    asm("POP     {lr}");
    asm("POP     {lr}");
    asm("POP     {R1}");
    asm("cpsie   I");
    asm("bx      lr");
};
void ContextSwitch(){
	//asm("cpsid		I");
	asm("stmdb	sp!,{r0-r3,r12,lr}");//push r0-r3,r12,lr

	asm("PUSH		{R4-R11}");
	asm("LDR		R0,=RunPt");
	asm("LDR		R1,[R0]");
	asm("STR		SP,[R1]");
	asm("LDR		R1,=SchPt");
	asm("LDR		R2,[R1]");
	asm("STR		R2,[R0]");
	asm("LDR		SP,[R2]");
	asm("POP		{R4-R11}");
	asm("cpsie		I");

	asm("ldmia	sp!,{r0-r3,r12,lr}");//pop r0-r3,r12,lr
	asm("BX			LR");
};
void PendSV_Handler(){
	;
};
