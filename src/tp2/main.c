/*************************************************************************
 * Programa de prueba basado para LM3S6965.
 *
 * Basado en el demo provisto por FreeRTOS, para ejecutar sobre QEMU.
 *
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "osram128x64x4.h"
#include "uart.h"
#include "bitmap.h"

/*-----------------------------------------------------------*/

/* Dimensions the buffer for text messages. */
#define mainMAX_MSG_LEN                     25

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT                ( 9 )
#define mainMAX_ROWS_128                    ( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96                     ( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64                     ( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE                      ( 15 )
#define ulSSI_FREQUENCY                     ( 3500000UL )

/* Tasks periods. */
#define TASK1_PERIOD 	3000
#define TASK2_PERIOD 	4000
#define TASK3_PERIOD 	6000

/* Tasks WCETs. */
#define TASK1_WCET		1000
#define TASK2_WCET		1000
#define TASK3_WCET		1000

/* Sistema a analizar */
#define SISTEMA			4
#define MAX_TASKS	10

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Basic polling UART write function.
 */
static void prvPrintString( const char * pcString );

/*
 * Busy wait the specified number of ticks.
 */
static void vBusyWait( TickType_t ticks );

/*
 * Periodic task.
 */
static void prvTask( void* pvParameters );

/*-----------------------------------------------------------*/

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( uint32_t ) = NULL;
void ( *vOLEDStringDraw )( const char *, uint32_t, uint32_t, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, uint32_t, uint32_t, uint32_t, uint32_t ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

/*-----------------------------------------------------------*/

struct xTaskStruct {
	TickType_t wcet;
	TickType_t period;
};

typedef struct xTaskStruct xTask;

xTask task1 = { 1000, 4000 };
xTask task2 = { 1000, 5000 };
xTask task3 = { 2000, 8000 };


xTask tasks[MAX_TASKS] = {};
int ts[10][MAX_TASKS][3] = {{
			{1, 4, 4}, {1, 5, 5}, {2, 8, 8}
		},
		{
			{1, 4, 4}, {1, 8, 8}, {2, 9, 9}
		},
		{
			{1, 4, 4}, {1, 6, 6}, {1, 8, 8}, {3, 13, 13}
		},
		{
			{1, 4, 4}, {2, 7, 7}, {1, 12, 12}, {2, 14, 14}
		},
		{
			{1, 5, 5}, {1, 9, 9}, {1, 10, 10}, {2, 15, 15}, {2, 16, 16}
		},
		{
			{1, 5, 5}, {1, 9, 9}, {1, 10, 10}, {2, 15, 15}, {2, 16, 16}
		},
		{
			{1, 6, 6}, {1, 8, 8}, {2, 11, 11}, {1, 15, 15}, {2, 17, 17}
		},
		{
			{1, 4, 4}, {1, 7, 7}, {2, 10, 10}, {2, 14, 14}
		},
		{
			{1, 4, 4}, {1, 8, 8}, {2, 10, 10}, {2, 14, 14}
		},
		{
			{1, 5, 5}, {1, 8, 8}, {2, 12, 12}, {1, 15, 15}, {1, 16, 16}
		},
		{
			{1, 5, 5}, {1, 8, 8}, {1, 12, 12}, {1, 13, 13}, {2, 16, 16}
		}
};

static const int numTasksPorSistema[10] = {
	3, /* Sistema 1 */
	3, /* Sistema 2 */
	4, /* Sistema 3 */
	4, /* Sistema 4 */
	5, /* Sistema 5 */
	5, /* Sistema 6 */
	4, /* Sistema 7 */
	4, /* Sistema 8 */
	5, /* Sistema 9 */
	5  /* Sistema 10 */
};

//long ts[3][3] = {{1, 4, 4}, {1, 5, 5}, {2, 8, 8}};

/*************************************************************************
 * Main
 *************************************************************************/
int main( void )
{
	/* Initialise the trace recorder. */
	const int numTasks = numTasksPorSistema[SISTEMA - 1];
	for (int i = 0; i < numTasks; i++) {
		tasks[i] = (xTask) {ts[SISTEMA - 1][i][0] * 1000, ts[SISTEMA - 1][i][1] * 1000};
	}

	vTraceEnable( TRC_INIT );

    prvSetupHardware();

    /* Map the OLED access functions to the driver functions that are appropriate
    for the evaluation kit being used. */
    //configASSERT( ( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK ) == SYSCTL_DID1_PRTNO_6965 );
    vOLEDInit = OSRAM128x64x4Init;
    vOLEDStringDraw = OSRAM128x64x4StringDraw;
    vOLEDImageDraw = OSRAM128x64x4ImageDraw;
    vOLEDClear = OSRAM128x64x4Clear;

    /* Initialise the OLED and display a startup message. */
    vOLEDInit( ulSSI_FREQUENCY );

    /* Print Hello World! to the OLED display. */
    static char cMessage[ mainMAX_MSG_LEN ];
    sprintf(cMessage, "Hello World!");
    vOLEDStringDraw( cMessage, 0, 0, mainFULL_SCALE );

    /* Print "Start!" to the UART. */
    prvPrintString("Start!\n\r");


    for (int i = 0; i < numTasks; i++) {
    	char s[3];
    	sprintf(s, "T%d", i + 1);
    	xTaskCreate( prvTask, s, configMINIMAL_STACK_SIZE + 50, (void*) &tasks[i], configMAX_PRIORITIES - 1 - i, NULL );
    }

    vTraceEnable( TRC_START );

    /* Launch the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }

    /* Set the clocking to run from the PLL at 50 MHz */
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

    /* Initialise the UART - QEMU usage does not seem to require this
    initialisation. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );
    UARTEnable( UART0_BASE );
}
/*-----------------------------------------------------------*/

static void prvPrintString( const char * pcString )
{
    while( *pcString != 0x00 )
    {
        UARTCharPut( UART0_BASE, *pcString );
        pcString++;
    }
}
/*-----------------------------------------------------------*/

static void vBusyWait( TickType_t ticks )
{
    TickType_t elapsedTicks = 0;
    TickType_t currentTick = 0;
    while ( elapsedTicks < ticks ) {
        currentTick = xTaskGetTickCount();
        while ( currentTick == xTaskGetTickCount() ) {
            asm("nop");
        }
        elapsedTicks++;
    }
}
/*-----------------------------------------------------------*/

void prvTask( void *pvParameters )
{
	char cMessage[ mainMAX_MSG_LEN ];
	unsigned int uxReleaseCount = 0;
	TickType_t pxPreviousWakeTime = 0;
	xTask *task = (xTask*) pvParameters;
	int instance = 1;

	for( ;; )
	{
        sprintf( cMessage, "S %s - %d - %d\n\r", pcTaskGetTaskName(NULL), instance, xTaskGetTickCount());

        prvPrintString( cMessage );

        vBusyWait( task->wcet );

		sprintf(cMessage, "E %s - %d - %d\n\r", pcTaskGetTaskName( NULL ), instance, xTaskGetTickCount());
        prvPrintString( cMessage );

		vTaskDelayUntil( &pxPreviousWakeTime, task->period );

		uxReleaseCount += 1;
		instance++;
	}

	vTaskDelete( NULL );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    volatile uint32_t ulSetTo1InDebuggerToExit = 0;
    {
        while( ulSetTo1InDebuggerToExit == 0 )
        {
            /* Nothing to do here.  Set the loop variable to a non zero value in
            the debugger to step out of this function to the point that caused
            the assertion. */
            ( void ) pcFile;
            ( void ) ulLine;
        }
    }
}

char* _sbrk_r (struct _reent *r, int incr)
{
    /* Just to keep the linker quiet. */
    ( void ) r;
    ( void ) incr;

    /* Check this function is never called by forcing an assert() if it is. */
    //configASSERT( incr == -1 );

    return NULL;
}

int __error__(char *pcFilename, unsigned long ulLine) {
    return 0;
}

