#include <Prokatim2015cfg.h>
#include <csl.h>
#include <csl_mcbsp.h>
#include <csl_irq.h>
#include <csl_edma.h>
#include <dsk6713_led.h>
#include <log.h>
#include <math.h>
#include "config_AIC23.h"
#include "skeleton.h"

#define		TWO_PI				(2*3.1415926)
#define		FS					48000
#define 	START_FREQ			1000						//sweep start frequency
#define		END_FREQ			10000						//sweep end frequency
#define 	STEP_FREQ			6							//frequency step size
#define 	TS					0.00002083					//seconds
#define     GAIN                20000
#define     SAMPLES_IGNORE      180                        //ignored sampels


signed short transmitBuffer[3000], receiveBuffer[3500], delay=0, i=0,j = 0, l = 0;

int k=0;
float sample1 = 1, sample2 = 0, sample0 = 0, amplitudeCompensation = 0, A;

void config_interrupts(void)
{
	IRQ_globalDisable();

	IRQ_map(IRQ_EVT_XINT1, 12);
	IRQ_clear(IRQ_EVT_XINT1);
	IRQ_enable(IRQ_EVT_XINT1);

	IRQ_map(IRQ_EVT_RINT1, 13);
	IRQ_clear(IRQ_EVT_RINT1);
	IRQ_enable(IRQ_EVT_RINT1);

	IRQ_nmiEnable();
	IRQ_globalEnable();
}

void calculateCoefficients(short freq)
{
    float w;
    w = TWO_PI * freq;
	A = 2 * cos(w*TS);
	amplitudeCompensation = (sqrt((sample2*sample2) + (sample1*sample1) - (2*sample1*sample2*cos(w*TS)))) / sin(w*TS);
}

	
void generateTransmitBuffer()
{
    short counter = 0, freq = START_FREQ;

    calculateCoefficients(START_FREQ);

    while(counter < 3000)
    {
    	if(counter == 1499)
    	{
    		LOG_printf(&ProkatimLog, " ");

    	}
	    sample0 = ((A*sample1) - sample2) / amplitudeCompensation;
	    sample2 = sample1;
	    sample1 = sample0;
	    transmitBuffer[counter] = transmitBuffer[counter+1] = GAIN * sample0;
	    //LOG_printf(&ProkatimLog, "transmitBuffer[%d] = %d \n", counter, transmitBuffer[counter]);
	    counter+=2;
	    freq += STEP_FREQ;
	    calculateCoefficients(freq);
	}

}

 void TX_interrupt_service()
 {


 	if(i <= 3000)
 	{
 		MCBSP_write(hMcbsp, transmitBuffer[i]);
 		i++;
 	}

 	if(MCBSP_xsyncerr(hMcbsp))
 	{
 		LOG_printf(&ProkatimLog, "XSync error! ");
 	}

 	if(MCBSP_rsyncerr(hMcbsp))
 	{
 		LOG_printf(&ProkatimLog, "RSync error! ");
 	}
 }

 void RX_interrupt_service()
 {
 	short rec;
	 if(i>SAMPLES_IGNORE)
 	{
 		if( j <= 3500 )
 		{
 			//receiveBuffer[j] = receiveBuffer[j+1] = MCBSP_read(hMcbsp);
 			rec = MCBSP_read(hMcbsp);
 			if(l%2 == 0)
 			{
 				receiveBuffer[j] = rec;
 				j++;
 			}
 			l++;
 		}
 	}
 	else
 		{
 			receiveBuffer[0] = MCBSP_read(hMcbsp);
 		}
 }


short calculateCrossCorrelation(short a[], short b[])
 {    
	short i,j,delay=0,k=0;
    double max=0, sum;
 
    for( i=0; i<=2000; i++)
    {
    	k = 0;
    	for(j=0; j<=3000; j+=2)
    	{
    		sum = sum + (a[j] * b[k+i]);
    		if(sum > max)
    			{  
    				max=sum;
    				delay=i;
    			}
    		k++;
    		sum = 0;
    	}
   	}

   	return delay;
 }

float calcluateDistance(short delay)
{
	float distance;
	distance = (delay * TS ) * 340.29;
	return distance/2;
}

void main()
{
	float distance;
	CSL_init();  /* init CSL */

	/* Konfiguration des AIC23 über McBSP0 */
	Config_DSK6713_AIC23();
	//DSK6713_LED_init();


	/* Konfiguration des McBSP1 - Datenschnittstelle */
	hMcbsp = MCBSP_open(MCBSP_DEV1, MCBSP_OPEN_RESET);
    MCBSP_config(hMcbsp, &datainterface_config);

    /* interrupts immer zuletzt */
    config_interrupts();

    generateTransmitBuffer();

    MCBSP_start(hMcbsp,  MCBSP_XMIT_START | MCBSP_RCV_START, 220);
    MCBSP_write(hMcbsp, transmitBuffer[0]); /* einmal schießen */

    while( j <= 3500 );
    delay = calculateCrossCorrelation(transmitBuffer,receiveBuffer);

    distance = calcluateDistance(delay);

	LOG_printf(&ProkatimLog, " ");

}

