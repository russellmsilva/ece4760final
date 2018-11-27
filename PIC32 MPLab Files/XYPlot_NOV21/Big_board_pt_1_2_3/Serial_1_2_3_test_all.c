/*********************************************************************
 *  DDS demo code
 *  sine synth to SPI to  MCP4822 dual channel 12-bit DAC
 *********************************************************************
 * Bruce Land Cornell University
 * Sept 2017
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* ====== MCP4822 control word ==============
bit 15 A/B: DACA or DACB Selection bit
1 = Write to DACB
0 = Write to DACA
bit 14 : Don't Care
bit 13 GA: Output Gain Selection bit
1 = 1x (VOUT = VREF * D/4096)
0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12 SHDN: Output Shutdown Control bit
1 = Active mode operation. VOUT is available. 
0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
bit 11-0 D11:D0: DAC Input Data bits. 
*/
// A-channel, 1x, active
//#define DAC_config_chan_A 0b0011000000000000
//#define DAC_config_chan_B 0b1011000000000000
#define Fs 200000.0
#define two32 4294967296.0 // 2^32 
#define Finit 220.0

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_2_3.h"
// threading library
#include "pt_cornell_1_2_3.h"
// for sine
#include <math.h>

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
////////////////////////////////////

// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;

//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;

// string buffer
char buffer[60];

// actual scaled DAC 
volatile  int DAC_data;
static struct pt pt_param ;
static char character;
static struct pt pt1, pt2, pt3, pt4, pt_input, pt_output ;
char buffer[60];
// profiling of ISR
volatile int isr_enter_time, isr_exit_time;
//=============================

static void do_scan ()
{
//    // get the character
//        // yield until there is a valid character so that other
//        // threads can execute
//        //PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
//        //UARTSendDataByte(UART2, character);
//        //PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
//       // while(!UARTReceivedDataIsAvailable(UART2)){};
//        //character = UARTGetDataByte(UART2);
//        //max_chars is 64
//        sprintf(PT_send_buffer,"ATLED0=100\r");
//        PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
//        //int a = sscanf(PT_term_buffer, "%s", buffer);
//        PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
//        //char a;
//        //if(PT_timeout==0) {
//            //sscanf(PT_term_buffer, "%c", &a);
//        //}
//        
//		tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//		//sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(30, 160);
//		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
//        //sscanf(PT_term_buffer, "%s", buffer);
//		tft_writeString(PT_term_buffer);
//        
//        tft_setCursor(30, 170);
////		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
////		tft_writeString(PT_term_buffer);
//        sprintf(buffer,"Hello");
//        tft_writeString(buffer);
}

static PT_THREAD (protothread_param(struct pt *pt))
{
    PT_BEGIN(pt);
    static int pattern;
    mPORTBSetPinsDigitalOut(BIT_3 | BIT_4 | BIT_5 | BIT_7);    //Set port as output
    mPORTASetPinsDigitalIn(BIT_8);    //Set port as input
    EnablePullDownA(BIT_8);  // and turn on pull-down on inputs
    
    static unsigned long input_response;
    static int i, j;
    static int up;
    //RB3, RB4, RB5, RB7, RB8, 
	while (1) {
		PT_YIELD_TIME_msec(500);

		//input_response = mPORTAReadBits(BIT_8);
		mPORTAClearBits(BIT_8);     
        
        //on startup reset sensor position
        //BIT_7 ready bit
        //pattern = 8+16+32+128;
        mPORTBClearBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
        mPORTBSetBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
    
        tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
		//sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(30, 160);
		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
        //sscanf(PT_term_buffer, "%s", buffer);
		sprintf(buffer,"Hello Christina");
        tft_writeString(buffer);
       
        PT_YIELD_UNTIL(pt, mPORTBReadBits(BIT_8) == 256); 
        mPORTBClearBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
        
        tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
		//sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(30, 160);
		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
        //sscanf(PT_term_buffer, "%s", buffer);
		sprintf(buffer,"Hello Michelle");
        tft_writeString(buffer);
        
        // get the character
        // yield until there is a valid character so that other
        // threads can execute
        //PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
        //UARTSendDataByte(UART2, character);
        //PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
       // while(!UARTReceivedDataIsAvailable(UART2)){};
        //character = UARTGetDataByte(UART2);
        //max_chars is 64
        sprintf(PT_send_buffer,"ATLED0=100\r");
        PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
        //int a = sscanf(PT_term_buffer, "%s", buffer);
        PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
        //char a;
        //if(PT_timeout==0) {
            //sscanf(PT_term_buffer, "%c", &a);
        //}
        
		tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
		//sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(30, 160);
		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
        //sscanf(PT_term_buffer, "%s", buffer);
		tft_writeString(PT_term_buffer);
        
        tft_setCursor(30, 170);
//		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
//		tft_writeString(PT_term_buffer);
        sprintf(buffer,"Hello");
        tft_writeString(buffer);
        
        sprintf(PT_send_buffer,"ATLED0=0\r");
        PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
        PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
             
        up = 0;
        for (i = 0; i<6; i++)
        {
            for (j = 0; j<6; j++)
            {
                mPORTBClearBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
                mPORTAClearBits(BIT_8); 
                
                if (up)
                {
                    mPORTBSetBits(BIT_4);
                    mPORTBSetBits(BIT_7);
                }
                
                else
                {
                    mPORTBSetBits(BIT_5);
                    mPORTBSetBits(BIT_7);
                }
       
                PT_YIELD_UNTIL(pt, mPORTBReadBits(BIT_8) == 256); 
                mPORTBClearBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
                
                // get the character
            // yield until there is a valid character so that other
            // threads can execute
            //PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
            //UARTSendDataByte(UART2, character);
            //PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
           // while(!UARTReceivedDataIsAvailable(UART2)){};
            //character = UARTGetDataByte(UART2);
            //max_chars is 64
            sprintf(PT_send_buffer,"ATLED0=100\r");
            PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
            //int a = sscanf(PT_term_buffer, "%s", buffer);
            PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
            //char a;
            //if(PT_timeout==0) {
                //sscanf(PT_term_buffer, "%c", &a);
            //}

            tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            //sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(30, 160);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
            //sscanf(PT_term_buffer, "%s", buffer);
            tft_writeString(PT_term_buffer);

            tft_setCursor(30, 170);
    //		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
    //		tft_writeString(PT_term_buffer);
            sprintf(buffer,"Hello");
            tft_writeString(buffer);
            
            sprintf(PT_send_buffer,"ATLED0=0\r");
            PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
            PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
        
            }
            mPORTBClearBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
            mPORTAClearBits(BIT_8); 
            
            mPORTBSetBits(BIT_4 | BIT_5);
            mPORTBSetBits(BIT_7);
            PT_YIELD_UNTIL(pt, mPORTBReadBits(BIT_8) == 256); 
            mPORTBClearBits(BIT_3 | BIT_4 | BIT_5 | BIT_7);
             
                // get the character
            // yield until there is a valid character so that other
            // threads can execute
            //PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
            //UARTSendDataByte(UART2, character);
            //PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
           // while(!UARTReceivedDataIsAvailable(UART2)){};
            //character = UARTGetDataByte(UART2);
            //max_chars is 64
            sprintf(PT_send_buffer,"ATLED0=100\r");
            PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
            //int a = sscanf(PT_term_buffer, "%s", buffer);
            PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
            //char a;
            //if(PT_timeout==0) {
                //sscanf(PT_term_buffer, "%c", &a);
            //}

            tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            //sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(30, 160);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
            //sscanf(PT_term_buffer, "%s", buffer);
            tft_writeString(PT_term_buffer);

            tft_setCursor(30, 170);
    //		tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
    //		tft_writeString(PT_term_buffer);
            sprintf(buffer,"Hello");
            tft_writeString(buffer);
            
            sprintf(PT_send_buffer,"ATLED0=0\r");
            PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
            PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
            
            if (up)
            {
                up = 0;
            }
                
            else
            {
                up = 1;
            }
        }
	} // END WHILE(1)
	PT_END(pt);
} // thread 4



// === Main  ======================================================
// set up UART, timer2, threads
// then schedule them as fast as possible

int main(void)
{
    ANSELA = 0; ANSELB = 0; 
  /// timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // 400 is 100 ksamples/sec at 30 MHz clock
    // 200 is 200 ksamples/sec
    
    // === init the uart2 ===================
//    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
//    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
//    UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
//    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
//    printf("protothreads start..\n\r");
    PT_terminate_char = '\n' ;
    //PT_terminate_count = 1 ;
    PT_terminate_time = 5000;
  
    // SETUP OUTPUT COMPARE
    //OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 40000);
    // set up the timer interrupt with a priority of 2
    //ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    //mT2ClearIntFlag(); // and clear the interrupt flag

    //OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,
    //            pwm_on_time, pwm_on_time);
    /// SPI setup //////////////////////////////////////////
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    //PPSOutput(4, RPB9, OC3);
    
    // END SETUP COMPARE
    
    // control CS for DAC
    //mPORTBSetPinsDigitalOut(BIT_4);
    //mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    //SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
    
    // the LED
    //mPORTASetPinsDigitalOut(BIT_0);
    //mPORTASetBits(BIT_0);
    
    // BEGIN ADC ///////////////////////////////////////
    // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

    // define setup parameters for OpenADC10
    // ADC ref external  | disable offset test | disable scan mode | do 2 sample | use single buf | alternate mode on
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_ON
            //
    // Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    // SLOW it down a little
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

    // define setup parameters for OpenADC10
    // set AN11 and  as analog inputs
    #define PARAM4 ENABLE_AN1_ANA | ENABLE_AN5_ANA // 

    // define setup parameters for OpenADC10
    // do not assign channels to scan
    #define PARAM5 SKIP_SCAN_ALL //|SKIP_SCAN_AN5 //SKIP_SCAN_AN1 |SKIP_SCAN_AN5  //SKIP_SCAN_ALL

    // // configure to sample AN5 and AN1 on MUX A and B
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );

    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC
    // TO READ the two channels
    // angle_sensor_reading = ReadADC10(0);  //AN1 which is pin 3
    // setting = ReadADC10(1);  //AN5 which is pin 7
    
    // END ADC 
    
//    // SET UP BUTTON NOT IN MAIN. BIT_3 gives a value of 8!
//    mPORTBSetPinsDigitalIn(BIT_7 | BIT_8);    //Set port as input
//    // and turn on pull-down on inputs
//    EnablePullDownB(BIT_7 | BIT_8);
//    cycle_lcd  = mPORTBReadBits(BIT_7);
//    load_lcd  = mPORTBReadBits(BIT_8);
//    mPORTBClearBits(BIT_7 | BIT_8);
        
    // END BUTTONS
    
    // SPI setup for DAC A and B
    /// SPI setup //////////////////////////////////////////
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    //PPSOutput(2, RPB5, SDO2);
    // control CS for DAC
    //mPORTBSetPinsDigitalOut(BIT_4);
    //mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    //SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);    
    
    // === config the uart, DMA, vref, timer5 ISR =============
    PT_setup();

    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
  // === now the threads ====================

  // init the threads
  PT_INIT(&pt_param);
  
   // init the display
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  // erase
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(1); // Use tft_setRotation(1) for 320x240
  
  // schedule the threads
  while(1) {
    // round robin
    PT_SCHEDULE(protothread_param(&pt_param));
  }
} // main