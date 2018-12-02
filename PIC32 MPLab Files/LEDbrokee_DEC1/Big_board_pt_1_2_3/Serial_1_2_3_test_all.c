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

#include "string.h"
#include "stdlib.h"

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
char delimited[6][15];

// actual scaled DAC 
volatile  int DAC_data;
static struct pt pt_param ;
static char character;
static int reset;
static int adc_9;
static int prevadc_9;
static int left;
static int up;
static int steps_right, steps_left, max_right, max_left, max_down;
static int total_steps_down;
static int state;
static int fruit_ripe;
static int setting;
static int ripe_detections;
static int fruit_presence_counter;
static unsigned short color_reading;
static float running_average;
//Ring buffer of 680nm readings used to detect whether a fruit is ripe
static float ripe_buffer[8];
static int sensor_pos;
static struct pt pt1, pt2, pt3, pt4, pt_input, pt_output ;
char buffer[60];
char fruit[20];
char banana[20];
char apple[20];
char avocado[20];

#define MAX_BLUE 1400
#define MIN_BLUE 850
#define MAX_PINK 3500
#define MIN_PINK 2750

// profiling of ISR
volatile int isr_enter_time, isr_exit_time;
//=============================

//static void do_scan ()
//{
//    PT_terminate_char = '\n' ;
//    sprintf(PT_send_buffer,"ATLED1=100\r");
//    PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
//    PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
//    PT_YIELD_TIME_msec(100);
//    sprintf(PT_send_buffer,"ATCDATA\r");
//    PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
//    //int a = sscanf(PT_term_buffer, "%s", buffer);
//    PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
//
//    original = PT_term_buffer;
//    delim = strtok(PT_term_buffer, ", ");
//
//    for (q = 0; q < 6; q++)
//    {
//        strcpy(delimited[q], delim);
//        if (q != 5)
//            delim = strtok(NULL, ",");
//    }
//
//    // print to TFT 
//    tft_setCursor(30, 150);
//    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
//                //sscanf(PT_term_buffer, "%s", buffer);
//    tft_fillRoundRect(30, 150, 320, 90, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//    tft_writeString(original);
//    for (q = 0; q < 6; q++)
//    {
//        //sprintf(buffer, "%c", a);													  //tft_fillRoundRect(30,150, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(30, 160 + q*10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
//        //sscanf(PT_term_buffer, "%s", buffer);
//        tft_writeString(delimited[q]);
//    }
//    // end print to TFT
//}

//TODO: Determine exact thresholds for fruits
//Should we still consider 2d array?
static PT_THREAD (protothread_param(struct pt *pt))
{
    PT_BEGIN(pt);
    static int pattern;
    mPORTBSetPinsDigitalOut(BIT_3 | BIT_4 | BIT_7);    //Set port as output
    mPORTASetPinsDigitalIn(BIT_5 | BIT_8);    //Set port as input
    EnablePullDownB(BIT_5);
    
    static int q;
    static int state;
    char *delim;
    
    int cursor_x = 0;
    int cursor_y = 0;
    int incr = 4;
    
    tft_fillRoundRect(30, 150, 80, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
    while (strcmp(fruit, banana) && strcmp(fruit, apple) && strcmp(fruit, avocado))
    {      
        tft_setTextColor(ILI9340_RED); tft_setTextSize(1.5);
        tft_fillRoundRect(30, (160 + (setting * 10)), 80, 10, 1, ILI9340_WHITE);// x,y,w,h,radius,color
        tft_setCursor(30, 150);
        sprintf(buffer, "Which fruit to detect ripeness?");
        tft_writeString(buffer);
        tft_setCursor(30, 160);
        sprintf(buffer, "Banana");
        tft_writeString(buffer);
        tft_setCursor(30, 170);
        sprintf(buffer, "Apple");
        tft_writeString(buffer);
        tft_setCursor(30, 180);
        sprintf(buffer, "Avocado");
        tft_writeString(buffer);
    
        // read the ADC AN11 
        adc_9 = ReadADC10(0);  
        AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below
                    
        if (adc_9 > prevadc_9 + 80)
        {
            setting++;
            prevadc_9 = adc_9;
            tft_fillRoundRect(30, 150, 80, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        }
            
        else if (adc_9 < prevadc_9 - 80)
        {
            setting--;
            prevadc_9 = adc_9;
            tft_fillRoundRect(30, 150, 80, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        }
            
        if (setting == 3)
            setting = 0;
        if (setting == -1)
            setting = 2;
       
        if (mPORTBReadBits(BIT_5))
        {
            if (setting == 0)
                strcpy(fruit, banana);
            if (setting == 1)
                strcpy(fruit,apple);
            if (setting == 2)
                strcpy(fruit,avocado);
        }
        
        PT_YIELD_TIME_msec(20);
    }
        
    //Turn on Spectrometer LED
    sprintf(PT_send_buffer,"ATLED1=100\r");
    PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
    PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));

    //Clear Corresponding Section of the TFT Display
    tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
    tft_setCursor(30, 160);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
    tft_writeString(PT_term_buffer);
                
    //RB3 - Left/Right or Up/Down 
    //RB4 - Direction
    //RB7 - Ready Bit
    //RB8 - Acknowledgment Bit
	while (1) {
		//PT_YIELD_TIME_msec(500);
        if (state != 0)
            PT_YIELD_UNTIL(pt, mPORTBReadBits(BIT_8) == 256);
        mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
        //do_scan();
        
        //Take Readings from the Spectrometer
        sprintf(PT_send_buffer,"ATCDATA\r");
        PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
        PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));

        delim = strtok(PT_term_buffer, ", ");

        for (q = 0; q < 6; q++)
        {
            strcpy(delimited[q], delim);
            if (q != 5)
                delim = strtok(NULL, ",");
        }
        
        //Color Reading Calculation for Color Map of Spectrometer Readings on
        //The TFT Display
        
        //From the 4760 webpage - The Adafruit TFT color format is 16 bit:
        //Top 5 bits encoding red intensity, mask is 0xf800
        //Middle 6 bits green, mask is 0x07e0
        //Low 5 bits blue, mask is 0x001f
        
        //From Spectrometer Datasheet - 4-byte floating point values
        //No way we will get numbers near the 32-bit max value
        //We will try 2048 as a max for now
        
        //Color map will be just black to red for now
        //0-2048 value range to 0-31 
        
        color_reading = (unsigned short)atoi(delimited[1]);
        //multiply by 32
        color_reading = color_reading << 5;
        //divide by 2048
        color_reading = color_reading >> 11;
        //Left shift 11 bits for red intensity
        color_reading = color_reading << 11;
        
        //Clear Corresponding Section of the TFT Display
        tft_fillRoundRect(30, 150, 320, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(30, 150);
        sprintf(buffer, "State: %d", state);
        tft_writeString(buffer);
        tft_setCursor(30, 160);
        sprintf(buffer, "610nm Value: %s", delimited[0]);
        tft_writeString(buffer);
        tft_setCursor(30, 170);
        sprintf(buffer, "680nm Value: %s", delimited[1]);
        tft_writeString(buffer);
        tft_setCursor(30, 180);
        sprintf(buffer, "Steps_left:%d, Steps_right:%d, Steps_down:%d", steps_left, steps_right, total_steps_down);
        tft_writeString(buffer);
        
        //Fruit must be 3 cm. by 3 cm. in order to be scanned properly
        switch (state) {
            case 0:  // init / reset
                
                if (atoi(delimited[0]) < MAX_BLUE && atoi(delimited[0]) > MIN_BLUE){
                    state = 2; //go blue
                }
                
                else if (atoi(delimited[0]) < MAX_PINK && atoi(delimited[0]) > MIN_PINK){
                    state = 3;
                }
                
                else state = 1;

                break;

            case 1: // neither
                
                // move left set whatever bits
                if (atoi(delimited[0]) < MAX_BLUE && atoi(delimited[0]) > MIN_BLUE){
                    state = 2; //go blue
                }

                else if (atoi(delimited[0]) < MAX_PINK && atoi(delimited[0]) > MIN_PINK){
                    state = 3; // go pink
                }
                            
                else{
                    state = 1;
                    
                    //MOVE
                    mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
                    mPORTBSetBits(BIT_3);
                    mPORTBSetBits(BIT_7);
                }
                
                break;
                
            case 2: // blue             
                
                // move up set whatever bits
                if (atoi(delimited[0]) < MAX_PINK && atoi(delimited[0]) > MIN_PINK){
                    state = 3;
                }
                
                else 
                {
                    state = 2;
                    
                    //MOVE
                    mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
                    mPORTBSetBits(BIT_7);
                }

                break;
                
            case 3: // pink ; begin moving right out of pink           
                
                // move right set whatever bits
                if (atoi(delimited[0]) < MAX_PINK && atoi(delimited[0]) > MIN_PINK){
                    state = 3; // go pink
                    
                    //MOVE
                    mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
                    mPORTBSetBits(BIT_3 | BIT_4);
                    mPORTBSetBits(BIT_7);
                }
                
                else
                    state = 4;
                
                break;
                
            case 4: // continue moving right; start drawing pixels
                
                //MOVE
                mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
                mPORTBSetBits(BIT_3 | BIT_4);
                mPORTBSetBits(BIT_7);                 
                
                tft_fillRoundRect(cursor_x, cursor_y, 4, 4, 1, color_reading);
                cursor_x+=incr;
                
                steps_right++; 
                
                //Save 680nm value in ring buffer
                ripe_buffer[(sensor_pos + 1)%8] = atoi(delimited[1]);
                sensor_pos = (sensor_pos + 1)%8;
                
                //Detect Ripeness
                for (q = 0; q < 8; q++)
                {
                    if (fruit == banana)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }
                    }
                    if (fruit == apple)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }
                    }
                    if (fruit == avocado)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }
                    }
                }
                
                //Take average of fruit detections
                running_average = running_average / 8.0;
                        
                //fruit is ripe, end scanning
                if (fruit == banana)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }
                if (fruit == apple)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }          
                if (fruit == avocado)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }
                
                fruit_presence_counter = 0;
                
                if (ripe_detections == 3)
                {
                    fruit_ripe = 1;
                    //state = 7;
                }
                                   
                if ( steps_right == max_right ){
                    state = 6;
                }
                
                else 
                    state = 4;
                
                running_average = 0;
                
                break;
                
            case 5: // continue moving left; start drawing pixels
                
                //MOVE
                mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
                mPORTBSetBits(BIT_3);
                mPORTBSetBits(BIT_7);
                    
                tft_fillRoundRect(cursor_x, cursor_y, 4, 4, 1, color_reading);
                cursor_x-=incr;
                
                steps_left++;
                
                //Save 680nm value in ring buffer
                ripe_buffer[(sensor_pos + 1)%8] = atoi(delimited[1]);
                sensor_pos = (sensor_pos + 1)%8;
                
                //Detect Ripeness
                for (q = 0; q < 8; q++)
                {
                    if (fruit == banana)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }
                    }
                    if (fruit == apple)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }
                    }
                    if (fruit == avocado)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }                   
                    }
                }
                
                //Take average of fruit detections
                running_average = running_average / 8.0;
                        
                //fruit is ripe, end scanning
                //fruit is ripe, end scanning
                if (fruit == banana)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }
                if (fruit == apple)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }          
                if (fruit == avocado)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }
                
                fruit_presence_counter = 0;
                
                if (ripe_detections == 3)
                {
                    fruit_ripe = 1;
                    //state = 7;
                }
                
                if ( steps_left >= max_left ){
                    state = 6;
                }
                
                else 
                    state = 5;
                
                running_average = 0;
                
                break;
                
            case 6: // move down
                
                //MOVE
                mPORTBClearBits(BIT_3 | BIT_4 | BIT_7);
                mPORTBSetBits(BIT_4);
                mPORTBSetBits(BIT_7);
                        
                total_steps_down++;
                
                //move down TFT screen
                tft_fillRoundRect(cursor_x, cursor_y, 4, 4, 1, color_reading);
                cursor_y+=incr;
                
                //Save 680nm value in ring buffer
                ripe_buffer[(sensor_pos + 1)%8] = atoi(delimited[1]);
                sensor_pos = (sensor_pos + 1)%8;
                
                //Detect Ripeness
                for (q = 0; q < 8; q++)
                {
                    if (fruit == banana)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }     
                    }
                    if (fruit == apple)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }
                    }
                    if (fruit == avocado)
                    {
                        //Detect if fruit is even there
                        if (ripe_buffer[q] > 100)
                        {
                            running_average = running_average + ripe_buffer[q];
                            fruit_presence_counter++;
                        }   
                    }
                }
                
                //Take average of fruit detections
                running_average = running_average / 8.0;
                        
                //fruit is ripe, end scanning
                //fruit is ripe, end scanning
                if (fruit == banana)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }
                if (fruit == apple)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }          
                if (fruit == avocado)
                {
                    if (running_average > 500 && fruit_presence_counter > 2)
                        ripe_detections++;
                }
                
                fruit_presence_counter = 0;
                
                if (ripe_detections == 3)
                {
                    fruit_ripe = 1;
                    //state = 7;
                }
                
                //Clear the ring buffer
                for (q = 0; q < 8; q++)
                {
                    ripe_buffer[q] = 0;
                }
                
                if (steps_right >= max_right){
                    steps_right = 0;
                    state = 5;
                }
                
                else if (steps_left >= max_left){
                    steps_left = 0;
                    state = 4;
                }
                
                if (total_steps_down >= max_down){
                    state = 7;
                }        
                
                running_average = 0;
                
                break;
            
            //end case
            case 7:
                 //Turn off Spectrometer LED
                sprintf(PT_send_buffer,"ATLED1=0\r");
                PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
                PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));
    
                tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                while (!reset)
                {
                    //Clear Corresponding Section of the TFT Display
                    tft_setCursor(30, 150);
                    sprintf(buffer, "Is the fruit ripe yet?: %d", fruit_ripe);
                    tft_writeString(buffer);
                    tft_setCursor(30, 160);
                    sprintf(buffer, "Push Button to Scan Again");
                    tft_writeString(buffer);
                    PT_YIELD_TIME_msec(20);
                    if (mPORTBReadBits(BIT_5))
                        reset = 1;
                }
                
                cursor_x = 0;
                cursor_y = 0;
                adc_9 = 0;
                prevadc_9 = 0;
                setting = 0;
                sprintf(fruit, "");

                tft_fillRoundRect(30, 150, 320, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                while (!strcmp(fruit, banana) && !strcmp(fruit, apple) && !strcmp(fruit, avocado))
                {  
                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
                    tft_fillRoundRect(30, (160 + (setting * 10)), 320, 10, 1, ILI9340_WHITE);// x,y,w,h,radius,color
                    tft_setCursor(30, 150);
                    sprintf(buffer, "Which fruit to detect ripeness?");
                    tft_writeString(buffer);
                    tft_setCursor(30, 160);
                    sprintf(buffer, "Banana");
                    tft_writeString(buffer);
                    tft_setCursor(30, 170);
                    sprintf(buffer, "Apple");
                    tft_writeString(buffer);
                    tft_setCursor(30, 180);
                    sprintf(buffer, "Avocado");
                    tft_writeString(buffer);

                    // read the ADC AN11 
                    adc_9 = ReadADC10(0);  
                    AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below

                    if (adc_9 > prevadc_9 + 80)
                    {
                        setting++;
                        prevadc_9 = adc_9;
                        tft_fillRoundRect(30, 150, 320, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                    }

                    else if (adc_9 < prevadc_9 - 80)
                    {
                        setting--;
                        prevadc_9 = adc_9;
                        tft_fillRoundRect(30, 150, 320, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                    }

                    if (setting == 3)
                        setting = 0;
                    if (setting == -1)
                        setting = 2;

                    if (mPORTBReadBits(BIT_5))
                    {
                        if (setting == 0)
                            strcpy(fruit, banana);
                        if (setting == 1)
                            strcpy(fruit, apple);
                        if (setting == 2)
                            strcpy(fruit, avocado);
                    }

                    PT_YIELD_TIME_msec(20);
                }

                //Turn on Spectrometer LED
                sprintf(PT_send_buffer,"ATLED1=100\r");
                PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
                PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input));

                //Clear Corresponding Section of the TFT Display
                tft_fillRoundRect(30, 150, 320, 42, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                tft_setCursor(30, 160);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1.5);
                tft_writeString(PT_term_buffer);
    
                state = 0;
                fruit_ripe = 0;
                sensor_pos = 0;
                running_average = 0;
                steps_left = 0;
                steps_right = 0;
                total_steps_down = 0;
                ripe_detections = 0;
                reset = 0;
                
                //Clear the ring buffer
                for (q = 0; q < 8; q++)
                {
                    ripe_buffer[q] = 0;
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
    state = 0;
    fruit_ripe = 0;
    sensor_pos = 0;
    running_average = 0;
    total_steps_down = 0;
    setting = 0;
    max_right = 20;
    max_left = 20;
    max_down = 20;  
    sprintf(fruit, "");
    sprintf(banana, "banana");
    sprintf(apple, "apple");
    sprintf(avocado, "avocado");
    adc_9 = 0;
    prevadc_9 = 0;
    ripe_detections = 0;
    reset = 0;
    steps_left = 0;
    steps_right = 0;
    fruit_presence_counter = 0;
    
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
  
// the ADC ///////////////////////////////////////
	// configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

					// define setup parameters for OpenADC10
					// Turn module on | ouput in integer | trigger mode auto | enable autosample
					// ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
					// ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
					// ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF //

                        // define setup parameters for OpenADC10
                        // ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
                        //
                        // Define setup parameters for OpenADC10
                        // use peripherial bus clock | set sample time | set ADC clock divider
                        // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
                        // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

                        // define setup parameters for OpenADC10
                        // set AN11 and  as analog inputs
    #define PARAM4	ENABLE_AN11_ANA // pin 24

                        // define setup parameters for OpenADC10
                        // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL

					// use ground as neg ref for A | use AN11 for input A     
					// configure to sample AN11 
	SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11); // configure to sample AN11 
	OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
				   ///////////////////////////////////////////////////////
    
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
  
  AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below
  
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