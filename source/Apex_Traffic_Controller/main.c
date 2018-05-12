// I2C0.c
// Runs on LM4F120/TM4C123
// Provide a function that initializes, sends, and receives the I2C0 module
// interfaced with an HMC6352 compass or TMP102 thermometer.
// Daniel Valvano
// July 2, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
Section 8.6.4 Programs 8.5, 8.6 and 8.7

Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
You may use, edit, run or distribute this file
as long as the above copyright notice remains
THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
For more information about my classes, my research, and my books, see
http://users.ece.utexas.edu/~valvano/
*/

// I2C0SCL connected to PB2 and to pin 4 of HMC6352 compass or pin 3 of TMP102 thermometer
// I2C0SDA connected to PB3 and to pin 3 of HMC6352 compass or pin 2 of TMP102 thermometer
// SCL and SDA lines pulled to +3.3 V with 10 k resistors (part of breakout module)
// ADD0 pin of TMP102 thermometer connected to GND
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "LM4F120.h"
#include "queue.h"
#include "main.h"
#include "SysTick_16.h"

//#include "sensor.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              2           // number of receive attempts before giving up

//clock gating for GPIO
#define SYSCTL_RCGC2          (*((volatile unsigned long *)0x400FE108))

volatile unsigned long Counts=0;

unsigned long jump_timer;
short int jump_flag;
short int jump_start;

int anode[3] = {0x02,0x01,0x10}; // LED_Display an arrow

int LED_1_x[3] = {0x10,0x28,0x28};
int LED_2_x[3] = {0x40,0xA0,0xA0};
int LED_3_x[3] = {0x02,0x09,0x09};
int LED_4_x[3] = {0x08,0x44,0x44};
//
int LED_1_g[3] = {0x38,0x10,0x10};
int LED_2_g[3] = {0xE0,0x40,0x40};
int LED_3_g[3] = {0x0B,0x02,0x02};
int LED_4_g[3] = {0x4C,0x08,0x08};

int LED_state[4]= {0,0,0,0};

int i = 0;
int j = 0;
uint8_t states[5];


/*Trampoline areas*/
int A1[2] = {54,55};
int A2[4] ={30,31,38,39};
int A3[16] = {3,4,11,12,19,20,27,28,35,36,43,44,51,52,59,60};
int A4[3]= {16,24,32};
int A5[3] = {5,6,7};

int average_1, average_2;
uint8_t rv;
uint8_t flag;


void UART_send(unsigned char);

void init_gpio (void){

    volatile unsigned long delay_clk;   // delay for clock, must have 3 sys clock delay, p.424
    SYSCTL_RCGC2 |= 0x1F;               // enable clock gating for port A, B,C,D, and E p.424
    delay_clk = SYSCTL_RCGC2;           // this is a dummy instruction to allow clock to settle, no-operation

    //initialize registers for PORTA
    GPIO_PORTA_DIR |= 0x38;     // bit 3,4,5,6,and 7 set to 1 (output)
    GPIO_PORTA_DEN |= 0x38;     // bit 3,4,5,6,and 7 set to 1(enabled)
    GPIO_PORTA_AFSEL &=~0x38;   //bit 3,4,5,6,and 7 set to 0 (disabled)

    //initialize registers for PORTB
    GPIO_PORTB_DIR |= 0xF3;     // bit 0,1,4,5,6,7  set to 1 (output)
    GPIO_PORTB_DEN |= 0xF3;     // bit 0,1,4,5,6,7set to 1 (enabled)
    GPIO_PORTB_AFSEL &=~0xF3;   //bit 0,1,4,5,6,7 set to 0 (disabled)

    //initialize registers for PORTC
    GPIO_PORTC_DIR |= 0xF0;     // bit 4,5,6,and 7  set to 1 (output)
    GPIO_PORTC_DEN |= 0xF0;     // bit 4,5,6,and 7 set to 1 (enabled)

    //initialize registers for PORTD
    GPIO_PORTD_DIR |= 0x4F;     // bit 0,1,2,3,and 6 set to 1 (output)
    GPIO_PORTD_DEN |= 0x4F;     // bit 0,1,2,3,and 6 set to 1 (enabled)
    GPIO_PORTD_AFSEL &=~0x4F;   //bit 0,1,2,3, and 6 set to 0 (disabled)

    //initialize registers for PORTE
    GPIO_PORTE_DIR |= 0x1F;     // bit 0,1,2,3,and 4  set to 1 (output)
    GPIO_PORTE_DEN |= 0x1F;     // bit 0,1,2,3, and 4 set to 1 (enabled)
    GPIO_PORTE_AFSEL &=~0x1F;   //bit 0,1,2,3, and 4 set to 0 (disabled)

}

void I2C0_Init(void){

    SYSCTL_RCGCI2C_R |= 0x0001;           // activate I2C0
    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only
    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
    // 6) configure PB2,3 as I2C
    GPIO_PORTB_PCTL_R |= (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3
    I2C0_MCR_R = I2C_MCR_MFE;             // 9) master function enable
    I2C0_MTPR_R = 0x7;                    // 8) configure for 100 kbps clock
    // 20*(TPR+1)*20ns = 10us, with TPR=24
}

uint8_t I2C0_send_Recv(uint8_t reg_addr){

    while(I2C0_MCS_R&I2C_MCS_BUSY){};   // wait for I2C ready
    I2C0_MSA_R = (0x68 <<1)&0xFE;       // MSA[7:1] is slave address
    I2C0_MSA_R &= ~0x01;                // MSA[0] is 0 for write
    I2C0_MDR_R = reg_addr;              // prepare first byte
    I2C0_MCS_R = (0
            //| I2C_MCS_STOP            // generate stop
            | I2C_MCS_START             // generate start/restart
            | I2C_MCS_RUN);             // master enable
    while(I2C0_MCS_R&I2C_MCS_BUSY){};   // wait for transmission done

    //check if slave address and register address sent correctly
    if((I2C0_MCS_R & (I2C_MCS_ADRACK | I2C_MCS_DATACK | I2C_MCS_ERROR)) == 0){

        while(I2C0_MCS_R&I2C_MCS_BUSY){};   // wait for I2C ready
        I2C0_MSA_R = (0x68<<1)&0xFE;        // MSA[7:1] is slave address
        I2C0_MSA_R |= 0x01;                 // MSA[0] is 1 for receive
        I2C0_MCS_R = (0
                // & ~I2C_MCS_ACK           // negative data ack (last byte)
                | I2C_MCS_STOP              // generate stop
                | I2C_MCS_START             // generate start/restart
                | I2C_MCS_RUN);             // master enable
        while(I2C0_MCS_R&I2C_MCS_BUSY){};   // wait for transmission done

    }

    return (I2C0_MDR_R&0xFF);

}

void I2C1_Init(void){

    SYSCTL_RCGCI2C_R |= 0x02;           // activate I2C1
    GPIO_PORTA_AFSEL |= 0xC0;           // 3) enable alt funct on PA6,and PA7
    GPIO_PORTA_ODR |= 0x80;             // 4) enable open drain on PA7 only
    GPIO_PORTA_DEN |= 0xC0;             // 5) enable digital I/O on PA6 and PA7
    // 6) configure PA6, PA7 as I2C
    GPIO_PORTA_PCTL |= (GPIO_PORTA_PCTL &0x00FFFFFF)+0x33000000;
    GPIO_PORTA_AMSEL &= ~0xC0;          // 7) disable analog functionality on PA6,and PA7
    I2C1_MCR_R = I2C_MCR_MFE;           // 9) master function enable
    I2C1_MTPR_R = 0x7;                  // 8) configure for 100 kbps clock
    // 20*(TPR+1)*20ns = 10us, with TPR=24

}

uint8_t I2C1_send_Recv(uint8_t reg_addr){

    while(I2C1_MCS_R&I2C_MCS_BUSY){};   // wait for I2C ready
    I2C1_MSA_R = (0x68 <<1)&0xFE;       // MSA[7:1] is slave address
    I2C1_MSA_R &= ~0x01;                // MSA[0] is 0 for write
    I2C1_MDR_R = reg_addr;              // prepare first byte
    I2C1_MCS_R = (0

            //| I2C_MCS_STOP            // generate stop
            | I2C_MCS_START             // generate start/restart
            | I2C_MCS_RUN);             // master enable
    while(I2C1_MCS_R&I2C_MCS_BUSY){};   // wait for transmission done

    //check if slave address and register address sent correctly
    if((I2C1_MCS_R & (I2C_MCS_ADRACK | I2C_MCS_DATACK|I2C_MCS_ERROR)) == 0){

        while(I2C1_MCS_R&I2C_MCS_BUSY){};   // wait for I2C ready
        I2C1_MSA_R = (0x68<<1)&0xFE;        // MSA[7:1] is slave address
        I2C1_MSA_R |= 0x01;                 // MSA[0] is 1 for receive
        I2C1_MCS_R = (0
                // & ~I2C_MCS_ACK           // negative data ack (last byte)
                | I2C_MCS_STOP              // generate stop
                | I2C_MCS_START             // generate start/restart
                | I2C_MCS_RUN);             // master enable
        while(I2C1_MCS_R&I2C_MCS_BUSY){};   // wait for transmission done

    }

    return (I2C1_MDR_R&0xFF);

}

void UART_init(void){

    SYSCTL_RCGCUART |= 0x01;            // p.421, activate UART0
    UART0_CTL &= ~0x01;                 // p.868, disable UART0 during config
    UART0_IBRD = 104;                   // baud rate
    UART0_FBRD = 11;
    UART0_LCRH |= 0x60;                 // p.866, word length 8 bit, all other default, 8N1
    UART0_CC = 0x0;
    UART0_CTL = (1<<0)|(1<<8)|(1<<9);   // Enable UART0 after config

    // Configure PA0 and PA1 as UART ports
    GPIO_PORTA_AFSEL |= 0x03;           // p.624, enable alternate func for PA0 and PA1
    GPIO_PORTA_DEN |= 0x03;             // p.636, enable DEN for PA0 and PA1
    GPIO_PORTA_PCTL  |= (GPIO_PORTA_PCTL&0xFFFFFF00)+0x00000011;    //is not needed since the default is '1' for PA0-1; see Table 10-11 on p.641

}

void LED_init(){

    LED_state[0] = 0;
    LED_state[1] = 0;
    LED_state[2] = 0;
    LED_state[3] = 0;

}

//delay function
void delay (){

    long unsigned i;
    for (i=0; i<2000000; i++);

}

// UART send character function
void UART_send(unsigned char uartSendData){

    while((UART0_FR & 0x20) != 0);  // wait if Transmit FIFO is full, TXFF
    UART0_DR = uartSendData;

}

uint8_t conver(uint8_t hex){

    uint8_t decimal_number = 0, remainder;
    uint8_t count = 0;

    while(hex > 0)
    {
        remainder = hex % 10;
        decimal_number = decimal_number + remainder * pow(16, count);
        hex = hex / 10;
        count++;
    }
    return decimal_number;

}

void printString(char *pt){

    while(*pt){
        UART_send(*pt);
        pt++;
    }

}

void tostring(char str[], int num){

    int i, rem, len = 0, n;

    n = num;

    while (n != 0)

    {
        len++;
        n /= 10;

    }

    for (i = 0; i < len; i++)

    {

        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';

    }

    str[len] = '\0';

}

void converter_new(uint8_t *ini_data_1, uint8_t *ini_data_2){

    int i;
    uint8_t temp;
    for (i=0;i<64;i++)
    {
        temp = ini_data_1[i];
        if (temp> (average_1 +3)){
            ini_data_1[i] = 1;
        }
        else{
            ini_data_1[i] = 0;
        }
    }

    for (i=0;i<64;i++)
    {
        temp = ini_data_2[i];
        if (temp> (average_2 +3)){
            ini_data_2[i] = 1;
        }
        else{
            ini_data_2[i] = 0;
        }
    }

}

void slicer(uint8_t *new_data_1){
    // assign trampoline 1
    int a;
    int i = 0;

    // tramp_1
    int temp_1 = 0;
    for (a = 0;a<(sizeof(A1)/sizeof(int));a++){
        i = A1[a];
        temp_1 = temp_1 + new_data_1[i];
    }

    if(temp_1 == 0){
        states[0] = 0;
    }
    else{
        states[0] = 1;
    }
    //tramp_2
    int temp_2 = 0;
    for (a=0;a<(sizeof(A2)/sizeof(int));a++){
        i = A2[a];
        temp_2 = temp_2 + new_data_1[i];
    }

    if(temp_2 == 0){
        states[1] = 0;
    }
    else{
        states[1] =1;
    }
    // airbag
    int temp_3 = 0;
    for (a=0;a<(sizeof(A3)/sizeof(int));a++){
        i = A3[a];
        temp_3 = temp_3 + new_data_1[i];
    }

    if(temp_3 == 0){
        states[2] = 0;
    }
    else{
        states[2] = 1;
    }
    //big platform
    int temp_4 = 0;

    for (a=0;a<(sizeof(A5)/sizeof(int));a++){
        i = A5[a];
        temp_4 = temp_4 + new_data_1[i];
    }
    if(temp_4 == 0){
        states[3] = 0;
    }
    else{
        states[3] = 1;
    }
    //smal platform
    int temp_5 = 0;
    for (a=0;a<(sizeof(A4)/sizeof(int));a++){
        i = A4[a];
        temp_5 = temp_5 + new_data_1[i];
    }

    if(temp_5 == 0){
        states[4] = 0;
    }
    else{
        states[4] = 1;
    }
    i = 0;
}

void scheduler(){

    int counter = 0;

    //check if station is next in queue and if there is someone at the station

    /*if next in queue and ready to jump and nobody is in the airbag
     *
     * rv = Next in Queue
     * State[0] = next in queue platform available jumper?
     * state[2] = person in airbag
     * */

    /*check if there is activity on more than one station
     *This signifies that there are jumpers ready on more than
     *one platform
     */
    while((states[0] + states[1] + states[3] + states[4] > 1) && (flag == 1) && (counter < 5)){

        /*allow jumper on current station 5 secs
         *and then remove from the queue*/
        /*
        if (jump_timer > 500000){
            Data_Display();
            counter++;
            jump_flag = 1;
        }
        */

        SysTick_Wait1ms(1000);
        Data_Display();
        counter++;

    }

    if(counter > 5){

        LED_state[rv] = 0;
        counter = 0;
        push(rv);
        rv = pop();
        //jump_flag = 1;

    }

    if((rv == 0) && (states[0] == 1) && (states[2] == 0)){
        LED_state[0] = 1;
        flag = 1;
        jump_flag = 1;

    }
    else if((rv == 1) && (states[1] == 1) && (states[2] == 0)){
        LED_state[1] = 1;
        flag = 1;
        jump_flag = 1;

    }
    else if((rv == 2) && (states[3] == 1) && (states[2] == 0)){
        LED_state[2] = 1;
        flag = 1;
        jump_flag = 1;

    }
    else if((rv == 3) && (states[4] == 1) && (states[2] == 0)){
        LED_state[3] = 1;
        flag = 1;
        jump_flag = 1;

    }
    else{

        flag = 0;
    }

    /*No activity on platform. push back and pop new*/
    if(flag == 0){

        LED_state[rv] = 0;
        push(rv);
        rv = pop();
    }

}

void turn_off(){

    GPIO_PORTB_DATA = 0xE0;
    GPIO_PORTA_DATA = 0x38;
    GPIO_PORTE_DATA = 0x0B;
    GPIO_PORTD_DATA = 0x4C;

}

void LED_Display(){

    if((LED_state[0] == 0) && (j == 0)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTA_DATA = ~LED_1_x[i];

    }

    else if ((LED_state[0] == 1) && (j == 0)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTA_DATA = ~LED_1_g[i];

    }

    else if((LED_state[1] == 0) && (j == 1)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTB_DATA &= ~LED_2_x[i];

    }

    else if((LED_state[1] == 1) && (j == 1)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTB_DATA &= ~LED_2_g[i];

    }
    else if((LED_state[2] == 0) && (j == 2)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTE_DATA &= ~LED_3_x[i];


    }

    else if((LED_state[2] == 1) && (j == 2)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTE_DATA &= ~LED_3_g[i];

    }

    else if((LED_state[3] == 0) && (j == 3)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTD_DATA &= ~LED_4_x[i];


    }

    else if((LED_state[3] == 1) && (j == 3)){

        turn_off();
        GPIO_PORTB_DATA |= anode[i];
        GPIO_PORTD_DATA &= ~LED_4_g[i];

    }

    i++;
    if(i > 2){
        i = 0;
        j++;
    }
    if(j > 3){
        j = 0;

    }


}

void Data_Display(){

    int i,k,h;
    uint8_t data_0, temp_0;
    uint8_t data_1, temp_1;
    char data_arr_1[16][4];
    uint8_t f1[64] ;
    uint8_t f2[64];
    uint8_t temp[16];

        k = 0;
        h = 0;
        average_1 = 0;
        average_2 = 0;
        for(i =0x80; i<256; (i = i+2)){

            temp_0 = I2C0_send_Recv(i);
            data_0 = (temp_0 /4);
            average_1 = data_0 + average_1;
            f1[h] = data_0;

            temp_1 = I2C1_send_Recv(i);
            data_1  = (temp_1 /4);
            average_2 = data_1 + average_2;
            f2[h] = data_1;

            h++;
        }

        average_1 = average_1/64;
        average_2 = average_2/64;
        h = 0;

        for(i =0; i < 64; i++){

            data_0 = f1[i];
            data_1 = f2[i];

            tostring(data_arr_1[h], data_0);
            tostring(data_arr_1[h+1], data_1);
            temp[h]   = data_0;
            temp[h+1] = data_1;

            h= h+2;

            if(i == 0){

                printString("Sensor1 \t");
                printString("\t");
                printString("\t");
                printString("Sensor2");
                UART_send('\n');
                UART_send('\r');
            }
            else if(((i+1) % 8) == 0){

                /*Sensor 1 readings*/
                UART_send ('[');
                for(k = 0; k <16; k = k+2){

                    if(temp[k] > (average_1 + 3)){
                        printString("\x1b[32;1m");
                        printString(data_arr_1[k]);
                        printString("\x1b[37m");
                    }
                    else{

                        printString(data_arr_1[k]);

                    }
                    UART_send (',');
                }

                printString ("] \t");
                UART_send ('[');

                /*Sensor 2 readings*/
                for(k = 1; k < 16; k = k+2){

                    if(temp[k] > (average_2 + 3)){
                        printString("\x1b[32;1m");
                        printString(data_arr_1[k]);
                        printString("\x1b[37m");
                    }
                    else{

                        printString(data_arr_1[k]);
                    }
                    UART_send (',');
                }
                UART_send (']');
                UART_send('\n');
                UART_send('\r');
                h = 0;

            }

        }

        tostring(data_arr_1[0], average_1);
        printString("Average: ");
        printString(data_arr_1[0]);

        printString("\t");
        printString("\t");
        printString("\t");

        tostring(data_arr_1[0], average_2);
        printString("Average: ");
        printString(data_arr_1[0]);

        UART_send('\n');
        UART_send('\r');
        h = 0;
        converter_new(f1,f2);
        slicer(f2);

}

int main(void){

    init_gpio(); // initialize GPIO functions
    SysTick_Init();
    I2C0_Init();
    I2C1_Init();
    UART_init();
    LED_init();

    /* 0 - Tramp 1
     * 1 - Tramp 2
     * 2 - Big Platform
     * 3 - Small Platform
     */
    for(i = 0; i <4; i++){
        push(i);
    }

    /*Initial pop*/
    rv = pop();


    while (1){

        if (jump_flag == 1){
            jump_start = NVIC_ST_CURRENT_R;
            jump_flag == 0;
        }

        jump_timer = ((jump_start - NVIC_ST_CURRENT_R) & 0x00FFFFFF);

        //wait for interrupt
        Data_Display();
        scheduler();
    }

}
