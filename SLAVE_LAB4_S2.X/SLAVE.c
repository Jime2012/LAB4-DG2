/* 
 * File:   Main.c
 * Author: Compu Fire
 *
 * Created on 29 de julio de 2022, 8:51
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


/*------------------------------------------------------------------------------
 * Includes & Librerías
------------------------------------------------------------------------------*/

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <stdint.h>

#include "I2C.h"
#include "Oscilador.h"
#include "ADC.h"

/*------------------------------------------------------------------------------
 * Constantes
------------------------------------------------------------------------------*/
#define _XTAL_FREQ 8000000  //Oscilador de 8 MHz
uint8_t z; 
uint8_t valor; 
/*------------------------------------------------------------------------------
 * Prototipos de funciones
------------------------------------------------------------------------------*/
void __interrupt() isr(void){
   if (PIR1bits.ADIF)  // Interrupción del ADC
    {
        if (ADCON0bits.CHS == 0b0000)       //Si fue conversión del canal 0
        {    
           
            valor = ADRESH;
        }
 
    PIR1bits.ADIF = 0;                      //Reiniciar ADC
    }
   
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL))
        {
            z = SSPBUF;
            SSPCONbits.SSPOV = 0;
            SSPCONbits.WCOL = 0;
            SSPCONbits.CKP = 1;
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW)
        {
            //__delay_us(7);
            z = SSPBUF;
            //__delay_us(2);
            PIR1bits.SSPIF = 0;
            SSPCONbits.CKP = 1;
            while(!SSPSTATbits.BF);
            PORTD = SSPBUF;
            __delay_us(250);
            
        }
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW)
        {
            z = SSPBUF;
            BF = 0;
            SSPBUF = valor;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
}



void setup (void);                  // Prototipo de función SETUP

void main(void) {
    setup();
    while(1){
    adc_start(0);                           // Realizar conversión ADC en cada ciclo
     __delay_ms(20);
     
    }
    
    

    return;
}

void setup(void) {
    TRISA = 0b00000001;         // PORTA como salida, RA0 & RA1 como entradas
    PORTA = 0;                  // Limpiamos PORTA 
    
    ANSEL = 0b00000001;          //AN0 & AN1 como entradas analógicas
    ANSELH = 0;    
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    I2C_Slave_Init(0x50);
    init_osc_MHz (8);
    //Configuración del ADC 
    adc_init (8,0,0);
    
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    return;
}

