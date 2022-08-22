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
#include "LCD.h"


/*------------------------------------------------------------------------------
 * Constantes
------------------------------------------------------------------------------*/
#define _XTAL_FREQ 8000000  //Oscilador de 8 MHz
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 524288            // Valor maximo de entrada del potenciometro
#define OUT_MIN 0              // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 65
#define BMP280_I2C_ADDRESS  0xEE
uint16_t dig_T1;
uint16_t dig_T2;
uint16_t dig_T3;
int cont1 = 0;
int valor = 0;
signed long temp;
uint8_t tempA;
uint8_t tempB;
uint8_t tempC;
uint8_t val_canal0 = 0;         // Variable para almacenar el valor del canal 0
uint8_t config;
uint8_t ctrl_meas;
signed long temperature;

uint8_t voltage1 [4] = {0,0,0,0};    // Variables para almacenar los dígitos de los valores de voltaje - centenas, decenas, unidades, residuo
uint16_t calor [5] = {0,0,0,0,0};
/*------------------------------------------------------------------------------
 * Prototipos de funciones
------------------------------------------------------------------------------*/
void setup (void);                  // Prototipo de función SETUP
void valor_voltaje(int valor_adc);          // Prototipo de función de conversión a voltaje
long int temperatura(uint8_t a, uint8_t b, uint8_t c);//, uint8_t c);
void valor_temperatura (signed long sensor);
unsigned long map(unsigned long x, unsigned long x0, unsigned long x1,
            unsigned short y0, unsigned short y1);
/*
 * 
 */
void __interrupt() isr(void){ // FUNCION DE INTERRUPCIONES

   
 return;   
}
void main(void) {
    setup();
                           // Configuraciones de los módulos
    
    Lcd_Set_Cursor(1,1);                  // Iniciamos la escritura en la primera posición
    Lcd_Write_String("  S1     TEMP");   // Escribimos los datos deseados en el display
    
    while(1){
        config = ((0x00 << 5) | (0x00 << 2)) & 0xFC;
        ctrl_meas = (0x01 << 5) | (0x01 << 2) | 0x03;
         //_config = ((standby << 5) | (filter << 2)) & 0xFC;
         //_ctrl_meas = (T_sampling << 5) | (P_sampling << 2) | mode;
  
        I2C_Master_Start();
        I2C_Master_Write(0x50);
        I2C_Master_Write(valor);
        I2C_Master_Stop();
        __delay_ms(1);
        
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        cont1 = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(1);
        
      I2C_Master_Start();
      I2C_Master_Write(0xEE);
      I2C_Master_Write(0xF5);
      I2C_Master_Write(config);
      I2C_Master_Stop();
        __delay_ms(1);
        
      I2C_Master_Start();
      I2C_Master_Write(0xEE);
      I2C_Master_Write(0xF4);
      I2C_Master_Write(ctrl_meas);
      I2C_Master_Stop();
        __delay_ms(250);
        
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE);
        I2C_Master_Write(0xFA);
        
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0xEF);
        tempA = I2C_Master_Read(0);
        tempB = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(1);
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE);
        I2C_Master_Write(0xFC);
        
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0xEF);
        tempC = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(1);
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE);
        I2C_Master_Write(0x88);
        
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0xEF);
        dig_T1 = ((I2C_Master_Read(0))<<8);
        dig_T1 += I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(1);
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE);
        I2C_Master_Write(0x8A);
        
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0xEF);
        dig_T2 = ((I2C_Master_Read(0))<<8);
        dig_T2 += I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(250);
        
        I2C_Master_Start();
        I2C_Master_Write(0xEE);
        I2C_Master_Write(0x8C);
        
        I2C_Master_RepeatedStart();
        I2C_Master_Write(0xEF);
        dig_T3 = ((I2C_Master_Read(0))<<8);
        dig_T3 += I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(1);
        

        temp = (signed long) temperatura(tempA,tempB,tempC);
        
        valor_temperatura (temp);
        
        val_canal0 =  (int) (cont1*((float )5/255)*(100));
        
        if (!RB0)
            {valor = 1;  
            }
        else if (!RB1)
            {valor = 2; 
            }
        else if (!RB2)
            {valor = 4; 
            }
        else if (!RB3)
            {valor = 8; 
            }
        else
            {valor = 0;
            }
        
       valor_voltaje (val_canal0);
        
       Lcd_Set_Cursor(2,1);
        
        
        Lcd_Write_String(" ");
        Lcd_Write_Char(voltage1 [0] + 48); //ubicar ek valor del caracter en el ASCII
        Lcd_Write_Char(46); //caracter que representa al punto
        Lcd_Write_Char(voltage1 [1] + 48);
        Lcd_Write_Char(voltage1 [2] + 48);
        Lcd_Write_Char(86);//Caracter que representa la V en el ASCII
        
        Lcd_Write_String(" ");
        Lcd_Write_String(" ");
        
        Lcd_Write_Char(calor [0] + 48); //ubicar ek valor del caracter en el ASCII
        Lcd_Write_Char(calor[1] + 48);
        Lcd_Write_Char(46); //caracter que representa al punto
        Lcd_Write_Char(calor[2] + 48);
        Lcd_Write_Char(calor[3] + 48);
        Lcd_Write_Char(67);//Caracter que representa la V en el ASCII
        
                
        
    }
    return;
}

void setup(void) {
    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    TRISC = 0b00011000;
    PORTC = 0;
    
    init_osc_MHz (8);
    
    Lcd_Init();
    
    I2C_Master_Init(10000);
    
    TRISB = 0x0F; 
    PORTB = 0x00;
    TRISA = 0;
    PORTA = 0;
    
         
    
    //Config. PULL-UP
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;   
    WPUBbits.WPUB2 = 1;
    WPUBbits.WPUB3 = 1;  
    
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    return;
}

void valor_voltaje (int valor_adc) // funcion que encuentra el valor del voltaje
{  
        voltage1 [0] =  (uint8_t)(valor_adc/100);
        voltage1 [3] =   valor_adc%100;

        voltage1 [1] =   voltage1 [3]/10;
        voltage1 [3] =   voltage1 [3]%10;

        voltage1 [2] =  voltage1 [3];
       
    return;
}

long int temperatura(uint8_t a, uint8_t b, uint8_t c) // funcion que encuentra el valor del voltaje
 {   signed long medida;
     uint8_t variable;

        medida = a;
        medida = medida<<8;
        medida = medida|b;
        medida = medida<<3;
        variable = c>>4;
        medida = medida | variable;
       
    return medida;
}

void valor_temperatura (signed long sensor) // funcion que encuentra el valor del voltaje
{  double var1;
   double var2;
   double t_fine;
   uint16_t T;
   
        var1 = (((double)sensor)/16384-((double)dig_T1)/1024)*((double)dig_T2);
        var2 = ((((double)sensor)/131072-((double)dig_T1)/8192)*(((double)sensor)/131072-((double)dig_T1)/8192))*((double)dig_T3);
        t_fine = (signed long)(var1 + var2);
        T = (uint16_t)(t_fine/5120);
   
        calor [0] =  (uint16_t)(T/1000);
        calor [4] =   T%1000;

        calor [1] =   calor [4]/100;
        calor [4] =   calor [4]%100;
        
        calor [2] =   calor [4]/10;
        calor [4] =   calor [4]%10;

        calor [3] =  calor [4];
    return;
}

unsigned long map(unsigned long x, unsigned long x0,unsigned long x1,
            unsigned short y0, unsigned short y1){
    return (unsigned long)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}