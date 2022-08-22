/* 
 * File:   ADC.h
 * Author: Compu Fire
 *
 * Created on 22 de julio de 2022, 7:21
 */

#ifndef ADC_H
#define	ADC_H
#ifndef _XTAL_FREQ 
#define _XTAL_FREQ 8000000
#endif

#include <stdint.h>
#include <xc.h>

//******************************************************
// Prototipo de la funcion para configurar e inicializar modulo ADC
// Parametros: Seleccion de reloj (FOSC/2, FOSC/8, FOSC/32 Frc), voltajes de ref externo o internos
// y seleccion de canales
// Esta libreria configura las interrupciones del ADC
// Revisar pagina 99 del manual del pic
//******************************************************
void adc_init (uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);
void adc_start (uint8_t channel);
uint16_t adc_read (void);


#endif	/* ADC_CONFIG_H */
