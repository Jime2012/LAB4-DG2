/* 
 * File:   Oscilador.h
 * Author: Compu Fire
 *
 * Created on 29 de julio de 2022, 9:02
 */

#ifndef OSCILADOR_H
#define	OSCILADOR_H

#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdint.h>

void init_osc_MHz (uint8_t freq);  // Prototipos de funciones

#endif

