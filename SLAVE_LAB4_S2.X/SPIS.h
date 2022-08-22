/* 
 * File:   SPIS.h
 * Author: Compu Fire
 *
 * Created on 3 de agosto de 2022, 20:42
 */

#ifndef SPIS_H
#define	SPIS_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdint.h>

void init_SPI (void);  // Prototipos de funciones
void spiWrite(char dat);
void spiReceiveWait(void);
unsigned spiDataReady(void);
char spiRead(void);
#endif
