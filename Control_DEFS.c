/*
 * Control_DEFS.c
 *
 *  Created on: 20 de abr de 2023
 *      Author: Auro Gabriel
 */

#include "Controle_DEFS.h"

#include "F28x_Project.h"
#include "F2837xD_i2c.h"
void i2c_DspConfig(void){

    //##########__CONFIGURACAO Prescale I2C__#######################################################################

        I2caRegs.I2CMDR.bit.IRS=0;        //Deixo o m�dulo desligado pra poder mudar os valores
        I2caRegs.I2CPSC.bit.IPSC=128   ; // 200M/128= 1 562 500 Hz

        I2caRegs.I2CCLKH=16;       // divido pra dar a frequ�ncia de 98.16KHz -> No datasheet fala que o padrao e 100KHz, entao fe
        I2caRegs.I2CCLKL=16;
        I2caRegs.I2CMDR.bit.IRS=1; //Ligo o modulo que agora aceita os valores de prescale

        //##########__CONFIGURACAO Modo do I2C__#######################################################################

        I2caRegs.I2CMDR.bit.NACKMOD=0;// This bit is only applicable when the I2C module is acting as a receiver

        I2caRegs.I2CMDR.bit.FREE=1;   // Free= 1  The I2C module runs free that is, it continues to operate when a breakpoint occurs.

        //START MASTER MODE
        I2caRegs.I2CMDR.bit.STT=0;     // STT=0    In the master mode, STT is automatically cleared after the START condition has been generated.
        //STOP
        I2caRegs.I2CMDR.bit.STP=0;     // Note that the STT and STP bits can be used to terminate the repeat mode, and that this bit is not writable when IRS=0

        //NECESSARIO SETAR DEPOIS DO STOP:
        I2caRegs.I2CMDR.bit.MST=1;        //MST is automatically changed from 1 to 0 when the I2C master generates a STOP condition
                                          // 1h (R/W) = Master mode. The I2C module is a master and generates the serial clock on the SCL pin


        I2caRegs.I2CMDR.bit.TRX=1;       // TRX=1  Transmitter mode. When relevant, TRX selects whether the I2C module is in the transmitter mode or the receiver mode.

        I2caRegs.I2CMDR.bit.XA=0;          // Expanded address enable bit.
                                        // 0h (R/W) = 7-bit addressing mode (normal address mode). The I2C  module transmits 7-bit slave addresses (from bits 6-0 of I2CSAR),
                                        // and its own slave address has 7 bits (bits 6-0 of I2COAR).

        I2caRegs.I2CMDR.bit.RM=1;      // Repeat mode bit (only applicable when the I2C module is a mastertransmitter)

        I2caRegs.I2CMDR.bit.DLB=0;     // 0H= DISABLE - Digital loopback mode bit.

        // START THE MODULE
        I2caRegs.I2CMDR.bit.STB=1;     //1h (R/W) = The I2C module is in the START byte mode. When you set the START condition bit (STT),
                                        //the I2C module begins the transfer with more than just a START condition.
       I2caRegs.I2CMDR.bit.FDF=0;      // 0h (R/W) = Free data format mode is disabled. Transfers use the
                                       //7-/10-bit addressing format selected by the XA bit.

        I2caRegs.I2CMDR.bit.BC=0;   //0h (R/W) = 8 bits per data byte. Bit count bits.

}


void i2c_MPUselftest(void){





}

