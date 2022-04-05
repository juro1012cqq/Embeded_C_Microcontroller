//  Microcontroller 16F887 lib.
//  Author: juro1012cqq
//  Date:   4/3/22
//  License: FREE.
//  Enviroment: OS supported by CCS
//
#ifndef __My16f887_h__
#define __My16f887_h__
#endif

int tmp, tmp1;

//////////////////////////////////////////////////////////////////////////////////////////

/*--------------------------------------------------------------------------------------*/
// DATATYPE                                                                             //
/*--------------------------------------------------------------------------------------*/
#if (!defined(__DATATYPEC__))
#define __uint8__ unsigned int8
#define __int8__  int8
#define __int16__ int16
#define __uint16__ unsigned int16 
#define __int32__ int32
#define __uint32__ unsigned int32
#define __int64__ int64
#define __uint64__ unsigned int64
#endif

//

//////////////////////////////////////////////////////////////////////////////////////////

/*--------------------------------------------------------------------------------------*/
// MEMORY OGANIZATION                                                                   //
/*--------------------------------------------------------------------------------------*/
#ifndef __TQFP44_memory_oganization__
#define __TQFP44_memory_oganization__
// Memory's bank define:
#define BANK0           0
#define BANK1           1
#define BANK2           2
#define BANK3           3
// Define offsets inside bank 0:
#define TMR0_ADDR       0x1
#define PCL0_ADDR       0x2
#define STATUS0_ADDR    0x3
#define FSR0_ADDR       0x4
#define PORTA_ADDR      0x5
#define PORTB_ADDR      0x6
#define PORTC_ADDR      0x7
#define PORTD_ADDR      0x8
#define PORTE_ADDR      0x9
#define PCLATH0_ADDR    0xA
#define INTCON0_ADDR    0xB
#define PIR1_ADDR       0xC
#define PIR2_ADDR       0xD
#define TMR1L_ADDR      0xE
#define TMR1H_ADDR      0xF
#define T1CON_ADDR      0x10
#define TMR2_ADDR       0x11
#define T2CON_ADDR      0x12
#define SSPBUF_ADDR     0x13
#define SSPCON_ADDR     0x14
#define CCPR1L_ADDR     0x15
#define CCPR1H_ADDR     0x16
#define CCP1CON_ADDR    0x17
#define RCSTA_ADDR      0x18
#define TXREG_ADDR      0x19
#define RCREG_ADDR      0x1a
#define CCPR2L_ADDR     0x1b
#define CCPR2H_ADDR     0x1c
#define CCP2CON_ADDR    0x1d
#define ADRESH_ADDR     0x1e
#define ADCON0_ADDR     0x1f

// Define offsets inside bank 1:
#define OPTIONREG1_ADDR 0x81
#define PCL1_ADDR       0x82
#define STATUS1_ADDR    0x83
#define FSR1_ADDR       0x84
#define TRISA_ADDR      0x85
#define TRISB_ADDR      0x86
#define TRISC_ADDR      0x87
#define TRISD_ADDR      0x88
#define TRISE_ADDR      0x89
#define PCLATH1_ADDR    0x8a
#define INTCON1_ADDR    0x8b
#define PIE1_ADDR       0x8c
#define PIE2_ADDR       0x8d
#define PCON_ADDR       0x8e
#define OSCCON_ADDR     0x8f
#define OSCTUNE_ADDR    0x90
#define SSPCON2_ADDR    0x91
#define PR2_ADDR        0x92
#define SSPADD_ADDR     0x93
#define SSPSTAT_ADDR    0x94
#define WPUB_ADDR       0x95
#define IOCB_ADDR       0x96
#define VRCON_ADDR      0x97
#define TXSTA_ADDR      0x98
#define SPBRG_ADDR      0x99
#define SPBRGh_ADDR     0x9A
#define PWM1CON_ADDR    0x9B
#define ECCPAS_ADDR     0x9C
#define PSTRCON_ADDR    0x9D
#define ADRESL_ADDR     0x9E
#define ADCON1_ADDR     0x9F

// Define offsets inside bank 2:
#define TMR0_2_ADDR     0x101
#define PCL2_ADDR       0x102
#define STATUS2_ADDR    0x103
#define FSR2_ADDR       0x104
#define WDTCON_ADDR     0x105
#define PORTB2_ADDR     0x106
#define CM1CON0_ADDR    0x107
#define CM2CON0_ADDR    0x108
#define CM2CON1_ADDR    0x109
#define PCLATH2_ADDR    0x10a
#define INTCON2_ADDR    0x10b
#define EEDAT_ADDR      0x10c
#define EEADR_ADDR      0x10d
#define EEDATH_ADDR     0x10e
#define EEADRH_ADDR     0x10f

// Define offsets inside bank 3:
#define OPTIONREG3_ADDR 0x181
#define PCL3_ADDR       0x182
#define STATUS3_ADDR    0x183
#define FSR3_ADDR       0x184
#define SRCON_ADDR      0x185
#define TRISB3_ADDR     0x186
#define BAUDCTL_ADDR    0x187
#define ANSEL_ADDR      0x188
#define ANSELH_ADDR     0x189
#define PCLATH3_ADDR    0x18A
#define INTCON3_ADDR    0x18B
#define EECON1_ADDR     0x18C
#define EECON2_ADDR     0x18D

#endif

#ifndef __my_byte_address__
#define __my_byte_address__

// Bank 0:
#byte TMR0              = TMR0_ADDR
#byte PCL0              = PCL0_ADDR
#byte STATUS0           = STATUS0_ADDR
#byte FSR0              = FSR0_ADDR
#byte PCLATH0           = PCLATH0_ADDR
#byte INTCON0           = INTCON0_ADDR
#byte PIR1              = PIR1_ADDR
#byte PIR2              = PIR2_ADDR
#byte TMR1L             = TMR1L_ADDR
#byte TMR1H             = TMR1H_ADDR
#byte T1CON             = T1CON_ADDR
#byte TMR2              = TMR2_ADDR
#byte T2CON             = T2CON_ADDR
#byte SSPBUF            = SSPBUF_ADDR
#byte SSPCON            = SSPCON_ADDR
#byte CCPR1L            = CCPR1L_ADDR
#byte CCPR1H            = CCPR1H_ADDR
#byte CCP1CON           = CCP1CON_ADDR
#byte RCSTA             = RCSTA_ADDR
#byte TXREG             = TXREG_ADDR
#byte RCREG             = RCREG_ADDR
#byte CCPR2L            = CCPR2L_ADDR
#byte CCPR2H            = CCPR2H_ADDR
#byte CCP2CON           = CCP2CON_ADDR
#byte ADRESH            = ADRESH_ADDR
#byte ADCON0            = ADCON0_ADDR

//Bank 1:
#byte OPTIONREG1        = OPTIONREG1_ADDR
#byte PCL1              = PCL1_ADDR
#byte STATUS1           = STATUS1_ADDR
#byte FSR1              = FSR1_ADDR
#byte PCLATH1           = PCLATH1_ADDR
#byte INTCON1           = INTCON1_ADDR
#byte PIE1              = PIE1_ADDR
#byte PIE2              = PIE2_ADDR
#byte PCON              = PCON_ADDR
#byte OSCCON            = OSCCON_ADDR
#byte OSCTUNE           = OSCTUNE_ADDR
#byte SSPCON2           = SSPCON2_ADDR
#byte PR2               = PR2_ADDR
#byte SSPADD            = SSPADD_ADDR
#byte SSPSTAT           = SSPSTAT_ADDR
#byte WPUB              = WPUB_ADDR
#byte IOCB              = IOCB_ADDR
#byte VRCON             = VRCON_ADDR
#byte TXSTA             = TXSTA_ADDR
#byte SPBRG             = SPBRG_ADDR
#byte SPBRGh            = SPBRGh_ADDR
#byte PWM1CON           = PWM1CON_ADDR
#byte ECCPAS            = ECCPAS_ADDR
#byte PSTRCON           = PSTRCON_ADDR
#byte ADRESL            = ADRESL_ADDR
#byte ADCON1            = ADCON1_ADDR

// Bank 2:
#byte TMR0_2            = TMR0_2_ADDR
#byte PCL2              = PCL2_ADDR
#byte STATUS2           = STATUS2_ADDR
#byte FSR2              = FSR2_ADDR
#byte WDTCON            = WDTCON_ADDR
#byte PORTB2            = PORTB2_ADDR
#byte CM1CON0           = CM1CON0_ADDR
#byte CM2CON0           = CM2CON0_ADDR
#byte CM2CON1           = CM2CON1_ADDR
#byte PCLATH2           = PCLATH2_ADDR
#byte INTCON2           = INTCON2_ADDR
#byte EEDAT             = EEDAT_ADDR
#byte EEADR             = EEADR_ADDR
#byte EEDATH            = EEDATH_ADDR
#byte EEADRH            = EEADRH_ADDR

// Bank 3:
#byte OPTIONREG3        = OPTIONREG3_ADDR
#byte PCL3              = PCL3_ADDR
#byte STATUS3           = STATUS3_ADDR
#byte FSR3              = FSR3_ADDR
#byte SRCON             = SRCON_ADDR
#byte TRISB3            = TRISB3_ADDR
#byte BAUDCTL           = BAUDCTL_ADDR
#byte ANSEL             = ANSEL_ADDR
#byte ANSELH            = ANSELH_ADDR
#byte PCLATH3           = PCLATH3_ADDR
#byte INTCON3           = INTCON3_ADDR
#byte EECON1            = EECON1_ADDR
#byte EECON2            = EECON2_ADDR

typedef struct {
    unsigned P0 : 1; 
    unsigned P1 : 1; 
    unsigned P2 : 1; 
    unsigned P3 : 1; 
    unsigned P4 : 1; 
    unsigned P5 : 1; 
    unsigned P6 : 1; 
    unsigned P7 : 1;
    } __X8_bits;
//

//////////////////////////////////////////////////////////////////////////////////////////

/*--------------------------------------------------------------------------------------*/
// DEFINE PORT<A:E> REGISTER MEMORY: BPort<A:E>.P<0:7>, PORT<A:E> BTris<A:E>.B<0:7> TRIS<A:E>//
/*--------------------------------------------------------------------------------------*/

__X8_bits BPortA;
__X8_bits BPortB;
__X8_bits BPortC;
__X8_bits BPortD;
__X8_bits BPortE;
__X8_bits BTrisA;
__X8_bits BTrisB;
__X8_bits BTrisC;
__X8_bits BTrisD;
__X8_bits BTrisE;

#byte BPortA            = PORTA_ADDR
#byte BPortB            = PORTB_ADDR
#byte BPortC            = PORTC_ADDR
#byte BPortD            = PORTD_ADDR
#byte BPortE            = PORTE_ADDR

#byte PORTA             = PORTA_ADDR
#byte PORTB             = PORTB_ADDR
#byte PORTC             = PORTC_ADDR
#byte PORTD             = PORTD_ADDR
#byte PORTE             = PORTE_ADDR

#define GET_PORTA() PORTA
#define SET_PORTA(x) PORTA = x
#define GET_PORTB() PORTB
#define SET_PORTB(x) PORTB = x
#define GET_PORTC() PORTC
#define SET_PORTC(x) PORTC = x
#define GET_PORTD() PORTD
#define SET_PORTD(x) PORTD = x
#define GET_PORTE() PORTE
#define SET_PORTE(x) PORTE = x

#byte BTrisA            = TRISA_ADDR
#byte BTrisB            = TRISB_ADDR
#byte BTrisC            = TRISC_ADDR
#byte BTrisD            = TRISD_ADDR
#byte BTrisE            = TRISE_ADDR

#byte TRISA             = TRISA_ADDR
#byte TRISB             = TRISB_ADDR
#byte TRISC             = TRISC_ADDR
#byte TRISD             = TRISD_ADDR
#byte TRISE             = TRISE_ADDR

#define GET_TRISA() TRISA
#define SET_TRISA(x) TRISA = x
#define GET_TRISB() TRISB
#define SET_TRISB(x) TRISB = x
#define GET_TRISC() TRISC
#define SET_TRISC(x) TRISC = x
#define GET_TRISD() TRISD
#define SET_TRISD(x) TRISD = x
#define GET_TRISE() TRISE
#define SET_TRISE(x) TRISE = x
//


//////////////////////////////////////////////////////////////////////////////////////////

/*--------------------------------------------------------------------------------------*/
// 74HC595 Supported                                                                    //
/*--------------------------------------------------------------------------------------*/
typedef struct {__int8__ clockPin; __int8__ dataPin; __int8__ laughtPin; __int8__ resetPin;} __74hc595__;
extern void init_74HC595(__int8__ * portSelected, __74hc595__ * configued){
    // Setting output mode:
    if (portSelected == &PORTA)
       TRISA = ~((0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin)));
    else if(portSelected == &PORTB)
        TRISB = ~((0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin)));
    else if(portSelected == &PORTC)
        TRISC = ~((0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin)));
    else if(portSelected == &PORTD)
        TRISD = ~((0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin)));
    else if(portSelected == &PORTE)
        TRISE = ~((0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin)));
    *portSelected |= (0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin);
} 

extern void run8_74HC595( __int8__ * portSelected, __74hc595__ * configued, __uint8__ data){
    tmp = 0;
    for (;tmp<8; tmp++){
        if ((data & (1<< (7- tmp))) >> (7- tmp))
            *portSelected |= 0x01 << configued->dataPin;
        else *portSelected &= ~(0x01 << configued->dataPin);
        delay_us(20);
        *portSelected &= ~(0x01 << configued->clockPin);
        delay_us(20);
        *portSelected |= (0x01 << configued->clockPin);
        delay_us(20);
    }
    *portSelected &= ~(0x01 << configued->laughtPin);
    delay_us(20);
    *portSelected |= (0x01 << configued->laughtPin);
    delay_us(20);
}

extern void run16_74HC595(__int8__ * portSelected, __74hc595__ * configued, __uint16__ data){
    run8_74HC595(portSelected, configued, (__uint8__)(data & 0xff));
    run8_74HC595(portSelected, configued, (__uint8__)((data >> 8) & 0xff));
}

extern void run32_74HC595(__int8__ * portSelected, __74hc595__ * configued, __uint32__ data){
    run16_74HC595(portSelected, configued, (__uint16__)(data & 0xffff));
    run16_74HC595(portSelected, configued, (__uint16__)((data >> 16) & 0xffff));
}

extern void reset_74HC595(__int8__ * portSelected, __74hc595__ * configued){
    tmp = 0;
    *portSelected &= ~(0x01 << configued->resetPin);
    delay_ms(20);
    *portSelected &= ~(0x01 << configued->laughtPin);
    delay_us(20);
    *portSelected |= (0x01 << configued->laughtPin);
    delay_us(20);
    *portSelected |= (0x01 << configued->resetPin);
    delay_us(20);
}
//
#endif