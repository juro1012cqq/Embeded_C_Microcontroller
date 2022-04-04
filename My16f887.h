//  Microcontroller 16F887 lib.
//  Author: juro1012cqq
//  Date:   4/3/22
//  License: FREE.
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
#define FSR2_ADD        0x104
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
// DEFINE PORT<A:E> REGISTER MEMORY: P<A:E>.P<0:7>, PORT<A:E> TRS<A:E>.B<0:7> TRIS<A:E>//
/*--------------------------------------------------------------------------------------*/

__X8_bits PA;
__X8_bits PB;
__X8_bits PC;
__X8_bits PD;
__X8_bits PE;
__X8_bits TRSA;
__X8_bits TRSB;
__X8_bits TRSC;
__X8_bits TRSD;
__X8_bits TRSE;

#byte PA = PORTA_ADDR
#byte PB = PORTB_ADDR
#byte PC = PORTC_ADDR
#byte PD = PORTD_ADDR
#byte PE = PORTE_ADDR

#byte PORTA = PORTA_ADDR
#byte PORTB = PORTB_ADDR
#byte PORTC = PORTC_ADDR
#byte PORTD = PORTD_ADDR
#byte PORTE = PORTE_ADDR

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
#define SET_PORT(mode, x) if(mode == 0){PORTA=x;} else if(mode == 1){PORTB = x;} else if (mode == 2) {PORTC=x;} else if (mode == 3) {PORTD=x;} else if (mode == 4) {PORTE=x;}

#byte TRSA = TRISA_ADDR
#byte TRSB = TRISB_ADDR
#byte TRSC = TRISC_ADDR
#byte TRSD = TRISD_ADDR
#byte TRSE = TRISE_ADDR

#byte TRISA = TRISA_ADDR
#byte TRISB = TRISB_ADDR
#byte TRISC = TRISC_ADDR
#byte TRISD = TRISD_ADDR
#byte TRISE = TRISE_ADDR

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
#define SET_TRIS(mode, x) if(mode == 0){TRISA=x;} else if(mode == 1){TRISB = x;} else if (mode == 2) {TRISC=x;} else if (mode == 3) {TRISD=x;} else if (mode == 4) {TRISE=x;}
//


//////////////////////////////////////////////////////////////////////////////////////////

/*--------------------------------------------------------------------------------------*/
// 74HC595 Supported                                                                    //
/*--------------------------------------------------------------------------------------*/
#define PRTA 0
#define PRTB 1
#define PRTC 2
#define PRTD 3
#define PRTE 4

typedef struct {__int8__ clockPin; __int8__ dataPin; __int8__ laughtPin; __int8__ resetPin;} __74hc595__;
extern void init_74HC595(__int8__ portSelected, __74hc595__ * configued){
    // Setting output mode:
    SET_TRIS(portSelected, ~((0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin)));
    SET_PORT(portSelected,(0x01 << configued->clockPin) | (0x01 << configued->dataPin) | (0x01 << configued->laughtPin) | (0x01 << configued->resetPin));
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

