/*
 * File:   newmain.c
 * Author: Khaled Abolneel
 *
 * Created on October 21, 2023, 4:48 PM
 */
// DSPIC33CK256MP506 Configuration Bit Settings
// 'C' source line config statements
// FOSCSEL
//#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
//#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Mode Select bits (EC (External Clock) Mode)

#pragma config ALTI2C2 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 500000 
#include <xc.h>
#include <p33CK256MP506.h>
//#include <util/delay.h>

int dataAN12=0;
int dataAN16=0;
int dataAN7=0;

int currentADC12=0;
int currentADC16=0;
int currentADC7=0;

_Bool FLAG=0;
int first = 1;
int channel1=1;
int state=0;


unsigned short ltlEndian=0;
unsigned short bigEndian=0;
unsigned short ltlEndianAN16=0;
unsigned short bigEndianAN16=0;

// ADC AN12 ISR
void __attribute__((interrupt(no_auto_psv))) _ADCAN12Interrupt(void)
{
    dataAN12 = ADCBUF12; // read conversion result
    IFS6bits.ADCAN12IF = 0; // clear interrupt flag
    
}

// ADC AN16 ISR
void __attribute__((interrupt(no_auto_psv))) _ADCAN16Interrupt(void)
{
    dataAN16 = ADCBUF16; // read conversion result
    IFS6bits.ADCAN16IF = 0; // clear interrupt flag
}

// ADC AN7 ISR
void __attribute__((interrupt(no_auto_psv))) _ADCAN7Interrupt(void)
{
    dataAN7 = ADCBUF7; // read conversion result
    IFS6bits.ADCAN7IF = 0; // clear interrupt flag
}

void __attribute__((interrupt(no_auto_psv))) _SI2C2Interrupt(void)
{
    int temp; 
    if( (I2C2STATbits.R_W == 1) && (I2C2STATbits.D_A == 0) ) // Device address matched 
    {
        temp = I2C2RCV;
        currentADC12=dataAN12;
        ltlEndian=currentADC12&0xFF;
        bigEndian=currentADC12&0xFF00;
        bigEndian=(bigEndian>>8);
        
        currentADC16=dataAN16;
        ltlEndianAN16=currentADC16&0xFF;
        bigEndianAN16=currentADC16&0xFF00;
        bigEndianAN16=(bigEndianAN16>>8);        
        
        if(channel1==1)
        {
            I2C2TRN = 0xA1;
            //I2C2TRN = bigEndian;
        }
        else
        {
            I2C2TRN = 0xb1;
            //I2C2TRN = bigEndianAN16;
        }
        I2C2CON1bits.SCLREL = 1;
    }
    else if ( (I2C2STATbits.R_W == 1) && (I2C2STATbits.D_A == 1) && (I2C2STATbits.ACKSTAT == 0 ))
    {
        temp = I2C2RCV;
        if(channel1==1)
        {
            I2C2TRN = 0xa2;
            //I2C2TRN = ltlEndian;
            channel1=0;
        }
        else
        {
            I2C2TRN = 0xb2;
            //I2C2TRN = ltlEndianAN16;
            channel1=1;
        }
        I2C2CON1bits.SCLREL = 1; 
    }
IFS2bits.SI2C2IF=0;
}
/*void __attribute__((interrupt(no_auto_psv))) _MI2C2Interrupt(void)
{
    LATCbits.LATC7=!(PORTCbits.RC7);
    
    if(first)
    {
        I2C2TRN=0x70;
        //I2C2STATbits.R_W=0;
        while(I2C2STATbits.ACKSTAT==1);
        I2C2CONLbits.PEN=1;
        first=0;
    }
    else
    {
        I2C2TRN=ltlEndian;
        while(I2C2STATbits.ACKSTAT==1);
        I2C2CONLbits.PEN=1;
    }
    IFS2bits.MI2C2IF=0;
}*/
    
void __attribute__((interrupt(no_auto_psv))) _T1Interrupt(void)
{
    LATDbits.LATD10=!(PORTDbits.RD10);
    if(first)
    {
        ADCON3Lbits.CNVCHSEL=12;
        //"connect" channels to ADC and create SW trigger
        ADCON3Lbits.CNVRTCH=1;
        ADCON3Lbits.SHRSAMP=1;
        ADCON3Lbits.SHRSAMP=0;
        first=0;
    }
    else
    {
        ADCON3Lbits.CNVCHSEL=16;
        //"connect" channels to ADC and create SW trigger
        ADCON3Lbits.CNVRTCH=1;
        ADCON3Lbits.SHRSAMP=1;  
        ADCON3Lbits.SHRSAMP=0;
        first=1;    
    }
    /*
    ADCON3Lbits.CNVCHSEL=7;
    //"connect" channels to ADC and create SW trigger
    ADCON3Lbits.CNVRTCH=1;
    ADCON3Lbits.SHRSAMP=1;  
    */
    
    
    //}
    /*else if(state==2)
    {
        state=0;
        //TIMER FLAG
        IFS0bits.T1IF = 0; // clear interrupt flag
    }*/
    //LATCbits.LATC7=!(PORTCbits.RC7);
    //LATCbits.LATC3=!(PORTCbits.RC3);
    IFS0bits.T1IF = 0; // clear interrupt flag
}

void ADCclockSetup (void)
{
    ACLKCON1bits.FRCSEL=1;      // FRC is the clock source for APLL (FFRC=8MHz)
    ACLKCON1bits.APLLPRE=8;     //SETS N1 TO 8 to divide Fvco clock by 8
    APLLFBD1bits.APLLFBDIV=1;   //STES M value to 1, multiplies by Fvco clock   
}

void initializeI2C(void)
{   
    /*CLKDIVbits.DOZEN=0;
    OSCCON=0X7846;
    OSCCON=0X9A57;
    OSCCONbits.NOSC=2;*/
    
    //I2C2BRG=16;//16 FOR 32M/16M***********************************
    I2C2CONLbits.I2CEN=0;        //Enables the I2C1 module, and configures the SDA1 and SCL1 pins as serial port pins
    
    //I2C1CONLbits.I2CSIDL=1;      //Discontinues module operation when device enters Idle mode
    //I2C1CONLbits.SCLREL=1;       //RELEASE THE scl CLOCK (ONLY SLAVE SHOULD HOLD IT LOW)
    //I2C1CONLbits.STREN=0;        //Disables clock stretching
    //I2C1CONLbits.A10M=0;       //Slew rate control is disabled for High-Speed mode (400 kHz)
    //I2C1ADDbits.ADD=0xA1;
    //I2C1CONLbits.ACKDT=0;         //ACK is sent (AT THE END OF RECEIVE)
    //I2C1CONLbits.ACKEN=1;         //Initiates Acknowledge sequence on SDA and SCL pins, and transmits ACKDT data bit (MASTER RECEIVE ACK MODE ONLY - SHOULD NOT BE HERE FOR ACTUAL IMPLEMENTATION)
    //I2C1CONLbits.RCEN=1;          //Enables Receive mode for I2C; automatically cleared by hardware at end of 8-bit receive data byte (WHEN INTERUPT RECEIVED FROM NRF)
    //IEC2bits.MI2C2IE=1;//******************************************
    IEC2bits.SI2C2IE=1;
    I2C2CONLbits.I2CEN=1; 
    
    I2C2CONLbits.A10M=0;//7-BIT SLAVE ADDRESS

    I2C2ADD=0x70;
    I2C2CONLbits.GCEN=0;//ENABLE INTERRUPT when a general call address is received in I2CxRSR; module is enabled for reception
    I2C2CONHbits.SCIE=1;
    I2C2CONLbits.STREN=0;//NO ADDRESS HOLDING
}

void interruptT1_ADCTRG(void)
{
    
    IEC0bits.T1IE=1;
    IFS0bits.T1IF=0;
    
    T1CONbits.TECS = 3; // clock from FRC
    T1CONbits.TCKPS = 3; // 1:256 prescale
    PR1 = 2; // rollover every 4 clocks
    T1CONbits.TON = 1; // start timer to generate ADC triggers*/
}

void OutTestPinsSetup(void)
{
    ANSELCbits.ANSELC3=0;       //Set pin RC3 as digital
    TRISCbits.TRISC3=0;         //Set pin RC3 as output
    
    ANSELDbits.ANSELD11=0;       //Set pin RCD11 as digital
    TRISDbits.TRISD11=0;         //Set pin RD11 as output
    
    ANSELDbits.ANSELD10=0;       //Set pin RCD10 as digital
    TRISDbits.TRISD10=0;         //Set pin RD10 as output
}

void initializeADC(void)
{
    ANSELCbits.ANSELC0=1;       //Set pin RC0 as analog
    TRISCbits.TRISC0=1;         //Set pin RC0 as input
    
    ANSELCbits.ANSELC7=1;       //Set pin RC7 as analog
    TRISCbits.TRISC7=1;         //Set pin RC7 as input
    
    ANSELBbits.ANSELB2=1;       //Set pin RB2 as analog
    TRISBbits.TRISB2=1;         //Set pin RB2 as input    
       
    ADCON1Hbits.SHRRES=3;       //choose 12-bit resolution for shared ADC core
    ADCON3Lbits.REFSEL=0;       //ADC reference voltage VrefH=AVdd & VrefL=AVss
    ADCON2Hbits.SHRSAMC=13;      //how shared ADC core clock cycles to wait for sampling lowest allowed is 2
   
    ADCON3Hbits.CLKSEL=3;       //CLOCK SOURCE IS Fvco/4
    ADCON2Lbits.SHRADCS=1;    //divides Fvco/4 by 200; Fvco=250KHz now shared core clock is 1250Hz 
    
    ADCON1Hbits.FORM = 0;
        
    ADCON3Hbits.CLKDIV=0;      //no division all division takes place in SHRADCS
    
    ADCON5Hbits.WARMTIME = 4;
    ADCON3Hbits.C0EN=0;
    ADCON3Hbits.C1EN=0;
    ADCON1Lbits.ADON=1;         //enable ADC (must be enabled after all ADC bit configuration is done)
    ADCON5Lbits.SHRPWR=1;        //shared ADC core is powered and ready for operation
    while(ADCON5Lbits.SHRRDY==0);
    
    ADCON3Hbits.SHREN=1;         //shared ADC Core is enabled
    
    INTCON2bits.GIE=1;
    
    ADIELbits.IE12=1;           //enable interrupt for AN12
    ADIEHbits.IE16=1;           //enable interrupt for AN16
    ADIELbits.IE7=1;            //enable interrupt for AN16
    
    _ADCAN12IE=1;               //enable interrupt for AN12
    _ADCAN12IF=0;               //clear interrupt flag for AN12
    
    _ADCAN16IE=1;               //enable interrupt for AN16
    _ADCAN16IF=0;               //clear interrupt flag for AN16
    
    _ADCAN7IE=1;               //enable interrupt for AN7
    _ADCAN7IF=0;               //clear interrupt flag for AN7 
}

int main(void)
{
    OutTestPinsSetup();
    ADCclockSetup();
    initializeADC();
    interruptT1_ADCTRG();
    initializeI2C(); 
    
    while(1)
    {
        if((dataAN12<512)&&(dataAN12>0)){
            LATCbits.LATC3=1;
        }else if((dataAN12<1024)&&(dataAN12>512)){
            LATCbits.LATC3=0;
        }else if((dataAN12<1700)&&(dataAN12>=1024)){    
            LATCbits.LATC3=1;
        }else if((dataAN12<2200)&&(dataAN12>=1700)){    
            LATCbits.LATC3=0;
        }else if((dataAN12<=3000)&&(dataAN12>=2200)){
            LATCbits.LATC3=1;
        }else if((dataAN12<3200)&&(dataAN12>=3000)){    
            LATCbits.LATC3=0;
        }else if((dataAN12<3900)&&(dataAN12>=3200)){    
            LATCbits.LATC3=1;
        }else if((dataAN12<=4096)&&(dataAN12>=3900)){
            LATCbits.LATC3=0;
        }
/**************************************************************/        
       
        if((dataAN16<512)&&(dataAN16>0)){
            LATDbits.LATD11=1;
        }else if((dataAN16<1024)&&(dataAN16>512)){
            LATDbits.LATD11=0;
        }else if((dataAN16<1700)&&(dataAN16>=1024)){    
            LATDbits.LATD11=1;
        }else if((dataAN16<2200)&&(dataAN16>=1700)){    
            LATDbits.LATD11=0;
        }else if((dataAN16<=3000)&&(dataAN16>=2200)){
            LATDbits.LATD11=1;
        }else if((dataAN16<3200)&&(dataAN16>=3000)){    
            LATDbits.LATD11=0;
        }else if((dataAN16<3900)&&(dataAN16>=3200)){    
            LATDbits.LATD11=1;
        }else if((dataAN16<=4096)&&(dataAN16>=3900)){
            LATDbits.LATD11=0;
        }
/**************************************************************/
        /*
        if((dataAN7<512)&&(dataAN7>0)){
            LATDbits.LATD10=1;
        }else if((dataAN7<1024)&&(dataAN7>512)){
            LATDbits.LATD10=0;
        }else if((dataAN7<1700)&&(dataAN7>=1024)){    
            LATDbits.LATD10=1;
        }else if((dataAN7<2200)&&(dataAN7>=1700)){    
            LATDbits.LATD10=0;
        }else if((dataAN7<=3000)&&(dataAN7>=2200)){
            LATDbits.LATD10=1;
        }else if((dataAN7<3200)&&(dataAN7>=3000)){    
            LATDbits.LATD10=0;
        }else if((dataAN7<3900)&&(dataAN7>=3200)){    
            LATDbits.LATD10=1;
        }else if((dataAN7<=4096)&&(dataAN7>=3900)){
            LATDbits.LATD10=0;
        }*/
    }
    return 0;
}