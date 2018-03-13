 //
// FILE     main.c
// DATE     140128
// WRITTEN  RAC
// PURPOSE  Utilities to communicate with FM receiver IC.   $
// LANGUAGE MPLAB C18
// KEYWORDS USB/I2C/SPARKFUN/FM_MODULE/RECEIVER
// PROJECT  FM TxRx Lab experiment
// CATEGORY UTILITY
// TARGET   Darwin
//
//


// This is skeletal code which won't run until 'fleshed out'.
// It does give an idea of the setup and initialization required
// of the PIC and the FM tuner module.  The basics of communication
// over the I2C bus are also covered.
// 
// This version contains revisions needed to compile under MPLABX.
// 
// 



#pragma config OSC = INTIO7     // Internal osc, RA6=CLKO, RA7=I/O
#pragma config FCMEN = OFF		// Fail-Safe Clock Monitor disabled 
#pragma config IESO = OFF		// Oscillator Switchover mode disabled 
#pragma config WDT = OFF        // WDT disabled (control through SWDTEN bit)
#pragma config PWRT = OFF       // racmod  -> PWRT disabled
#pragma config MCLRE = ON       // MCLR pin enabled; RG5 input pin disabled
#pragma config XINST = OFF      // Instruction set extension disabled
#pragma config BOREN = OFF      // Brown-out controlled by software
#pragma config BORV = 3         // Brown-out voltage set for 2.0V, nominal
#pragma config STVREN = OFF		// Stack full/underflow will not cause Reset
#pragma config CP = OFF			// Program memory block not code-protected 



#include <plib/i2c.h>

#include "fm.h"




// FM register bank defaults -
const unsigned int regDflt[18] = {
	0xFFFF,     // R0 -- the first writable register .  (disable xo_en)   
	0x5B15,     // R1.   
	0xD0B9,     // R2.   Digit 9 is TUNE
	0xA010,     // R3   seekTHD = 16   
	0x0780,     // R4   
	0x28AB,     // R5   
	0x6400,     // R6   
	0x1EE7,     // R7   
	0x7141,     // R8   
	0x007D,     // R9   
	0x82C6,     // R10  disable wrap   
	0x4F55,     // R11. <--- (disable xo_output)   
	0x970C,     // R12.   
	0xB845,     // R13   
	0xFC2D,     // R14   
	0x8097,     // R15   
	0x04A1,     // R16   
	0xDF6A      // R17
};

unsigned int regImg[18];	// FM register bank images

unsigned int freq;



/*
 * Obtain latest change in state for the pushbutton set.
 *
 * @param butn Which button changed.  See fm.h.
 *
 * Returns 0 if nothing changed
 * 
 * Returns 1 if button pressed
 * 
 * Returns 2 if button released
 *
 */
unsigned char butnEvent(unsigned char *btn) 
{
    if (PORTBbits.RB0 == 0)
    {
        *btn = BUTN1;
        dly(100);
        return 1;
    }
    
    if (PORTBbits.RB5 == 0)
    {
        *btn = BUTN2;
        dly(100);
        return 1;
    }
    
    if (PORTAbits.RA0 == 0)
    {
        *btn = BUTN3;
        dly(100);
        return 1;
    }
    
    if (PORTAbits.RA1 == 0)
    {
        *btn = BUTN4;
        dly(100);
        return 1;
    }
    
    if (PORTGbits.RG0 == 0)
    {
        *btn = BUTN5;
        dly(100);
        return 1;
    }
    
    if (PORTGbits.RG1 == 0)
    {
        *btn = BUTN6;
        dly(100);
        return 1;
    }
    
    if (PORTGbits.RG2 == 0)
    {
        *btn = BUTN7;
        dly(100);
        return 1;
    }
    
    if (PORTGbits.RG3 == 0)
    {
        *btn = BUTN8;
        dly(100);
        return 1;
    }
    
    
    return 0;
    
}
//
// end butnEvent ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

void dly(int d) { // does delay in 10ths of milliseconds, so for 10ms delay d = 100

	int i = 0;

	for ( ; d; --d) 
		for (i = 1000;  i;  --i) ;
}
//
// end dly ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

/*
 * Set all LCD segments to 0 (off, clear).
 *
 */
void clrscn() {

	int i = 0;
	unsigned char *CLEARptr;        // Pointer used to clear all LCDDATA
	

	for (	i = 0,
			CLEARptr = (unsigned char *) &LCDDATA0;  // Point to first segment
			i < 28; 
			i++)		// Turn off all segments
		*CLEARptr++ = 0x00;
}
//
// end clrscn ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

void Init() {

	int i;

	OSCCON = 0b01110010;        	// Select 8 MHz internal oscillator
	LCDSE0 = 0b11111111;        	// Enable  LCD segments 07-00
	LCDSE1 = 0b11111111;        	// Enable  LCD segments 15-08
	LCDSE2 = 0b11111111;        	// Enable  LCD segments 23-16
	LCDSE3 = 0b00000000;        	// Disable LCD segments 31-24
	LCDCON = 0b10001000;         	// Enab LC controller. Static mode. INTRC clock
	LCDPS  = 0b00110110;         	// 37 Hz frame frequency
	ADCON1 = 0b00111111;        	// Make all ADC/IO pins digital
	TRISA = 0b00000011;             // RA0 and RA1 pbutton
	TRISB = 0b00100001;				// RB0 and RB5 pbutton
	TRISC = 0b00011000;				// RC3 and RC4 do the I2C bus
	TRISG = 0b11111111;				// RG0, RG1 & RG3 pbutton
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
    PORTG = 0;
    INTCONbits.TMR0IF = 0;          // Clear timer flag
	//T0CON = 0b00000011;				// Prescale by 16
    T0CON = 0b00001000;             // No prescale
    TMR0H = 0;                      // Clear timer count
    TMR0L = 0;
    T0CONbits.TMR0ON = 1;           // Start timer
	OpenI2C( MASTER, SLEW_OFF);
	SSPADD = 0x3F;
}
//
// end Init ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

/*
 * Write an individual LCD segment.  
 *
 * @param segOrd The segment ordinal.  Between 0 and 23.
 *
 * @param state Whether to turn the segment dark (true) or clear (false).
 *
 */
void segWrt(unsigned char segOrd,  unsigned char state) {

	unsigned char bitSelect;
	unsigned char *LCReg;

	if (segOrd > 23) return;
	LCReg = (unsigned char *)&LCDDATA0 + (segOrd >> 3);
	bitSelect = 1 << (segOrd & 0x07);
	if (state) *LCReg  |=  bitSelect;		// Segment on
	else *LCReg &= ~bitSelect;				// Segment off
}
//
// end segWrt ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//





/*
 * FMwrite() -  Write a two byte word to the FM module.  The new 
 * register contents are obtained from the image bank.
 *
 * @param adr The address of the register in the FM module that needs 
 * to be written.
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMwrite(unsigned char adr) {

	unsigned char firstByt;
	unsigned char secndByt;

	firstByt = regImg[adr] >> 8;
	secndByt = regImg[adr];

	StartI2C();					// Begin I2C communication
	IdleI2C();

	// Send slave address of the chip onto the bus
	if (WriteI2C(FMI2CADR)) return XF;
	IdleI2C();
	WriteI2C(adr);				// Address the internal register
	IdleI2C();
	WriteI2C(firstByt);			// Ask for write to FM chip
	IdleI2C();
	WriteI2C(secndByt);
	IdleI2C();
	StopI2C();
	IdleI2C();
	return XS;
}
//
// end FMwrite ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//



/*
 * FMread - Read a two byte register from the FM module.
 *
 * @param regAddr The address of the register in the module that needs 
 *        to be read.
 *
 * @param data Where to store the reading.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMread(unsigned char regAddr, unsigned int *data) {

	unsigned char firstByt;
	unsigned char secndByt;

	StartI2C();					// Begin I2C communication
	IdleI2C();					// Allow the bus to settle

	// Send address of the chip onto the bus
	if (WriteI2C(FMI2CADR)) return XF;	
	IdleI2C();
	WriteI2C(regAddr);			// Address the internal register
	IdleI2C();
	RestartI2C();				// Initiate a RESTART command
	IdleI2C();
	WriteI2C(FMI2CADR + DEVRD);	// Ask for read from FM chip
	IdleI2C();
	firstByt = ReadI2C(); 		// Returns the MSB byte
	IdleI2C();
	AckI2C();					// Send back Acknowledge
	IdleI2C();
	secndByt = ReadI2C();		// Returns the LSB of the temperature
	IdleI2C();
	NotAckI2C();
	IdleI2C();
	StopI2C();
	IdleI2C();
	*data = firstByt;
	*data <<= 8;
	*data = *data | secndByt;

	return XS;
}
//
// end FMread ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



/*
 * FMready - See if the FM module is ready.
 *
 * @param rdy Where to store the busy/ready status.  Will become
 * non-zero if the chip is ready, zero if busy.
 * 
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMready(unsigned int *rdy) {

	unsigned int sts;

	if (FMread(FMCHIPSTSADR, &sts)  != XS) return XF;
	sts &= FMASKSTATUS;
	*rdy = sts ? TRUE : FALSE;
	return XS;
}
//
// end FMready ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMinit() -  Initialise the FM module.  
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMinit() {

	unsigned char ad;
	unsigned int dat;

	// Copy default FM register values to the image set -
	for(ad = 0; ad < 18; ad++) regImg[ad] = regDflt[ad];

	dat = regImg[0];
	regImg[0] &= ~1;
	if (FMwrite(0) != XS) return  XF;
	for(ad = 1; ad < 18; ad++) {
		if (FMwrite(ad) != XS)return XF;
	}

	regImg[0] = dat | 1;
	if (FMwrite(0) != XS) return XF;
	dly(20);
	while (FMready(&dat), !dat) dly(2);
	showFreq(freq);
	return XS;
}
//
// end FMinit ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//




/*
 * FMfrequenc(f) -  Tune the FM module to new frequency.  
 *
 *
 * @param f The new frequency as a multiple of 100 kHz.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMfrequenc(unsigned int f) {

	unsigned int dat;
	unsigned int cn;		// AR1010 channel number

	cn = f - 690;

	// NB AR1010 retunes on 0 to 1 transition of TUNE bit -	
	regImg[2] &= ~FMASKTUNE;
	if (FMwrite(2) != XS) return XF;
	regImg[2] &= 0xfe00; 
	regImg[2] |= (cn | FMASKTUNE);
	if (FMwrite(2) != XS) return XF;
	do {
		dly(2);
		if (FMready(&dat) != XS) return XF;
	} while (!dat);
    
    return XS;
}
//
// end FMfrequenc ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//


/*
 * FMvers - Obtain the FM chip version.
 *
 * @param vsn Where to store the version number.  Will become
 * 0x65B1 for vintage 2009 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMvers(unsigned int *vsn) {
	if (FMread(FMCHIPVERSADR, vsn)  != XS) return XF;
	return XS;
}
//
// end FMvers ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMid - Obtain the FM chip ID.
 * * @param id Where to store the ID number.  Will become
 * 0x1010 for AR1010 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMid(unsigned int *id) {

	if (FMread(FMCHIPIDADR, id)  != XS) return XF;
	return XS;
}
//
// end FMid ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//




/*
 * nextChan() -  Tune to the next channel.
 *
 * @param up Set to non-zero for frequency increase,
 *  zero for frequency down.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char nextChan(unsigned char up) {

    if(up){
        if((freq + 1)<= FMHIGHCHAN){
            freq += 1;
            mute(TRUE);
            FMfrequenc(freq);
            mute(FALSE);
            return XS;
        }
    }else if(!up){
        if((freq - 1)>= FMLOWCHAN){
            freq -= 1;
            mute(TRUE);
            FMfrequenc(freq);
            mute(FALSE);
            return XS;
        }
    }
    return XF;
    
}
//
// end nextChan ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//


/*
 * scanNext() -  Tune to the next channel.
 *
 * @param up Set to non-zero for next channel up,
 *  zero for preset down.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char scanNext(unsigned char up){
    
    if(up){
        
    }else if(!up){
        
    }
    return XF;
    
}

//
// end scanNext ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//

unsigned char FMreadChan(){
    unsigned int newFreq = 0;
    FMread(13,&newFreq);
    newFreq &= FMASKRDCHAN;
    return XS;
}

/*
 * errfm() -  Firmware error.   Call this on a showstopper.
 *
 *
 * @return Never!
 *
 */
void errfm() {

	;		// Do something helpful
	for(;;) ;
}
//
// end errfm ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//


/*
 * mute() -  Firmware error.   Call this on a showstopper.
 *
 * @param mute whether to mute or unmute the device
 *
 * @return success or failure
 *
 */
unsigned char mute(char mute) {
    if(mute){
        regImg[1] |= FMASKMUTE;
        return FMwrite(1);
    }else{
        regImg[1] &= ~FMASKMUTE;
        return FMwrite(1);
    }	
}
//
// end mute ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//

/*
 * Display the frequency that the receiver chip is set to.
 *
 * @return XS if successful, else XF on failure.
 *
 */
unsigned char showFreq(unsigned int frequency) {

    int count = 0, i, num = 10;
    
    clrscn();
    segWrt(22, TRUE);
    
    while(frequency != 0)
    {
        frequency /= 10;
        ++count;
    }
    
    if (count == 4)
    {
        segWrt(21, TRUE);
        count--;
    }
    
    for (i = 1; i++; i<=count)
    {
        int digit;
        
        digit = (frequency/num)%10;
        createDigit(digit, i);
        num = num*10;
    }
    
    ;		// Etc
    return XS;
}
// int freq1 freq2
//  freq1 = frequency; freq2 = frequency/10
//  freq1 -= freq2*10
// end showFreq ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


void createDigit(unsigned int digit, unsigned char seg){
    
    int a, b, c, d, e, f, g;
    if (seg == 3)
    { 
        a = 0;
        b = 1;
        c = 2;
        d = 3;
        e = 4;
        f = 5;
        g = 6;
    }
    
    if (seg == 2)
    { 
        a = 7;
        b = 8;
        c = 9;
        d = 10;
        e = 11;
        f = 12;
        g = 13;
    }
    
    if (seg == 1)
    { 
        a = 14;
        b = 15;
        c = 16;
        d = 17;
        e = 18;
        f = 19;
        g = 20;
    }
    
    if (digit == 0)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(c, TRUE);
        segWrt(d, TRUE);
        segWrt(e, TRUE);
        segWrt(f, TRUE);  
    }
    
    if (digit == 1)
    {
        segWrt(b, TRUE);
        segWrt(c, TRUE);
    }
    
    if (digit == 2)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(g, TRUE);
        segWrt(e, TRUE);
        segWrt(g, TRUE);
    }
    if (digit == 3)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(g, TRUE);
        segWrt(c, TRUE);
        segWrt(d, TRUE);
    }
    if (digit == 4)
    {
        segWrt(f, TRUE);
        segWrt(g, TRUE);
        segWrt(b, TRUE);
        segWrt(c, TRUE);
    }
    
    if (digit == 5)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(g, TRUE);
        segWrt(e, TRUE);
        segWrt(d, TRUE);
    }
    
    if (digit == 6)
    {
        segWrt(a, TRUE);
        segWrt(f, TRUE);
        segWrt(g, TRUE);
        segWrt(e, TRUE);
        segWrt(c, TRUE);
        segWrt(d, TRUE);
    }
    if (digit == 7)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(c, TRUE);
    }
    
    if (digit == 8)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(c, TRUE);
        segWrt(d, TRUE);
        segWrt(e, TRUE);
        segWrt(f, TRUE);
        segWrt(g, TRUE);
    }
    
    if (digit == 9)
    {
        segWrt(a, TRUE);
        segWrt(b, TRUE);
        segWrt(c, TRUE);
        segWrt(f, TRUE);
        segWrt(g, TRUE);
    }
}

void setVolume (int vol)
{

    int a;
    
    unsigned int current1, current2, current;
    
    
    current1 = (regImg[3]&FMASKVOL1) << 1;
    current2 = (regImg[14]&FMASKVOL2);
    
    current = (current1 & current2);
    

     
    if (vol == TRUE)    //Volume increased
    {
        switch (current)
        {
            case 0x0F00 :               //Volume 0->1
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0xC000; 
                FMwrite(14);
                break;
            
            case 0xCF00 :               //Volume on 1->2
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0xD000;
                FMwrite(14);
                break;
            
            case 0xDF00 :               //Volume on 2->3
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
                
            case 0xFF00 :               //Volume on 3->4
                regImg[3] |= 0x0580;
                FMwrite(3);
                regImg[14] |= 0xC000;
                FMwrite(14);
                break;
                
            case 0xCB00 :               //Volume on 4->5 
                regImg[3] |= 0x0580;
                FMwrite(3);
                regImg[14] |= 0xD000;
                FMwrite(14);
                break;
            case 0xDB00 :               //Volume on 5->6
                regImg[3] |= 0x0580;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xFB00 :               //Volume on 6->7
                regImg[3] |= 0x0500;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xFA00 :               //Volume on 7->8 
                regImg[3] |= 0x0480;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF900 :               //Volume on 8->9 
                regImg[3] |= 0x0400;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF800 :               //Volume on 9->10 
                regImg[3] |= 0x0380;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF700 :               //Volume on 10->11 
                regImg[3] |= 0x0300;
                FMwrite(3);
                regImg[14] |= 0xD000;
                FMwrite(14);
                break;
            case 0xD600 :               //Volume on 11->12 
                regImg[3] |= 0x0300;
                FMwrite(3);
                regImg[14] |= 0xE000;
                FMwrite(14);
                break;
            case 0xE600 :               //Volume on 12->13 
                regImg[3] |= 0x0300;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF600 :               //Volume on 13->14
                regImg[3] |= 0x0180;
                FMwrite(3);
                regImg[14] |= 0xE000;
                FMwrite(14);
                break;
            case 0xE300 :               //Volume on 14->15 
                regImg[3] |= 0x0180;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF300 :               //Volume on 15->16 
                regImg[3] |= 0x0100;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF200 :               //Volume on 16->17 
                regImg[3] |= 0x0080;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF100 :               //Volume on 17->18 
                regImg[3] |= 0x0000;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF100 :               //Volume on 18 
                //Do nothing as volume already full
                break;
            
            default : break;     
        }
    }
    
    if (vol == FALSE)   // Volume decreased
    {
        switch (current)
        {
            case 0x0F00 :               //Volume 0
                // Do nothing as volume already low
                break;
            
            case 0xCF00 :               //Volume on 1->0
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0x0000;
                FMwrite(14);
                break;
            
            case 0xDF00 :               //Volume on 2->1
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0xC000; 
                FMwrite(14);
                break;
                
            case 0xFF00 :               //Volume on 3->2
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0xD000;
                FMwrite(14);
                break;
                
            case 0xCB00 :               //Volume on 4->3 
                regImg[3] |= 0x0780;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xDB00 :               //Volume on 5->4
                regImg[3] |= 0x0580;
                FMwrite(3);
                regImg[14] |= 0xC000;
                FMwrite(14);
                break;
            case 0xFB00 :               //Volume on 6->5
                regImg[3] |= 0x0580;
                FMwrite(3);
                regImg[14] |= 0xD000;
                FMwrite(14);
                break;
            case 0xFA00 :               //Volume on 7->6 
                regImg[3] |= 0x0580;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF900 :               //Volume on 8->7 
                regImg[3] |= 0x0500;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF800 :               //Volume on 9->8 
                regImg[3] |= 0x0480;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF700 :               //Volume on 10->9
                regImg[3] |= 0x0400;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xD600 :               //Volume on 11->10 
                regImg[3] |= 0x0380;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xE600 :               //Volume on 12->11 
                regImg[3] |= 0x0300;
                FMwrite(3);
                regImg[14] |= 0xD000;
                FMwrite(14);
                break;
            case 0xF600 :               //Volume on 13->12
                regImg[3] |= 0x0300;
                FMwrite(3);
                regImg[14] |= 0xE000;
                FMwrite(14);
                break;
            case 0xE300 :               //Volume on 14->13 
                regImg[3] |= 0x0300;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF300 :               //Volume on 15->14 
                regImg[3] |= 0x0180;
                FMwrite(3);
                regImg[14] |= 0xE000;
                FMwrite(14);
                break;
            case 0xF200 :               //Volume on 16->15 
                regImg[3] |= 0x0180;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF100 :               //Volume on 17->16 
                regImg[3] |= 0x0100;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            case 0xF100 :               //Volume on 18->17
                regImg[3] |= 0x0080;
                FMwrite(3);
                regImg[14] |= 0xF000;
                FMwrite(14);
                break;
            
            default : break;     
        }

            
    }
}


void main(void) {

	unsigned char btn;
	unsigned char evt;
	unsigned int ui;

	dly(2);
	Init();
	//FMvers(&ui);									// Check we have comms with FM chip
	//if (ui != 0x1010) errfm();
	//if (FMinit() != XS) errfm();
	for (;;) {
        
		evt = butnEvent(&btn);
        
        if (evt == 1)
            switch (btn)
            {
                case BUTN1 : nextChan(TRUE); break;
                case BUTN2 : nextChan(FALSE); break;
                case BUTN3 : ; break;
                case BUTN4 : ; break;
                case BUTN5 : setVolume(TRUE); break;
                case BUTN6 : setVolume(FALSE); break;
                case BUTN7 : ; break;
                case BUTN8 : errfm(); break;
                
                default : break;
            }
            
        }
        
        /*int x;
        for(x = 0; x < 23;x++){
            segWrt(x, TRUE);
        }
        */
        
        //createDigit(8,1);
        
        //PORTFbits.RF5 = 1;
	}
//
// end main ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
