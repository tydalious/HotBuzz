/* 
 * File:   main.c
 * Author: td1486
 *
 * Created on January 9, 2014, 12:52 PM
 */
#include <p24fxxxx.h>
#include <adc.h>
#include <uart.h>

// FBS
#pragma config BWRP = OFF               // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF                // Boot segment Protect (No boot flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Flash Write Protect (General segment may be written)
#pragma config GSS0 = OFF               // General Segment Code Protect (No Protection)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC Oscillator (FRC))
#pragma config SOSCSRC = ANA            // SOSC Source Type (Analog Mode for use with crystal)
#pragma config LPRCSEL = HP             // LPRC Power and Accuracy (High Power/High Accuracy)
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-speed Start-up disabled))

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF           // CLKO Pin I/O Function (CLKO output signal enabled)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range (Primary Oscillator/External Clock frequency >8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switching and Fail-safe Clock Monitor Enabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128            // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = SWON            // Watchdog Timer Enable bits (WDT controlled with SWDTEN bit setting)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected (windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default SCL1/SDA1 Pins for I2C1)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset at 1.8V)
#pragma config MCLRE = ON               // MCLR Pin Enable bit (RA5 input disabled; MCLR enabled)

// FICD
#pragma config ICS = PGx3               // ICD Pin Placement Select (EMUC/EMUD share PGC3/PGD3)

unsigned int ADCResult[16],DataAvailable=0;
unsigned char Rxdata[10];
unsigned char DataAvail=0;
unsigned char Txdata[] = "UART Enabled...";
int showAvgValues = 0;
int threshold = 4;

void printC(unsigned char str);
void initUART();
void checkRXbuff();
////*************** Interrupt Service routine for UART1 Transmission *************************************
//void __attribute__ ((interrupt,no_auto_psv)) _U1TXInterrupt(void)
//{
//  static unsigned int i=0;
//  U1TX_Clear_Intr_Status_Bit;  	//clear the interrupt status of UART1 TX
//  if(Txdata[i]!='\0')			//check for end of string
//  {
//   while(BusyUART1());			//wait till the UART is busy
//   WriteUART1((unsigned int)Txdata[i++]);//Transmit the data to hyper terminal of computer
//  }
//  else
//  DisableIntU1TX; 	//disable the UART TX interrupt after end of complete transmission
//
//}

//*************** Interrupt Service routine for UART1 reception *************************************
//void __attribute__ ((interrupt,no_auto_psv)) _U1RXInterrupt(void)
//{
//   static unsigned int j=0;
//   U1RX_Clear_Intr_Status_Bit;  	//clear the interrupt status of UART1 RX
//
//   while(!DataRdyUART1());	//wait for data reception on RX
//   Rxdata[j++] = ReadUART1();	//Read the data from UART buffer
//   if(j == 10)
//   DataAvail=1;		//Set the DataAvailability flag after complete reception
// }
//
//************************ Interrupt service routine for ADC ***********************************
void __attribute__((interrupt,no_auto_psv)) _ADC1Interrupt(void)
{
 static unsigned char j=0;
 ADCResult[j]= ReadADC10(j);	//Read the ADC conversion results to ADCResult variable
 j++;
 ADC1_Clear_Intr_Status_Bit;	//Clear interrupt status bit
 if (j==16) 		//Are 16 samples acquired.
  {
   DisableIntADC1;	//If so, disable the ADC
   DataAvailable=1;	//Set th e data available flag bit to interpret the ADC conversion results
  }
}

//
int main(void)
{
    int j;
    initUART();
    for(j = 0; j< sizeof(Txdata); j++)
    {
        if(Txdata[j] == '\0') break;
        printC(Txdata[j]);
    }
    
    ANSBbits.ANSB1 = 0;
    TRISBbits.TRISB1 = 0;
    PORTBbits.RB1 = 0;
    unsigned char i, ch;
    unsigned int channel;

    while(1)
    {
        checkRXbuff();
//        if (U1STAbits.URXDA) {
//            ch = ReadUART1();
//            while(U1STAbits.UTXBF);
//            WriteUART1(ch);
//        }
        CloseADC10();	//Turn off ADC in case if it was operational previously
        ANSBbits.ANSB0 = 1;
        TRISBbits.TRISB0 = 1;


//        unsigned char test[] = "test";
//        printS(test);
        /************** ADC configuration **********************************
        *
        *	Use AN2 channel for siganl sampling
        *	ADC uses internal RC as source of clock
        *	Auto scan is enabled
        *	conversion clcok is selected to 254Tcy
        *	Sampling interval is selected to 17 TAD
        *	ADC interrupt after 16 conversions
        *********************************************************************/
//        channel= ADC_CH0_POS_SAMPLEA_AN2;  //use AN2 for signal sampling
        AD1CHSbits.CH0SA = 0b0010;
        
        //config1 = ADC_MODULE_OFF | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON ;
        AD1CON1bits.SSRC = 0b111; //ADC_CLK_AUTO - Internal counter ends sampling and starts conversion
        AD1CON1bits.ASAM = 0b1; //ADC_AUTO_SAMPLING_ON
        //config2 = ADC_SCAN_ON | ADC_INTR_16_CONV ;
        AD1CON2bits.CSCNA = 0b1;
        AD1CON2bits.SMPI = 0b1111;
        //config3 = ADC_SAMPLE_TIME_17 | ADC_CONV_CLK_254Tcy;
        AD1CON3bits.SAMC = 0b10001;
        AD1CON3bits.ADCS = 0b11111;
        //configportl = 0x0000;
//        configscanl = ADC_SCAN_AN2 ;
        AD1CSSL = 0x4;
       // OpenADC10(config1,config2,config3,configportl,configporth,configscanl,configscanh);
        ConfigIntADC10(ADC_INT_ENABLE|ADC_INT_PRI_3); 	//configure ADC interrupt
        
        EnableADC1;    			//turn on the ADC
//        SetChanADC10(channel);    		//Set the channel to AN2
        while(!DataAvailable);			//wait till the data becomes avaialble
        DataAvailable=0;    		//clear the Data available flag bit

        int average = 0;
        int sum = 0;
        i = 0;
        while(i <10 )
        {
            EnableADC1;     		// Turn on the A/D converter
            while(BusyADC10());
            DisableADC1;    		// Turnoff the A/D converter
            ADCResult[1] = ReadADC10(0);//Read the conversion results from buffer to ADCResults
            sum = sum + ADCResult[1];
            i++;
        }

        CloseADC10(); 		//Close the ADC modue

        average = sum / 10;
        int adjValue;
        if(average < 0x0028)
        {
            adjValue = 0;
        }
        else if(average >= 0x0028 && average < 0x003A)
        {
            adjValue = 1;
        }
        else if(average >= 0x003A && average < 0x0043)
        {
            adjValue = 2;
        }
        else if(average >= 0x0043 && average < 0x0050)
        {
            adjValue = 3;
        }
        else if(average >= 0x0050 && average < 0x005C)
        {
            adjValue = 4;
        }
        else if(average >= 0x005C && average < 0x0066)
        {
            adjValue = 5;
        }
        else if(average >= 0x0066 && average < 0x0071)
        {
            adjValue = 6;
        }
        else if(average >= 0x0071 && average < 0x007F)
        {
            adjValue = 7;
        }
        else if(average >= 0x007F)
        {
            adjValue = 8;
        }


        if(showAvgValues)
        {
            while(U1STAbits.UTXBF);
            WriteUART1('\n');
            while(U1STAbits.UTXBF);
            WriteUART1('\r');
            while(U1STAbits.UTXBF);
            WriteUART1(':');
            while(U1STAbits.UTXBF);
            WriteUART1(adjValue+48);
        }
        
        if(adjValue > threshold)
        {
            //power on
            PORTBbits.RB1 = 1;
        }
        else
        {
            //power off
            PORTBbits.RB1 = 0;
        }

    }//end of program
}

void printC(unsigned char str)
{
    while(!U1STAbits.TRMT);
    WriteUART1(str);
}

void checkRXbuff()
{
    unsigned char ch;
    if(U1STAbits.URXDA)
    {
        ch = ReadUART1();
        while(U1STAbits.UTXBF);
        WriteUART1(ch);
        
        if(ch == 's')
        {
            showAvgValues = 1;
        }

        if(ch == 'd')
        {
            showAvgValues = 0;
        }

        if(ch == 't')
        {
            while(U1STAbits.UTXBF);
//            WriteUART1('\n');
            while(U1STAbits.UTXBF);
            WriteUART1(':');
            while(U1STAbits.UTXBF);
            WriteUART1(threshold+48);
        }

        if(ch == 'c')
        {
            while(U1STAbits.UTXBF);
            while(U1STAbits.URXDA);
            threshold = ReadUART1() - 48;
        }
    }
}

void initUART( )
{
//    U1STA = 0;
//    TRISBbits.TRISB2 = 1;
//    OpenUART1(UART_EN | UART_IDLE_CON | UART_IrDA_DISABLE | UART_MODE_SIMPLEX | UART_UEN_00 |
//            UART_EN_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD |
//            UART_NO_PAR_8BIT | UART_BRGH_FOUR | UART_1STOPBIT,
//            UART_TX_ENABLE , 103);
//
//    U1STA = 0;
//    U1MODE = 0x8000;
//
//
//    U1STAbits.UTXEN = 1;
//    TRISBbits.TRISB7 = 0;
//    TRISBbits.TRISB2 = 1;

    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB2 = 1;
    ANSBbits.ANSB2 = 0;

    U1BRG = 103;
    U1STA = 0;
    U1MODE = 0x8000;
    U1STAbits.UTXEN = 1;
//    U1MODEbits.LPBACK = 1;
}

//int main()
//{
//    initUART();
//
//    unsigned char ch;
//
//    WriteUART1('A');
//
//    while(1)
//    {
//        if (U1STAbits.URXDA) {
//            ch = ReadUART1();
//            while(U1STAbits.UTXBF);
//            WriteUART1(ch);
//        }
//     }
//}