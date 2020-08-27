
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
// external function prototypes
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);


// Prototype statements for functions found within this file.
void Gpio_select(void);
void Setup_ePWM(void);
void ConfigAdc(void);

interrupt void adc_isr(void);

unsigned int phaseShiftADC;
unsigned int dcBusVoltADC;
unsigned int batteryVoltADC;
unsigned int batteryCurrADC;

//###########################################################################
//						main code									
//###########################################################################
void main(void)
{


	InitSysCtrl();	// Basic Core Init from DSP2833x_SysCtrl.c

//	EALLOW;
 //  	SysCtrlRegs.WDCR= 0x00AF;	// Re-enable the watchdog
  // 	EDIS;			// 0x00AF  to NOT disable the Watchdog, Prescaler = 64

	DINT;				// Disable all interrupts
	
	Gpio_select();		// GPIO9, GPIO11, GPIO34 and GPIO49 as output
					    // to 4 LEDs at Peripheral Explorer
	Setup_ePWM();		// init of ePWM1A

	InitPieCtrl();		// basic setup of PIE table; from DSP2833x_PieCtrl.c
	InitPieVectTable();	// default ISR's in PIE

	InitAdc();          // Basic ADC setup, incl. calibration
	AdcRegs.ADCTRL1.bit.RESET = 1;
	ConfigAdc();

	EALLOW;
	    PieVectTable.ADCINT = &adc_isr;   //da ne bismo menjali standardnu funkciju iz defaultisr
	EDIS;

	PieCtrlRegs.PIEIER1.bit.INTx6 = 1; //enable ADC interupt

	IER |=1;  //enable samo INT1, tu se nalazi ADC interupt

	EINT;
	ERTM;


	while(1)
	{    
	    if(GpioDataRegs.GPADAT.bit.GPIO9 == 1)
	    {
	        GpioDataRegs.GPASET.bit.GPIO11 = 1;
	    EPwm2Regs.TBPHS.half.TBPHS= EPwm2Regs.TBPRD / 2;
	    }
	    else
	    {
	        GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	    EPwm2Regs.TBPHS.half.TBPHS= 0;
	    }
	}
} 

void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = 0;		// GPIO15 ... GPIO0 = General Puropse I/O
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;	// ePWM1A active-- first leg HV
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;	// ePWM1B active-- first leg HV
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // ePWM2A active-- second leg HV
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // ePWM2B active-- second leg HV
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // ePWM3A active-- first leg LV
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; // ePWM3B active-- first leg LV
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; // ePWM4A active-- second leg LV
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1; // ePWM4B active-- second leg LV
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1; // ePWM5A active-- general PWM for ADC....


	GpioCtrlRegs.GPAMUX2.all = 0;		// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPBMUX1.all = 0;		// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;		// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPCMUX1.all = 0;		// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;		// GPIO87 ... GPIO80 = General Purpose I/O
	 
	GpioCtrlRegs.GPADIR.all = 0;        //inputs
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;	//output


	GpioCtrlRegs.GPBDIR.all = 0;		// GPIO63-32 as inputs
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;	// peripheral explorer: LED LD3 at GPIO34
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1; // peripheral explorer: LED LD4 at GPIO49
	GpioCtrlRegs.GPCDIR.all = 0;		// GPIO87-64 as inputs
	EDIS;
}  

void Setup_ePWM(void)
{
    EALLOW;
    //************************************prvi H most
    //************************************ f= 70kHz
    //PWM1 (prvi leg HV)
	EPwm1Regs.TBCTL.bit.CLKDIV =  0;	// CLKDIV = 1
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;	// HSPCLKDIV = 1
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;	// up - down mode

	EPwm1Regs.AQCTLA.all = 0x0060;		// set ePWM1A on CMPA up
										// clear ePWM1A on CMPA down

	EPwm1Regs.TBPRD = 1070;			// 70KHz - PWM signal
	EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD / 2; //50% duty cycle

	EPwm1Regs.DBCTL.bit.IN_MODE=0;
	EPwm1Regs.DBCTL.bit.POLSEL=2;
	EPwm1Regs.DBCTL.bit.OUT_MODE=3;

	EPwm1Regs.DBRED=21;
	EPwm1Regs.DBFED=21;

	EPwm1Regs.TBCTL.bit.SYNCOSEL=1;

	//PWM2 (drugi leg HV)
	EPwm2Regs.TBCTL.bit.CLKDIV =  0;    // CLKDIV = 1
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;  // HSPCLKDIV = 1
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;    // up - down mode

	EPwm2Regs.AQCTLA.all = 0x0060;      // set ePWM2A on CMPA up
	                                    // clear ePWM2A on CMPA down
	EPwm2Regs.TBPRD = 1070;            // 70KHz - PWM signal
	EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD / 2; //50% duty cycle

	EPwm2Regs.DBCTL.bit.IN_MODE=0;
	EPwm2Regs.DBCTL.bit.POLSEL=2;
	EPwm2Regs.DBCTL.bit.OUT_MODE=3;

	EPwm2Regs.DBRED=21;
	EPwm2Regs.DBFED=21;

	EPwm2Regs.TBCTL.bit.PHSEN=1;
	EPwm2Regs.TBPHS.half.TBPHS= 0;

	//************************************drugi H most
	//************************************f=80kHz
	//PWM3 (prvi leg LV)
	EPwm3Regs.TBCTL.bit.CLKDIV =  0;    // CLKDIV = 1
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;  // HSPCLKDIV = 1
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;    // up - down mode

    EPwm3Regs.AQCTLA.all = 0x0060;      // set ePWM1A on CMPA up
	                                        // clear ePWM1A on CMPA down

    EPwm3Regs.TBPRD = 938;            // 1KHz - PWM signal
    EPwm3Regs.CMPA.half.CMPA = EPwm3Regs.TBPRD / 2; //50% duty cycle

    EPwm3Regs.DBCTL.bit.IN_MODE=0;
    EPwm3Regs.DBCTL.bit.POLSEL=2;
    EPwm3Regs.DBCTL.bit.OUT_MODE=3;

    EPwm3Regs.DBRED=21;
    EPwm3Regs.DBFED=21;

    EPwm3Regs.TBCTL.bit.SYNCOSEL=1;

    // PWM4 (drugi leg LV)
    EPwm4Regs.TBCTL.bit.CLKDIV =  0;    // CLKDIV = 1
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;  // HSPCLKDIV = 1
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;    // up - down mode

    EPwm4Regs.AQCTLA.all = 0x0060;      // set ePWM2A on CMPA up
	                                        // clear ePWM2A on CMPA down
    EPwm4Regs.TBPRD = 938;            // 80KHz - PWM signal
    EPwm4Regs.CMPA.half.CMPA = EPwm4Regs.TBPRD / 2; //50% duty cycle

    EPwm4Regs.DBCTL.bit.IN_MODE=0;
    EPwm4Regs.DBCTL.bit.POLSEL=2;
    EPwm4Regs.DBCTL.bit.OUT_MODE=3;

    EPwm4Regs.DBRED=21;
    EPwm4Regs.DBFED=21;

    EPwm4Regs.TBCTL.bit.PHSEN=1;
    EPwm4Regs.TBPHS.half.TBPHS= 0;


    //************************************ADC PWM
    //************************************f=80kHz
    EPwm5Regs.TBCTL.bit.CLKDIV =  0;    // CLKDIV = 1
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;  // HSPCLKDIV = 1
    EPwm5Regs.TBCTL.bit.CTRMODE = 2;    // up - down mode

    EPwm5Regs.CMPA.all = 0x501000;
    EPwm5Regs.TBPRD = 938;            // 80KHz - PWM signal
    EPwm5Regs.ETSEL.bit.SOCASEL = 2;            // ADCSOCA on TBCTR=TBPRD

    EPwm5Regs.TBPRD = 74;                       // Setup period (one off so DMA transfer will be obvious)
    EPwm5Regs.CMPA.all = 0x501000;
    EPwm5Regs.ETSEL.bit.SOCASEL = 2;            // ADCSOCA on TBCTR=TBPRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;             // Generate SOCA on 1st event
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA generation
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;          // /1 clock mode
    EDIS;
} 

void ConfigAdc(void)
{
    AdcRegs.ADCTRL1.all = 0;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 7;     // 7 = 8 x ADCCLK
    AdcRegs.ADCTRL1.bit.SEQ_CASC =1;    // 1=cascaded sequencer
    AdcRegs.ADCTRL1.bit.CPS = 0;        // divide by 1
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;
    AdcRegs.ADCTRL2.all = 0;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // 1=enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 =1;  // 1=SEQ1 start from ePWM_SOCA trigger
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;   // 0= interrupt after every end of sequence

    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;   // ADC clock: FCLK = HSPCLK / 2 * ADCCLKPS
                                        // HSPCLK = 75MHz (see DSP2833x_SysCtrl.c)
                                        // FCLK = 12.5 MHz

    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 3; //4 konverzije

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;         // ADCINA0
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;         // ADCINA1
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 2;         // ADCINA2
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 3;         // ADCINA3
}

interrupt void  adc_isr(void)
{
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;     // Clear INT SEQ1 bit

    phaseShiftADC = &AdcMirror.ADCRESULT0;
    dcBusVoltADC = &AdcMirror.ADCRESULT1;
    batteryVoltADC = &AdcMirror.ADCRESULT2;
    batteryCurrADC = &AdcMirror.ADCRESULT3;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
}

//===========================================================================
// End of SourceCode.
//===========================================================================
