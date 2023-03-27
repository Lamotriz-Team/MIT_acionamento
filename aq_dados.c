/*#########################################################
                         main.c
    Este c�digo utiliza a t�cnica SPWM para acionar as chaves de um inversor trif�sico.

    Universidade Federal do Cear� - Engenharia El�trica
    PIBIC 2020/2021: Desenvolvimento de uma bancada para acionamento de motores de indu��o trif�sicos
    Bolsista: Vinicius Vasconcelos.
    Vers�o: 1
    Data: 11/2020
    Autor: Vinicius Vasconcelos do Rego
//#########################################################

//#########################################################
                    Descri��o dos pinos
    GPIO 6   - Sa�da ePWM 4A
    GPIO 7   - Sa�da ePWM 4B
    GPIO 8   - Sa�da ePWM 5A
    GPIO 9   - Sa�da ePWM 5B
    GPIO 10  - Sa�da ePWM 6A
    GPIO 11  - Sa�da ePWM 6B
    GPIO 15  - Sinal de alarme, respons�vel por gerar uma interrup��o externa quando necess�rio (XINT2)
    GPIO 104 - Pino que aciona o rel� que energiza a contactora que, por sua vez, energiza o circuito de pot�ncia.
    GPIO 105  - Pino que aciona o rel� que energiza a fonte chaveada
    GPIO 14 - 3V3 (pino usado para o opto acplador de prote��o)
//#########################################################*/

#include "F28x_Project.h"
#include "math.h"

#define PORTADORA_FREQ 6000
#define MODULADORA_FREQ 60
#define NOS 200 //Number of Samples
#define Mi 0.8   // �ndice de modula��o
#define pi 3.14159265358979323846




Uint16 AlarmCount=0;                        // Alarm Counter
Uint16 index= 0;
Uint16 w1,w2,w3;
Uint16 TB_Prd;
Uint16 TB_Prescale;
Uint16 Comando_L_D;
Uint16 aux = 0;
Uint16 SPWM_State = 0;


float32 Converted_Voltage_P1=0;
float32 Converted_Voltage_P2=0;
float32 Converted_Voltage_P3=0;

float32 current_phase_1=0;
float32 current_phase_2=0;
float32 current_phase_3=0;


void Setup_GPIO(void);
void Setup_INTERRUPT(void);
void Setup_ePWM(void);
void Setup_ADC(void);

void Liga_Bancada(void);
void Desliga_Bancada(void);
void Stop_SPWM(void);

void Set_ePWM_Frequency(uint32_t freq_pwm);


//__interrupt void alarm_handler_isr(void);     // Alarm Handler interrupt service routine function prototype.
__interrupt void adca_isr(void);


void main(void){
//##########__INICIALIZA��O__#######################################################################

    // ATEN��O: A fun��o InitPeripheralClocks() (presente dentro da fun��o InitSysCtrl()) foi modificada
    // para habilitar apenas o clock dos perif�ricos que est�o sendo usados nesse c�digo. A saber:
    //  TBCLKSYNC; ePWM's. Seja consciente, economize energia :)




     InitSysCtrl();                          // PLL, WatchDog, enable Peripheral Clocks
       InitGpio();                             // Inicializa��o do GPIO
       DINT;                                   // Disable CPU interrupts
       InitPieCtrl();                          // Initialize PIE control registers to their default state.
       IER = 0x0000;                           // Disable CPU interrupts and clear all CPU interrupt flags:
       IFR = 0x0000;
       InitPieVectTable();                     // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

//##########__CONFIGURA��ES INICIAIS__#######################################################################
    Set_ePWM_Frequency(PORTADORA_FREQ);                // Set the ePWM frequence in Hz. Min 193 hz. A frequ�ncia da portadora deve ser um m�ltiplo da moduladora.
                                                       //Para garantir um n�mero inteiro de pulsos por semiciclo. (RASHID).

    Setup_GPIO();                                      // Configura��o dos GPIOs
    Setup_ePWM();                                      // Abre todas as chaves
    Setup_ADC();
                                                       // Configura��o das interrup��es

    EALLOW;                                             // Endere�o das rotinas de interrup��es
        PieVectTable.ADCA1_INT = &adca_isr;
       // PieVectTable.XINT2_INT  =  &alarm_handler_isr;
    EDIS;


    EALLOW;
   //##########__CONFIGURA��O XINT2 (ALARME)__#######################################################################
        PieCtrlRegs.PIECTRL.bit.ENPIE  = 1;        // Enable the PIE block
        PieCtrlRegs.PIEIER1.bit.INTx5 = 1;        // Enable the PIE Group 1 INT 5. (XINT2)

        XintRegs.XINT2CR.bit.POLARITY = 0;         // Interrupt will occur on the falling edge (high-to-low transition)
        XintRegs.XINT2CR.bit.ENABLE   = 1;         // Enable XINT 2.
    EDIS;

    //##########__CONFIGURA��O ADC_INT__#######################################################################
      EALLOW;

       PieCtrlRegs.PIEIER1.bit.INTx1   = 1 ;         // Habilita o PIE para interrup��o do ADC.
       AdcaRegs.ADCINTSEL1N2.bit.INT1E =1;
       EDIS;

       // IER |= M_INT1;

    EINT;                                          // Enable Global interrupt INTM
    ERTM;                                          // Enable Global realtime interrupt DBGM , UTILIZADO PARA ALTERAR O VALOR DOS REGISTRADORES EM TEMPO REAL.



//##########__CODIGO__#######################################################################
    while(1)
    {
        Comando_L_D != 0 ? Liga_Bancada():Desliga_Bancada(); // Uiliza o debug em tempo real para ligar ou desligar a bancada
                                                             // Alterando o valor da vari�vel Comando_L_D na janela de express�es
                                                             // do code composer studio.
    }

}
//##########__ADCA ISR___#######################################################################
__interrupt void adca_isr(){

        // Rotina ADC com 12 KHZ (frequ�ncia de amostragem do sinal senoidal da moduladora).

        if(index  == 200 )
          index = 0;     //Limpa o Buffer.
        else
          index++;
        // Transoforma o resultado decimal equivalente ao bin�rio da convers�o de cada fase em uma tens�o de -1,15 a 1,15 V
        Converted_Voltage_P1 = __divf32(3.0*AdcaResultRegs.ADCRESULT0,4096.0)-1.65;
        Converted_Voltage_P2 = __divf32(3.0*AdcbResultRegs.ADCRESULT1,4096.0)-1.65;
        Converted_Voltage_P3 = __divf32(3.0*AdccResultRegs.ADCRESULT2,4096.0)-1.65;

        /// * Adc Voltage Range = 0 : 3.0 V
        // * Sensor Voltage Range = 0.5 : 2.8 V
        // * Converted_Voltage_Px Voltage Range = -1.15 : 1.15 V


        //Calcula a corrente atrav�s da tens�o pela sensibilidade do sensor : 18.4 mV / A
         current_phase_1 =- __divf32(Converted_Voltage_P1,0.0184);
         current_phase_2 = -__divf32(Converted_Voltage_P2 ,0.0184);
         current_phase_3 =- __divf32(Converted_Voltage_P3,0.0184);


        //Gera Moduladora Senoidal .
        w1 = (Uint16) (TB_Prd/2)*(1+Mi*__sin(__divf32(2*pi,NOS) * (float) index   ));
        w2 = (Uint16) (TB_Prd/2)*(1+Mi*__sin(__divf32(2*pi,NOS) * (float) index - 2*pi/3));
        w3 = (Uint16) (TB_Prd/2)*(1+Mi*__sin(__divf32(2*pi,NOS) * (float) index - 4*pi/3));

        EPwm4Regs.CMPA.bit.CMPA = w1;
        EPwm5Regs.CMPA.bit.CMPA = w2;
        EPwm6Regs.CMPA.bit.CMPA = w3;


        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;    // Limpa as FLAGS provinientes do Trigger.
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  //

}
//##########__ALARME ISR___#######################################################################
interrupt void alarm_handler_isr(void){
/*
 *

    //Alarme soou, codigo ficar� preso at� que o bot�o de "RE-DEBUG" seja pressinado no DSP.

    Stop_SPWM();                                      //Para o PWM e seta as sa�das (Abre as chaves).

    AlarmCount++;                                      // Contador do alarme

    while(1){

        GpioDataRegs.GPATOGGLE.bit.GPIO31 =1;       //LEDS do DSP para sinaliza��o do Alarme
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 =1;
        DELAY_US(400000);
    }


*/
   //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Reabilita interrup��es provenientes do alarme
}


void Setup_GPIO(void){
    EALLOW;

    //##############################__FONTE 3V3__##############################

        GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;        // GPIO = GPIO (default)
        GpioCtrlRegs.GPAMUX1.bit.GPIO14  = 0;        // GPIO = GPIO (default)
        GpioCtrlRegs.GPADIR.bit.GPIO14   = 1;        // OUTPUT
        GpioDataRegs.GPASET.bit.GPIO14    = 1;        // HIGH

    //##############################_GND CABO FLAT_##############################

        //GPIO26
        GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;        // GPIO = GPIO (default)
        GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;        // GPIO = GPIO (default)
        GpioCtrlRegs.GPADIR.bit.GPIO26   = 1;        // OUTPUT
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;        // LOW

        //GPIO66
        GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0;        // GPIO = GPIO (default)
        GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // GPIO = GPIO (default)
        GpioCtrlRegs.GPCDIR.bit.GPIO66   = 1;        // OUTPUT
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;        // LOW

        //GPIO130
         GpioCtrlRegs.GPEGMUX1.bit.GPIO130 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPEDIR.bit.GPIO130   = 1;        // OUTPUT
         GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;        // LOW

         //GPIO131
         GpioCtrlRegs.GPEGMUX1.bit.GPIO131 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPEDIR.bit.GPIO131   = 1;        // OUTPUT
         GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;        // LOW

         //GPIO63
         GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPBDIR.bit.GPIO63   = 1;        // OUTPUT
         GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;        // LOW

         //GPIO64
         GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPCDIR.bit.GPIO64   = 1;        // OUTPUT
         GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;        // LOW

         //GPIO27
         GpioCtrlRegs.GPAGMUX2.bit.GPIO27= 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;        // GPIO = GPIO (default)
         GpioCtrlRegs.GPADIR.bit.GPIO27   = 1;        // OUTPUT
         GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

         //GPIO25
          GpioCtrlRegs.GPAGMUX2.bit.GPIO25= 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPADIR.bit.GPIO25   = 1;        // OUTPUT
          GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

     //##############################################################


          //GPIO95
          GpioCtrlRegs.GPCGMUX2.bit.GPIO95 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPCMUX2.bit.GPIO95 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPCDIR.bit.GPIO95   = 1;        // OUTPUT
          GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;        // LOW

          //GPIO139
          GpioCtrlRegs.GPEGMUX1.bit.GPIO139= 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPEMUX1.bit.GPIO139 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPEDIR.bit.GPIO139  = 1;        // OUTPUT
          GpioDataRegs.GPECLEAR.bit.GPIO139 = 1;

          //GPIO56
          GpioCtrlRegs.GPBGMUX2.bit.GPIO56= 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPBDIR.bit.GPIO56   = 1;        // OUTPUT
          GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1;

          //GPIO97
          GpioCtrlRegs.GPDGMUX1.bit.GPIO97 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPDMUX1.bit.GPIO97 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPDDIR.bit.GPIO97   = 1;        // OUTPUT
          GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;        // LOW

          //GPIO94
          GpioCtrlRegs.GPCGMUX2.bit.GPIO94 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPCDIR.bit.GPIO94   = 1;        // OUTPUT
          GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;        // LOW

          //GPIO65
          GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPCDIR.bit.GPIO65   = 1;        // OUTPUT
          GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;        // LOW

          //GPIO52
          GpioCtrlRegs.GPBGMUX2.bit.GPIO52= 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;        // GPIO = GPIO (default)
          GpioCtrlRegs.GPBDIR.bit.GPIO52   = 1;        // OUTPUT
          GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    //##############################__ePWM 4, 5 e 6__###########################


           GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
           GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
           GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;

           GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
           GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
           GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;


          GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;
          GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
          GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;

          GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;
          GpioCtrlRegs.GPAMUX1.bit.GPIO9= 1;
          GpioCtrlRegs.GPAPUD.bit.GPIO9= 1;


          GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;
          GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
          GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;

          GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;
          GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
          GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;



    //##############################__LEDS do DSP : ALARME__##############################

        GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
        GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;
        GpioCtrlRegs.GPADIR.bit.GPIO31  = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO31 =1;

        GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
        GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO34  = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;

    //##############################__ALARME__##############################

       GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;        // GPIO = GPIO (default)
       GpioCtrlRegs.GPAMUX1.bit.GPIO15  = 0;        // GPIO = GPIO (default)
       GpioCtrlRegs.GPADIR.bit.GPIO15   = 0;        // INPUT
     //  GpioCtrlRegs.GPAQSEL1.bit.GPIO15   = 2;          // XINT2 Qual using 6 samples
      // GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0xFF;      // Each sampling window
                                                      // is 510*SYSCLKOUT
      // InputXbarRegs.INPUT5SELECT = 15;               // GPIO 15 is XINT2


    //#############################__Acionamento e desligamento da bancada__###########

       //Base do transistor que aciona o rel� da fonte de controle.
       GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 0;
       GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 0;
       GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;

       //Base do transistor que aciona o rel� da fonte de pot�ncia.
       GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 0;
       GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 0;
       GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;



    EDIS;


}




void Setup_ePWM(void){

    EALLOW;                    //###### LEMBRAR DE AJUSTAR O FED E RED PARA OS PREESCALES DO TBCLOCK.

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;           // DISABLE TBCLK for ePWM configuration.

//##########__EPWM4__###################################################################

    EPwm4Regs.TBPRD = TB_Prd;                       // Set timer period TBPR = (EPWMCLK ) / ( 2 x 2 x freq_pwm) for up-down count mode.
    EPwm4Regs.CMPA.bit.CMPA = 0;                    // Clear CMPA
    EPwm4Regs.TBPHS.bit.TBPHS = 0;                  // Regsitrador de fase zerado.
    EPwm4Regs.TBCTL.bit.SYNCOSEL =TB_CTR_ZERO  ;    // Sincroniza as fases em TBPRD = 0.
    EPwm4Regs.TBCTR = 0x0000;                       // Limpa o contador.
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Configura a portadora para o modo sim�trico(up-down).
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Desabilita o carregamento das fases.

    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        //TBCLK = EPWMCLK/(HSPCLKDIV*CLKDIV)
    if(TB_Prescale == 1)         EPwm4Regs.TBCTL.bit.CLKDIV = 0;      // EPWMCLK/1.
    else if (TB_Prescale == 128) EPwm4Regs.TBCTL.bit.CLKDIV = 7;      // EPWMCLK/128.

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       //Habilita o shadow como um buffer duplo.
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; /* Carrega os registradores nos eventos TBCTR = ZERO  e TBCTR = PRD.
                                                         Necess�rio pois a senoide apresenta valores diferentes em CAU e CAD.
                                                      */

    // Configura��o do Action Qualifier para gera��o do SPWM.
    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Low complementary to use EPWM4A complementary to EMPW4B.
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // Habilita o m�dulo de Dead-Band.
    EPwm4Regs.DBFED.bit.DBFED = 400;                // FED = 350 TBCLKs (4.0 uS)
    EPwm4Regs.DBRED.bit.DBRED = 400;                // RED = 350 TBCLKs (4.0 uS)


    //##########__EPWM5__##########################################################################

    EPwm5Regs.TBPRD = TB_Prd;
    EPwm5Regs.CMPA.bit.CMPA = 0;
    EPwm5Regs.TBPHS.bit.TBPHS = 0;
    EPwm5Regs.TBCTL.bit.SYNCOSEL =TB_CTR_ZERO ;
    EPwm5Regs.TBCTR = 0x0000;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    if (TB_Prescale == 1)         EPwm5Regs.TBCTL.bit.CLKDIV = 0;      // EPWMCLK/1
    else if (TB_Prescale == 128) EPwm5Regs.TBCTL.bit.CLKDIV = 7;       // EPWMCLK/128

    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;


    EPwm5Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBFED.bit.DBFED = 400;
    EPwm5Regs.DBRED.bit.DBRED =400;

//##########__EPWM6__##########################################################################

    EPwm6Regs.TBPRD = TB_Prd;
    EPwm6Regs.CMPA.bit.CMPA = 0;
    EPwm6Regs.TBPHS.bit.TBPHS = 0;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm6Regs.TBCTR = 0x0000;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    if(TB_Prescale == 1)        EPwm6Regs.TBCTL.bit.CLKDIV = 0;
    else if (TB_Prescale == 128)EPwm6Regs.TBCTL.bit.CLKDIV = 7;

    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm6Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBFED.bit.DBFED = 400;
    EPwm6Regs.DBRED.bit.DBRED = 400;

//##########__EPWM1__##########################################################################

        // PWM sendo utilizado como TIMER , gerando SOC a 12 Khz.
       // Frequencia de amostragem deve ser um multiplo de 60.

     EPwm1Regs.TBPRD = 4165 ;  // Sampling at 12 Khz / TBPRD = TPWM/TBCLK - 1 -> FAZER O CALCULO COM A FREQ !!
     EPwm1Regs.TBPHS.bit.TBPHS = 0;
     EPwm1Regs.TBCTR = 0X0000;
     EPwm1Regs.TBCTL.bit.CTRMODE= TB_COUNT_UP; // Configura como up count mode.

     // Condigura��es padr�es para gerar TBCLK = 50 Mhz  ( EPWMCLK = SYSCLKOUT/2 = 100 Mhz , TBCLK = EPWMCLK/(HSPCLKDIV * CLKDIV) );
     EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;
     EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;

     EPwm1Regs.ETSEL.bit.SOCAEN =1;  // Enable SOC on A group
     EPwm1Regs.ETSEL.bit.SOCASEL=ET_CTR_PRD ;  // Dispara o SOC em CTR = PRD.
     EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;     // Dispara o SOC no primeiro evento.

     CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;                // ENABLE TBCLKs

       EDIS;

}

void Setup_ADC(){

        Uint16 acqps;

        // Configura��es m�nimas , consultar datasheet pag 105.
        if( ADC_RESOLUTION_12BIT  == AdcaRegs.ADCCTL2.bit.RESOLUTION)
            acqps = 14;
        else
            acqps = 63;

        EALLOW;

      //##################################################################### ADC A ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_A =1;   // Habilita o clock do m�dulo A do ADC.
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCA,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Gera interrup��o um ciclo de clock antes do EOC.
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;   // Energiza o ADC A .
        DELAY_US(1000);  // 1ms de delay para ligar o m�dulo do ADC.

        // SOC and INTERRUPT config
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3; // ADCINA3 - PINO 26 (J3).
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; // 64 SYSCLK cycles to charge the capacitor. Recomendado no Datasheet , pag 105.



        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // EOC DISPARA o  ADCINT1;
        AdcaRegs.ADCINTSEL1N2.bit.INT1E =0;   // Desabilita interrup��es do ADC;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 =1; //Make sure the INT1 flag is cleared.

        //##################################################################### ADC B ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_B = 1;   // Habilita o clock do m�dulo B do ADC.
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6;  // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCB,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdcbRegs.ADCCTL1.bit.INTPULSEPOS=1; // Gera interrup��o um ciclo de clock antes do EOC.
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Energiza o ADC B.
        DELAY_US(1000);  // 1ms de delay para ligar o m�dulo do ADC.

        // SOC config
        AdcbRegs.ADCSOC1CTL.bit.CHSEL =3 ; // ADCINB3 - PINO 25 (J3).
        AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;


        //##################################################################### ADC C ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
        AdccRegs.ADCCTL2.bit.PRESCALE = 6;  // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCC,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdccRegs.ADCCTL1.bit.INTPULSEPOS=1; // Gera interrup��o um ciclo de clock antes do EOC.
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Energiza o ADC C.
        DELAY_US(1000);  // 1ms de delay para ligar o m�dulo do ADC.

        // SOC config
        AdccRegs.ADCSOC2CTL.bit.CHSEL =3 ; // ADCINC3 - PINO 24 (J3).
        AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps;



        EDIS;

}


void Set_ePWM_Frequency(uint32_t freq_pwm){
    if(freq_pwm < 763){                      //Minimum Value of frequency without dividing the EPWMCLK . Thus is necessary to divide it if we want less freq.
        TB_Prd =  (0x17D784)/(2*2*freq_pwm); //0x17D784 = 200 MHz / 128.
        TB_Prescale = 128;                   //Control variable to choose witch value will divide EPWMCLK.
    }
    else{                                    // Not necessary to divide EPWMCLK.
        TB_Prd =  (0xBEBC200)/(2*2*freq_pwm);//0xBEBC200 = 200 Mhz = EPWMCLk/1;
        TB_Prescale = 1;                     // Divide EPWMCLK by 1.
    }
}



void Liga_Bancada(void)
{
    if(aux == 0 )
     {
         EALLOW;

           Stop_SPWM();

           GpioDataRegs.GPBSET.bit.GPIO41 = 1;     // Liga fonte de controle.
           DELAY_US(2000000);
           GpioDataRegs.GPBSET.bit.GPIO40 = 1;    //Liga fonte de pot�ncia.


           while(1){if(SPWM_State){break;} }       // Espera at� que SPWM_State seja diferente de zero
                                                  // Para liberar o PWM. Isso � feito alterando a vari�vel
                                                  // SPWM_Satte em tempo real atrav�s da aba de exprss�es do DSP.

           Setup_GPIO();                         //Libera PWM.
           IER |= M_INT1;

           aux++;
           EDIS;
     }
}

void Desliga_Bancada(void)
{


    EALLOW;

    GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1;  // Desliga fonte de pot�ncia

    DELAY_US(2000000);

    GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1; // Desliga fonte de controle

    IER &= M_INT1;
    aux = 0;
   // SPWM_State=0;
   // Vqref=0.10;
    index=0;
   // pos_buf=0;
   // cont=0;
    EDIS;
}

void Stop_SPWM(void){

    EALLOW;

    IER &= M_INT1;                          //Desabilita PWM assocaido a interrup��o do ADC.
    GpioCtrlRegs.GPAMUX1.all = 0;           // GPIO = GPIO
    GpioCtrlRegs.GPADIR.all = 0x00000FC0;
    GpioDataRegs.GPACLEAR.all = 0x00000FC0; // LOW (GPIO 6, 7, 8, 9, 10 e 11)
                                           // Abre todas as chaves.

    EDIS;
}
