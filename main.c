/*#########################################################
                         main.c

    Este cÃ³digo utiliza a tÃ©cnica SPWM para acionar as chaves de um inversor trifÃ¡sico e a transformada de Clarke-Park para se obter a moduladora.

    Universidade Federal do Ceará - Engenharia Elétrica
    PIBIC 2022/2023: Desenvolvimento de uma bancada para acionamento de motores de indução trifásicos
    Bolsista: Auro Gabriel Carvalho de Aramides
    Versão: 1.0
    Data: 08/2022
    Autor: Auro Gabriel Carvalho de Aramides
//#########################################################
//#########################################################
                    Descrição dos pinos
    GPIO 6   - Saída ePWM 4A
    GPIO 7   - Saída ePWM 4B
    GPIO 8   - Saída ePWM 5A
    GPIO 9   - Saída ePWM 5B
    GPIO 10  - Saída ePWM 6A
    GPIO 11  - Saída ePWM 6B
    GPIO 15  - Sinal de alarme, responsável por gerar uma interrupção externa quando necessário (XINT2)
    GPIO 104 - Pino que aciona o relé que energiza a contactora que, por sua vez, energiza o circuito de potência.
    GPIO 105  - Pino que aciona o relé que energiza a fonte chaveada
    GPIO 14 - 3V3 (pino usado para o opto acplador de proteção)
    GPIO96 - eQEP1A
    GPIO97 - eQEP1B
    GPIO99 - eQEP1I


//#########################################################*/

#include "F28x_Project.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define PORTADORA_FREQ 6000
#define MODULADORA_FREQ 60
#define NOS 200 //Number of Samples
#define Mi 0.8   // �ndice de modula��o
#define pi 3.14159265358979323846
#define DPI 6.28318530717958647692
#define sqrt2 1.4142135623730951

//Variáveis gerais.

float32 wa=0;   //Velocidade mecânica do rotor em rad/s.

float w = 0; // Velocidade angular elétrica medida em rpm

float thetak=0;
float32 thetak_ant;

unsigned int Posicao_ADC = 0; //Leitura da velocidade no módulo eqep.
float Rotor_Posicao = 0; //Posição angular do rotor.

Uint16 AlarmCount=0;                        // Alarm Counter

Uint64 index=0;
Uint64 cont=0;

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


//################// Parametros do Motor (Transformada)#############################


float32 Pn  = 7.5e3;     // W, nominal power
float32 Vn  = 380.0;       // V,  rms phase-to-phase, rated voltage
float32 fn  = 60.0;        // Hz, rated frequency

float32 rend = 0.91;
float32 fp   = 0.82;
float32 Sn;//   = Pn/(rend*fp);

float32 Rs  = 1.653;                // Resistência no estator
float32 Lls = 8.814531265e-3;                 // Indutância de dispersão no estator
float32 Rr  = 1.013;                // Resistência do rotor pelo lado do estator
float32 Llr = 9.26547027e-3;                 // Indutância de dispersão do rotor pelo lado do estator
float32 Lm  = 0.242154245;                // Indutância de magnetização
float32 Lr;// = Llr+Lm;                // Indutância própria no rotor
float32 Ls;// = Lls+Lm;                // Indutância própria no estator
float32 p = 2.0;                      // pares de polos -> 4polos


float32 T; // =Lr/Rr

//########################## Transformada Inversa de Clarke-Park #################################################


float32 Va,Vb,Vc;
float32 Vdref=0.4,Vqref=0.5,n_ref=10; // n_ref -> Velocidade de referência do campo magnético (Valores de referência da simulação)
float32 theta=0, theta_ant=0; // ângulos do campo magnético para o integrador discreto



 // Leitura das medidas dos sensores


void Setup_GPIO(void);
void Setup_INTERRUPT(void);
void Setup_ePWM(void);
void Setup_ADC(void);

void Liga_Bancada(void);
void Desliga_Bancada(void);
void Stop_SPWM(void);

void Set_ePWM_Frequency(uint32_t freq_pwm);

//Função do Encoder
void Setup_eQEP(void);




__interrupt void alarm_handler_isr(void);     // Alarm Handler interrupt service routine function prototype.
__interrupt void adca_isr(void);




void main(void){
//##########__INICIALIZA��O__#######################################################################

    // ATEN��O: A fun��o InitPeripheralClocks() (presente dentro da fun��o InitSysCtrl()) foi modificada
    // para habilitar apenas o clock dos perif�ricos que est�o sendo usados nesse c�digo. A saber:
    //  TBCLKSYNC; ePWM's. Seja     consciente, economize energia :)

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
    Setup_eQEP();


                                                          // Configura��o das interrup��es

    EALLOW;                                             // Endere�o das rotinas de interrup��es
        PieVectTable.ADCA1_INT = &adca_isr;
        PieVectTable.XINT2_INT  =  &alarm_handler_isr;
    EDIS;



    EALLOW;

      //DAC-B
      DacbRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
      DacbRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
      DacbRegs.DACVALS.all = 0x0800;              // Set mid-range
      DacbRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

      //DAC -A
      DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
      DacaRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
      DacaRegs.DACVALS.all = 0x0800;              // Set mid-range
      DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

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


    EINT;                                          // Enable Global interrupt INTM
    ERTM;                                          // Enable Global realtime interrupt DBGM , UTILIZADO PARA ALTERAR O VALOR DOS REGISTRADORES EM TEMPO REAL.

 //################################# Varáveis da Transformação ########################

    Lr= Llr+Lm;
    Ls= Lls+Lm;
    T = __divf32(Lr,Rr);

    Vqref=0; // zero para poder crescer em rampa

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

         if(cont==36000){
             Vqref=0.52;
         }

        // Rotina ADC com 12 KHZ (frequ�ncia de amostragem do sinal senoidal da moduladora).


        //Tratamento da variável theta-> Saturador + Integrador

        if( theta < 0 || theta>DPI){
         theta_ant = 0;
        }
       else{
            theta_ant = theta;
        }


        //Posicao_ADC  = EQep1Regs.QPOSCNT;

        if(EQep1Regs.QEPSTS.bit.COEF == 0 && EQep1Regs.QEPSTS.bit.CDEF == 0){
            wa = (float) __divf32(__divf32(8.0*60.0,20),__divf32(EQep1Regs.QCPRD*64.0,200.0e6));

            thetak = wa *DPI* __divf32 ( __divf32( ((float) EQep1Regs.QCPRD) ,200.0e6),60) + thetak_ant;
        }
        else{
            EQep1Regs.QEPSTS.bit.COEF = 0;
            EQep1Regs.QEPSTS.bit.CDEF = 0;

        }

        thetak_ant = thetak;

        if(thetak>=DPI ||  thetak_ant>=DPI )  thetak_ant=0;

        //POSIÇAO ANGULAR.
         Rotor_Posicao = __divf32( ((float)Posicao_ADC)*DPI , 20.0); // DoisPi/20 = Ângulo por pulso de quadratura    -->Auro: O produto desses dois dá o ângulo que já rodou
                                                          //  Posicao_ADC = Quantidade de pulsos                        # "Rotor_Posicao" é em radianos

         //Partida em rampa
          if(SPWM_State==1){
              if(Vqref<=0.42){
                  Vqref=Vqref+0.000001;
              }
              else{
                  SPWM_State++;
              }
          }


//######################### Estimativa Indireta de Fluxo ##################################################################################################

         theta= __divf32( (__divf32(Vqref,Vdref*T) + n_ref*0.1047197551*1),200) + theta_ant; // theta é o ângulo do campo magnético. 1/12000 -> tempo de amostragem

//#################################Transformada Inversa Clarke-Park##########################################################################################

         Va= (float32)  (TB_Prd/2)*(1+(0.8164965819*(Vdref*__cos(theta)+ Vqref*__sin(theta))));
         Vb= (float32)  (TB_Prd/2)*(1+(0.8164965819*(Vdref*__cos(theta+2.0943951024)+ Vqref*__sin(theta+2.0943951024))));
         Vc= (float32)  (TB_Prd/2)*(1+(0.8164965819*(Vdref*__cos(theta+4.1887902048)+ Vqref*__sin(theta+4.1887902048))));

         EPwm4Regs.CMPA.bit.CMPA = Va;
         EPwm5Regs.CMPA.bit.CMPA = Vb;
         EPwm6Regs.CMPA.bit.CMPA = Vc;



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



        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;    // Limpa as FLAGS provinientes do Trigger.
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  //

}
//##########__ALARME ISR___#######################################################################
interrupt void alarm_handler_isr(void){

    //Alarme soou, codigo ficar� preso at� que o bot�o de "RE-DEBUG" seja pressinado no DSP.

    Stop_SPWM();                                      //Para o PWM e seta as sa�das (Abre as chaves).

    AlarmCount++;                                      // Contador do alarme

    while(1){

        GpioDataRegs.GPATOGGLE.bit.GPIO31 =1;       //LEDS do DSP para sinaliza��o do Alarme
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 =1;
        DELAY_US(400000);
    }

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Reabilita interrup��es provenientes do alarme
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

        // PWM sendo utilizado como TIMER , gerando SOC a 12 Khz. 4165
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
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 5; // ADCINA3 - PINO 26 (J3).
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
        AdcbRegs.ADCSOC1CTL.bit.CHSEL =5 ; // ADCINB3 - PINO 25 (J3).
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
        AdccRegs.ADCSOC2CTL.bit.CHSEL =5 ; // ADCINC3 - PINO 24 (J3).
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


void Setup_eQEP(){

    EALLOW;


        //EQEP.
        InitEQep1Gpio();

        GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 0xF);
        GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 0xF);
        GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);


    //Configura o EQep1:

       EQep1Regs.QUPRD = 0x00000C95;  //0000 0000 1100 1001 0101;            // Unit Timer for 150Hz at 200 MHz
                                                // SYSCLKOUT


       EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode





//QEPCTL -
       EQep1Regs.QEPCTL.bit.FREE_SOFT = 0x2;  //QEP Control =QEPCTL
       EQep1Regs.QEPCTL.bit.PCRM = 0x1;       // PCRM=01 -> Position counter reset on the maximum position

       EQep1Regs.QEPCTL.bit.UTE = 1;        // Unit Timeout Enable
       EQep1Regs.QEPCTL.bit.QCLM = 1;       // Latch on unit time out


       EQep1Regs.QCAPCTL.bit.CEN = 1;       // QEP Capture Enable
       EQep1Regs.QEPCTL.bit.QPEN = 1;       //QEP enable

//QPOSCTL


       EQep1Regs.QPOSMAX = 0x14;            //0x14 = 20 contagens de quadratura
       EQep1Regs.QDECCTL.bit.SWAP = 0;      //troca o sentido da contagem

       EQep1Regs.QCAPCTL.bit.UPPS = 3;      // 1/8 for unit position
       EQep1Regs.QCAPCTL.bit.CCPS = 6;      // 1/64 for CAP clock


       //EQep1Regs.QEINT.bit.UTO = 1;       // 400 Hz interrupt for speed estimation
       EDIS;

    //################## Auro: Mudei os valores para o que a gente usa ############
    //
    // EXTRACTED FROM FILE:    Example_posspeed.c
    //
    // TITLE:   Pos/speed measurement using EQEP peripheral
    //
    // DESCRIPTION:
    //
    // This file includes the EQEP initialization and position and speed
    // calculation functions called by Eqep_posspeed.c.  The position and speed
    // calculation steps performed by POSSPEED_Calc() at  SYSCLKOUT =  200 MHz are
    // described below:

    //#############################################################################
    //  ###### Encoder de 5 pulsos por volta -> 20 de quadratura #################
    //#############################################################################

    // 1. This program calculates: **theta_mech**
    //
    //    theta_mech = QPOSCNT/mech_Scaler = QPOSCNT/20, where 20 is the number
    //                 of counts in 1 revolution.


    //                (20/4 = 5 line/rev. quadrature encoder)
    //

    // 2. This program calculates: **theta_elec**
    //
    //    theta_elec = (# pole pairs) * theta_mech = 2*QPOSCNT/20
    //


    // 3. This program calculates: **SpeedRpm_fr**
    //
    //    SpeedRpm_fr = [(x2-x1)/20]/T   - Equation 1

    //    Note (x2-x1) = difference in number of QPOSCNT counts. Dividing (x2-x1)
    //    by 20 gives position relative to Index in one revolution.


    // If base RPM  = 1800 rpm:   1800 rpm = [(x2-x1)/20]/10ms   - Equation 2

    //                                     = [(x2-x1)/20]/(.01s*1 min/60 sec)
    //                                     = [(x2-x1)/20]/(1/6000) min  - Equation 2

    //                         max (x2-x1) = 20 counts, or 1 revolution in 10 ms


    // If both sides of Equation 2 are divided by 1800 rpm, then:
    //                   1 = [(x2-x1)/20] rev./[(1/6000) min * 1800rpm]

    //                   Because (x2-x1) must be <20 (max) for QPOSCNT increment,
    //                   (x2-x1)/20 < 1 for CW rotation

    //                   And because (x2-x1) must be >-20 for QPOSCNT decrement,
    //                   (x2-x1)/1800>-1  for CCW rotation (CCW= Counter Clockwise)

    //                   speed_fr = [(x2-x1)/20]/[(1/6000) min * 1800rpm]
    //                            = (x2-x1)/2     - Equation 3


    // To convert speed_fr to RPM, multiply Equation 3 by 1800 rpm
    //                   SpeedRpm_fr = 1800rpm *(x2-x1)/2  - Final Equation



    // 2. **min rpm ** = selected at 10 rpm based on [CCPS] prescaler options
    //    available (128 is greatest)


    // 3. **SpeedRpm_pr**

    //    SpeedRpm_pr = X/(t2-t1)  - Equation 4

    //    where X = QCAPCTL [UPPS]/20 rev. (position relative to Index in
    //                                        1 revolution)
    // If  max/base speed = 1800 rpm:
    //               1800 = (8/20)/[(t2-t1)/(200MHz/64)]

    //          where 8 = QCAPCTL [UPPS] (Unit timeout - once every 8 edges)
    //          where 64 = QCAPCTL [CCPS]

    //            8/20 = position in 1 revolution (position as a fraction
    //                                                of 1 revolution)

    //   t2-t1/(200MHz/64), t2-t1= # of QCAPCLK cycles, and
    //      QCAPCLK cycle = 1/(200MHz/64)
//////                    = QCPRDLAT   #################################################


    // So:       1800 rpm = [UPPS(200MHz/CCPS)*60s/min]/[20(t2-t1)]
    //

    //           1800 rpm = [8*(200MHz/64)*60s/min]/[20(t2-t1)]
    //           t2-t1 = [8*(200MHz/64)*60s/min]/(20*1800rpm)  - Equation 5
    //                    ~= 416.66 CAPCLK cycles = maximum (t2-t1) = SpeedScaler
       //                   52.0825
    //
    // Divide both sides by (t2-t1), and:
    //            1 = 32/(t2-t1) = [8(200MHz/64)*60 s/min]/(20*1800rpm)]/(t2-t1)

    //       Because (t2-t1) must be < 416.66 for QPOSCNT increment:
    //               416.66/(t2-t1) < 1 for CW rotation
    //       And because (t2-t1) must be >-416.66 for QPOSCNT decrement:
    //                416.66/(t2-t1)> -1 for CCW rotation
    //
    //       eed_pr = 416.66/(t2-t1)
    //             or [8(200MHz/64)*60 s/min]/(20*1800rpm)]/(t2-t1) - Equation 6
    //
    // To convert speed_pr to RPM:
    // Multiply Equation 6 by 1800rpm:
    // SpeedRpm_fr  = 1800rpm * [8*(200MHz/64)*60 s/min]/[20*1800rpm*(t2-t1)]
    //              = [8(200MHz/64)*60 s/min]/[20*(t2-t1)]
    //              or [(8/20)rev * 60 s/min]/[(t2-t1)(QCPRDLAT)]-Final Equation
    //
    // More detailed calculation results can be found in the Example_freqcal.xls
    // spreadsheet included in the example folder.
    //
    // Olhar o exemplo da texas citado no "title" para entender mais coisas.
    //###########################################################################


}

