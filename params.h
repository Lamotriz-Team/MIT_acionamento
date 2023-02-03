/*
 * params.h
 *
 */

#ifndef PARAMS_H_
#define PARAMS_H_

//Parâmetros da máquina

#define Rs  35.58
#define Rr  87.44
#define Lls 0.16
#define Llr 0.16
#define Ls  1.044 //Lls +Lm
#define Lr  1.044 //Llr +Lm
#define Lm  0.884
#define Rsr 98.27222427

#define p 2 // pares de polos

#if defined(MRAS) || defined(SMO) || defined(MRAS_SMO)
#define Ts 4e-6           // tempo de amostragem do observador 1/(fsw*100)
#elif defined(EKF) || defined(EKF_MRAS) || defined(EKF_SMO)
#define Ts 16e-6
#endif

//Parâmetros do Observador
#define fsw 6e3            // Frequência de chaveamento
#define sigma 0.346442359  // coeficiente total de dispersão 1 - (Lm^2/(Ls*Lr))
#define Tr 0.011939615     // constante de tempo rotórica (Lr/Rr)
#define fpb_a 800          // parâmetro para FPB
#define fpb_b -792         // parâmetro para FPB (-0.99*a)


// Definicoes para o observador SMO
#define Pn   180        // Potência nominal (0.25 cv)
#define Vn   380        // Tensão de linha da rede elétrica
#define fn   60         // Frequência nominal da rede elétrica
#define rend 0.64       // Rendimento
#define fp   0.65       // Fator de Potência
#define Sn   432.69231  // Pn/(rend*fp)

#define H 0.00045       // Momento de inércia
#define F 0.0001        // Coeficiente de atrito

#define k2 2.764830516
#define k1 98.37266975
#define gamma 2.341101701
#define gamma1 0.01
#define gamma2 0.01

#define Tlinha 0.003680448

#define sig 0.283025792340101
#define Kr 0.846743295019157

#endif /* PARAMS_H_ */
