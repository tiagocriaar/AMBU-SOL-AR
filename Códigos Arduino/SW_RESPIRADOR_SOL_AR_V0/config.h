#ifndef _CONFIG_H_
#define _CONFIG_H_
#include "Arduino.h"

/* Definição do hardware 
  HARDWARE 0 definição oficial, 
  ativa motor em HIGH
  
  HARDWARE 1 teste 1
  ativa motor em LOW
  
  HARDWARE 2 defnição de teste de Albério
  Usando Arduino Mega, RAMPS 1.4, LCD Grafico 128X64
*/

/* Define qual hardwere para compilação */
#ifndef HARDWARE
  #define HARDWARE 0 //0=Hardware oficial, 1=teste1, 2=Teste Albério
#endif

#ifdef HARDWARE
  #if (HARDWARE == 0)
    /* Define metodo de habilitação do motor */
    #define HABILITA_MOTOR HIGH
  #elif (HARDWARE == 1)
    /* Define metodo de habilitação do motor */
    #define HABILITA_MOTOR LOW
  #elif (HARDWARE == 2)
    /* Comentar quando não estiver usando o IHM Gráfico */ 
    #define IHM_RAMPS_128X64

    /* Comentar quando não estiver usando a RAMPS para o driver do motor */
    #define RAMPS_DRIVER

    /* Define metodo de habilitação do motor */
    #define HABILITA_MOTOR LOW
  #endif  
#endif  

//String versao_SOLAR = " Versao Zero";

const byte pot_pressao_AMBU  =  A1;         // Leitura de pressão do Ambu
const byte pot_paciente      =  A2;         // Leitura de pressão do Paciente
const byte pot_vacuo         =  A3;         // Leitura de vácuo
const byte pot_Plato         =  A4;         // Regulagem de tempo do platô

//#define PIN_Y_MIN 14
//#define PIN_Y_MAX 15

#if defined(IHM_RAMPS_128X64)
  const byte port_Roraty_Encoder1  = 33;
  const byte port_Roraty_Encoder2  = 31;  
  const byte port_Roraty_EncoderBT = 35;
#else
  const byte LCD_address = 0x27;
  const byte port_Roraty_Encoder1 = 6;
  const byte port_Roraty_Encoder2 = 7;
#endif

const byte Btn_Avanca_MP   =  2;          // Botão de avanço do motor de passo (pino 2)
const byte Btn_Recua_MP    =  3;          // Botão de recuo do motor de passo (pino  3)
const byte led_pino        = 13;          // Pino do LED Arduino Mega

#if defined(RAMPS_DRIVER)
  const byte enable_MP       = A2;          // Habilita o driver do motor de passo (pino 24)
  const byte dir_MP          = A7;          // Define a direçao do motor de passo  (pino 25)
  const byte pulso_MP        = A6;          // Trem de pulso pra ajuste de velocidade (pino 5)
  const byte pot_MP          = A5;         // POTENCIÔMETRO VEL MOTOR DE PASSO
#else
  const byte pot_MP          = A0;         // POTENCIÔMETRO VEL MOTOR DE PASSO
  const byte enable_MP       = 24;          // Habilita o driver do motor de passo (pino 24)
  const byte dir_MP          = 25;          // Define a direçao do motor de passo  (pino 25)
  const byte pulso_MP        =  5;          // Trem de pulso pra ajuste de velocidade (pino 5)
#endif

const byte     Falha_E         = 28;          // Sinal de falha de energia (ligar nobreak)
const byte     TESTE_G         = 29;          // Botão de teste geral
const byte     R1_GER          = 30;          // Relé Geral
#if defined(IHM_RAMPS_128X64)
  const byte     R2_AVAN         = 14;          // Relé de Avanço do motor de passo
  const byte     R4_V_AMBU       = 15;          // Relé de Valvula do Ambu
  const byte     R6_V_AR         = 18;          // Relé de Ar
#else
  const byte     R2_AVAN         = 31;          // Relé de Avanço do motor de passo
  const byte     R4_V_AMBU       = 33;          // Relé de Valvula do Ambu
  const byte     R6_V_AR         = 35;          // Relé de Ar
#endif
const byte     R3_RET          = 32;          // Relé de retorno do motor de passo
const byte     R5_V_02         = 34;          // Relé de oxigênio
const byte     R7_BYPASS       = 36;          // Relé de By-pass

// TECLADO  LIN1(D38)           38
// TECLADO  LIN2(D40)           40
// TECLADO  LIN3(D42)           42
// TECLADO  LIN4(D44)           44
// TECLADO  COL1(D46)           46
// TECLADO  COL2(D48)           48
// TECLADO  COL3(D50)           50
// TECLADO  COL4(D52)           52

const byte     CFC_Inicio      = 39;          // Chave fim de curso início (home position)
const byte     CFC_Fim         = 41;          // Chave fim de curso fim (limte pressão do AMBU)

const byte Keys[4][4] =                 // Definicao dos valores das 16 teclas
  {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
  };


const byte LinhaPINO[4] =   {38, 40, 42, 44};      // Linhas do teclado D38 a D44
const byte ColunaPINO[4] = {46, 48, 50, 52};      // Colunas do teclado D46 a D52

#endif
