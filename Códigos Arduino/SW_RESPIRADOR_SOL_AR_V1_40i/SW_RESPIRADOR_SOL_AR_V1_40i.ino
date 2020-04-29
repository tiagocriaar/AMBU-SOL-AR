//============================================================================================================================
//
//           PROGRAMA RESPIRADOR SOL AR
//
//           https://github.com/tiagocriaar/AMBU-SOL-AR
//
//           Arduino Mega 2560  - Arduino IDE 1.8.12
//           Motor de Passo NEMA23 15 kgf.cm / Driver de motor WD2404
//           Display LCD 20x4 I2C / teclado com 16 Teclas -  Keypad 4x4 - Rotary Encoder
//           Eixo sem fim - Fuso 25 mm
//
//============================================================================================================================
//
//   Versao 1.40h
//   DATA: 26/04/20
//   HORA: 08:00    revisão e alterações Gustavo
//                  Alterações - estados de direção e enable motor  - Modo de passo = 2/A meio passo
//                  Configuração dos parametros / calibração do motor
//
//   ANALISTA
//   DE SISTEMAS:   SÉRGIO GOULART ALVES PEREIRA
//
//   PROGRAMADORES: SÉRGIO GOULART ALVES PEREIRA
//                  FELIPE JONATHAN DIAS TOBIAS
//                  JOSÉ GUSTAVO ABREU MURTA
//                  ALBERIO LIMA
//
//    REVISORES:    JOSÉ GUSTAVO ABREU MURTA
//                  SÉRGIO GOULART ALVES PEREIRA
//
//   COLABORADORES: EDUARDO HENRIQUE MARCONDES
//                  RUI VIANA
//

/* Bibliotecas de terceiros */
#include <Keypad.h>                      // https://github.com/Chris--A/Keypad  
#include <LiquidCrystal_PCF8574.h>       // https://github.com/mathertel/LiquidCrystal_PCF8574  
#include <FlexyStepper.h>                // https://github.com/Stan-Reifel/FlexyStepper    
#include <RotaryEncoder.h>               // https://github.com/mathertel/RotaryEncoder  

/* Bibliotecas e cabeçalhos locais */
#include "main.h"
#include "LCD_functions.h"
#include "controle_MP.h"
#include "controle_ambu.h"
#include "sensores.h"
#include "LCD_functions.h"

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Variáveis globais
int   frequencia_MP                 = 2000;  // valor de frequencia para ajuste de velocidade
float potenciometro_Plato           = 1000;  // leitura do potenciometro do Tempo de Plato
int   valor_pot_MP                  = 1000;  // leitura do potenciometro do MP
float valor_pot_TP                  =  200;  // leitura do potenciometro do Tempo de Plato
int   valor_pot_pressao_AMBU        = 0;     // leitura do Pot de pressão do Ambu - A1
float valor_pressao_Ambu            = 20;    // leitura da pressão do Ambu
float valor_PPI                     = 0;     // leitura da pressão inspiratória de pico em cmH2O
float frequencia_respiratoria       = 16;    // frequencia respiratoria
float periodo_respiratorio          = 3.75;  // período respiratório FR - 16
int   tempo_inspiracao              = 1000;  // tempo de inspiração em ms
float tempo_inspiracao_seg          = 1.0;   // tempo de inspiração em segundos
float tempo_expiracao               = 2.5;   // tempo de expiração
float divisor_relacao_insp_exp      = 2.5;   // divisor na relação inspiração x expiração
int   tempo_plato                   = 200;   // tempo de plato em ms
float tempo_plato_seg               = 0.2;   // tempo de plato em segundos
float pausa_expiratoria             = 0;     // pausa expiratoria
int   ciclo_inspiratorio            = 1250;  // ciclo inspiratório em ms
float ciclo_expiratorio             = 2.5;   // ciclo expiratorio
int   ponteiro_selecao              = 0;     // ponteiro de seleção de parametros
int   contador_encoder              = 0;     // contador do encoder rotativo
bool  selecao_parametro_irpm        = false; // parametro irpm
int   indice_velocidade             = 4;     // velocidade 134 mm/seg
int   velocidade_respirador;

float valor_pressao_Ambu_inicial = 20;

float Tempo_seg_Plato_NOK =  300;  // Tempo de platô  - convertido em segundos antes do confirma
float Tempo_seg_Plato_OK  =  301;  // Tempo de platô  - convertido em segundos após o confirma
float Tins            =    0;      // Tempo de inspiração  geralmente 1.0 s
float Texp            =    0;      // Tempo de expiração   geralmente 0.5 s
float TPlato          =    0;      // Tempo de platô
int   cod_causa       =    0;
char  tag_causa       =   "";
char  nome_causa_LN2  =   "";
char  nome_causa_LN3  =   "";
int   cod_efeito      =    0;
char  tag_efeito      =   "";
char  nome_efeito_LN2 =   "";
char  nome_efeito_LN3 =   "";

FlexyStepper stepper;                     // cria objeto stepper
LiquidCrystal_PCF8574 lcd(LCD_address);      // configura endereço do LCD
Keypad keypad = Keypad( makeKeymap(Keys), LinhaPINO, ColunaPINO, linhas, colunas );  // configuração do teclado
RotaryEncoder encoder(6, 7);              // pin CLK(A)= D6    pin DT(B)= D7  Encoder Rotativo

int newPos   = 0;                         // posição do Encoder Rotativo
int lastPos1 = 0;
int lastPos2 = 0;
int lastPos3 = 0;
int lastPos4 = 0;

float     percurso_Ambu;                  // percurso do movimento da aba em mm
int       StepsPerRevolution;             // passos por volta do motor (depende do modo de passo)
int       sentidoAmbu;                    // sentido do motor = -1 para Ambu(anti-horário), 1 para posição inicial (horário)
int       velocidade_mm_por_seg;          // velocidade em mm por segundos


//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
// Configuração do Hardware

void setup()
{
  Serial.begin(9600);                    // Habilita a comunicação com a velocidade de 9600 bits por segundo

  pinMode (Btn_enter, INPUT_PULLUP);     // Define o pino D8 como entrada digital do botão Enter
  pinMode (Btn_Avanca_MP, INPUT_PULLUP); // Define o pino 21 como entrada digital do botão Avança
  pinMode (Btn_Recua_MP, INPUT_PULLUP);  // Define o pino 20 como entrada digital do botão Recua
  pinMode (CFC_Inicio, INPUT_PULLUP);    // Define o pino 39 como entrada digital da chave CFC_Inicio
  pinMode (CFC_Fim , INPUT_PULLUP);      // Define o pino 41 como entrada digitalda chave CFC_Fim
  pinMode (enable_MP, OUTPUT);           // Define o pino 24 como saída digital de habilitação do driver
  pinMode (dir_MP, OUTPUT);              // Define o pino 25 como saída digital de sentido de giro
  pinMode (pulso_MP, OUTPUT);            // Define 3 pino como saída do trem de pulso
  pinMode (pino_teste, OUTPUT);          // Define o pino 22 como saída digital - pino de teste

  digitalWrite  (dir_MP, HIGH);          // Define sentido de giro inicial como avançar

  percurso_Ambu = 100;                                   // percurso do Ambu - 100 mm
  StepsPerRevolution = 400;                              // passos por volta = 400 passo - modo 2/A meio passo
  stepper.setStepsPerRevolution (StepsPerRevolution);    // configura passos por volta para o Driver do motor
  stepper.setStepsPerMillimeter(32);                     // meio micropasso - 400 passos/25mm => 16 passos/mm  (32 para motor da fabrica) 
  stepper.connectToPins(pulso_MP , dir_MP);              // configura pinos no driver WD2404

  desativa_Driver_Motor();

  lcd.begin(20, 4);                      // inicializa Display LCD 20x4

  LCD_Inicial ();            // Tela inicial no display LCD
  delay (500);                           // atraso 2 segundos

  keypad.addEventListener(keypadEvent);  // evento para leitura do teclado

  // attachInterrupt(digitalPinToInterrupt(Btn_Avanca_MP), procedimento_avanca_MP, FALLING);             // Habilita interrupção no botão de avanço
  // attachInterrupt(digitalPinToInterrupt(Btn_Recua_MP) , procedimento_recua_MP, FALLING);              // Habilita interrupção no botão de recuo

} //FIM do void setup

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Função principal

void loop()
{
  char key = keypad.getKey();            // faz varredura dos botões do teclado
  leitura_Encoder_Rotativo ();           // faz a leitura do Encoder Rotativo
}

void keypadEvent(KeypadEvent key)
{
  switch (keypad.getState())                  // verifica se algum botão foi pressionado
  {
    case PRESSED:                             // identifica o botão pressionado
      //-------------------------------------------------------------------------------
      if (key == '1')                         // Botão 1
      {
        calculo_Parametros ();
        LCD_mostra_Parametros ();             // ativa painel do operador do Ambu
      }
      //-------------------------------------------------------------------------------
      if (key == '2')                         // Botao 2
      {
        // Serial.println("tecla 2");
      }
      //-------------------------------------------------------------------------------
      if (key == '3')                         // Botao 3
      {
        // Serial.println("tecla 3");
      }
      //-------------------------------------------------------------------------------
      if (key == '4')                          // Botao 4 - Inicializar o Ambu
      {
        inicializa_Ambu();                     // posiciona a aba na posição inicial
      }
      //------------------------------------------------------------------------------- 1.40i
      if (key == '5')                                                                 // Botao 5 - teste do motor sentido do Ambu
      {
        velocidade_mm_por_seg = velocidade_inspiracao [indice_velocidade];            // velocidade do motor em mm por segundos
        LCD_avanca_Ambu();                                                            // Print avança Ambu no LCD       
        sentidoAmbu = -1;                                                             // sentido do motor = -1 para Ambu       
        percurso_Ambu = 100;                                                          // percurso do movimento da aba em mm
        motorPressionaAmbu ();                                                        // movimenta motor
      }
      //------------------------------------------------------------------------------- 1.40i
      if (key == '6')                                                                  // Botao 6 - teste do Ambu
      {
        calculo_Parametros ();
        LCD_mostra_Parametros ();
        velocidade_respirador = velocidade_inspiracao [indice_velocidade];            // velocidade do motor em mm por segundos
        respiradorAmbu ();                                                            // simulação do movimento do Ambu
      }
      //-------------------------------------------------------------------------------
      if (key == '7')                         // Botao 7
      {
        leitura_pressao_Ambu () ;             //
        LCD_Mostra_Valor_Pressao_Ambu();      //
      }
      //------------------------------------------------------------------------------- 1.40i
      if (key == '8')                                                                 // Botao 8 - teste do motor sentido posição inicial
      {
        velocidade_mm_por_seg = velocidade_inspiracao [indice_velocidade];            // velocidade do motor em mm por segundos
        LCD_retorna_Ambu();                                                           // Print avança Retorna no LCD
        sentidoAmbu = 1;                                                              // sentido do motor = 1 para posição inicial       
        percurso_Ambu = 100;                                                          // percurso do movimento da aba em mm
        motorPressionaAmbu ();                                                        // movimenta motor
      }
      //-------------------------------------------------------------------------------
      if (key == '9')                         // Botao 9 - testes de calibração do motor de passo
      {
        // Serial.println("tecla 9");
        calibracao_velocidade_motor();        // testes de calibração do motor de passo
      }
      //-------------------------------------------------------------------------------
      if (key == '0')                         // Botao 0
      {
        // Serial.println("tecla 0");
      }
      //-------------------------------------------------------------------------------
      if (key == '*')                         // Botao *
      {
        // Serial.println("tecla *");

      }
      //-------------------------------------------------------------------------------
      if (key == '#')                         // Botao #
      {
        // Serial.println("tecla #");
      }
      //-------------------------------------------------------------------------------
      if (key == 'A')                                 // Botao A
      {
        ponteiro_selecao = 1;                        // configuração da Frequencia Respiratoria
        encoder.setPosition(lastPos1);               // zera a posição do encoder
        LCD_mostra_selecao_parametro ();
      }
      //-------------------------------------------------------------------------------
      if (key == 'B')                                // Botao B
      {
        ponteiro_selecao = 2;                        // configuração da Pressão do Ambu
        encoder.setPosition(lastPos2);               // zera a posição do encoder
        LCD_mostra_selecao_parametro ();
      }
      //-------------------------------------------------------------------------------
      if (key == 'C')                                 // Botao C
      {
        ponteiro_selecao = 3;                         // configuração do Tempo de Inspiração
        encoder.setPosition(lastPos3);                // zera a posição do encoder
        LCD_mostra_selecao_parametro ();

      }
      //-------------------------------------------------------------------------------
      if (key == 'D')                                 // Botao D
      {
        ponteiro_selecao = 4;                         // configuração Tempo de Platô
        encoder.setPosition(lastPos4);                // zera a posição do encoder
        LCD_mostra_selecao_parametro ();
      }
      //-------------------------------------------------------------------------------
  }
}
