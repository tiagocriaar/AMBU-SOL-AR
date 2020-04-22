//============================================================================================================================
//
//           PROGRAMA RESPIRADOR SOL AR
//
//           https://github.com/tiagocriaar/AMBU-SOL-AR
//
//           Arduino Mega 2560  - Arduino IDE 1.8.10
//           Motor de Passo NEMA23 15 kgf.cm / Driver de motor WD2404
//           Display LCD 20x4 I2C / teclado com 16 Teclas -  Keypad 4x4 - Rotary Encoder
//           Eixo sem fim - Fuso 25 mm
//
//============================================================================================================================
//
//   Versao 1.40e
//   DATA: 20/04/20
//   HORA: 23:00    revisão e alterações Gustavo
//                  Alterações - estados de direção e enable motor  - Modo de passo = 2/A meio passo
//                  Configuração dos parametros / calibração do motor
//
//   ANALISTA
//   DE SISTEMAS:   SÉRGIO GOULART ALVES PEREIRA
//
//   PROGRAMADORES: SÉRGIO GOULART ALVES PEREIRA
//                  FELIPE JONATHAN DIAS TOBIAS
//                  JOSÉ GUSTAVO ABREU MURTA
//
//    REVISORES:    JOSÉ GUSTAVO ABREU MURTA
//                  SÉRGIO GOULART ALVES PEREIRA
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
float periodo_respiratorio          = 3.75;  // período respiratório
float tempo_inspiracao              = 1.0;   // tempo de inspiração
float relacao_insp_exp              = 2.5;   // relação inspiração x expiração
float tempo_plato                   = 0.15;  // tempo de plato
float pausa_expiratoria             = 0.10;  // pausa expiratoria
float ciclo_inspiratorio            = 1.15;  // ciclo inspiratório
float ciclo_expiratorio             = 2.65;   // ciclo expiratorio
int ponteiro_selecao                = 0;     // ponteiro de seleção de parametros
bool selecao_parametro_irpm         = false; // parametro irpm
float Tempo_seg_Plato_NOK =  300;  // Tempo de platô  - convertido em segundos antes do confirma
float Tempo_seg_Plato_OK  =  301;  // Tempo de platô  - convertido em segundos após o confirma
float Tins            =    0;      // Tempo de inspiração  geralmente 1.0 s
float Texp            =    0;      // Tempo de expiração   geralmente 0.5 s
float TPlato          =    0;      // Tempo de platô  - estado entre inspiração e expiração - ver uma variação 0s ou ... ou .. 0.7 s
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

int newPos = 0;                           // posição do Encoder Rotativo

float     percurso_Ambu;                  // percurso do movimento da aba em mm
int       StepsPerRevolution;             // passos por volta do motor (depende do modo de passo)
int       sentidoAmbu;                    // sentido do motor = -1 para Ambu(anti-horário), 1 para posição inicial (horário)
float     velocidade_mm_por_seg;          // velocidade em mm por segundos


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

  digitalWrite  (dir_MP, HIGH);          // Define sentido de giro inicial como avançar

  percurso_Ambu = 100;                                   // percurso do Ambu - 100 mm
  StepsPerRevolution = 400;                              // passos por volta = 400 passo - modo 2/A meio passo
  stepper.setStepsPerRevolution (StepsPerRevolution);    // configura passos por volta para o Driver do motor
  stepper.setStepsPerMillimeter(16);                     // meio micropasso - 400 passos/25mm => 16 passos/mm
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
      if (key == '1')                         // Botão 1 - LIGA
      {
        Serial.println("tecla 1");
      }
      //-------------------------------------------------------------------------------
      if (key == '2')                         // Botao 2 - PARAR
      {
        Serial.println("tecla 2");
      }
      //-------------------------------------------------------------------------------
      if (key == '3')                         // Botao 3
      {
        Serial.println("tecla 3");
      }
      //-------------------------------------------------------------------------------
      if (key == '4')                                    // Botao 4 para Presionar Ambu
      {
        Serial.println("tecla 4 - Inicializa Ambu");
        inicializa_Ambu();                               // posiciona a aba na posição inicial - linha 322
      }
      //-------------------------------------------------------------------------------
      if (key == '5')                                    // Botao 5
      {
        Serial.println("tecla 5");
      }
      //-------------------------------------------------------------------------------
      if (key == '6')                         // Botao 6
      {
        Serial.println("tecla 6");
      }
      //-------------------------------------------------------------------------------
      if (key == '7')                         // Botao 7
      {
        // Serial.println("tecla 7");
        leitura_pressao_Ambu () ;             // linha 276
        LCD_Mostra_Valor_Pressao_Ambu();     // linha 467
      }
      //-------------------------------------------------------------------------------
      if (key == '8')                         // Botao 8
      {
        Serial.println("tecla 8");
      }
      //-------------------------------------------------------------------------------
      if (key == '9')                         // Botao 9
      {
        Serial.println("tecla 9");
      }
      //-------------------------------------------------------------------------------
      if (key == '0')                         // Botao 0
      {
        Serial.println("tecla 0");
      }
      //-------------------------------------------------------------------------------
      if (key == '*')                         // Botao *
      {
        // Serial.println("tecla *");                           // presione * para selecionar o parametro
        ponteiro_selecao = ++ ponteiro_selecao;                 // adiciona 1 ao ponteiro de seleçao dos parametros
        if (ponteiro_selecao > 5) ponteiro_selecao = 0;         // de 0 a 5 somente
        LCD_mostra_selecao_parametro ();                        // mostra o parametro selecionado
      }
      //-------------------------------------------------------------------------------
      if (key == '#')                         // Botao #
      {
        Serial.println("tecla #");
      }
      //-------------------------------------------------------------------------------
      if (key == 'A')                          // Botao A
      {
        // Serial.println("tecla A");          // teste de motor - pressiona o Ambu
        sentidoAmbu = -1;                      // sentido do motor = -1 para Ambu
        velocidade_mm_por_seg = 50;            // velocidade do motor em mm por segundos
        percurso_Ambu = 100;                   // percurso do movimento da aba em mm
        motorPressionaAmbu ();                 // movimenta motor
      }
      //-------------------------------------------------------------------------------
      if (key == 'B')                         // Botao B
      {
        // Serial.println("tecla B");         // teste do motor - vai e vem
        LCD_mostra_Parametros ();             // simulaçao da tela
        respiradorAmbu ();                    // simulação do movimento do Ambu - linha 423
      }
      //-------------------------------------------------------------------------------
      if (key == 'C')                                           // Botao C
      {
        // Serial.println("tecla C");
        LCD_mostra_Parametros ();                               // linha 547
      }
      //-------------------------------------------------------------------------------
      if (key == 'D')                         // Botao D
      {
        Serial.println("tecla D");
        calibracao_velocidade_motor();
      }
      //-------------------------------------------------------------------------------
  }
}
