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
//
//============================================================================================================================
//
//   https://github.com/mathertel/LiquidCrystal_PCF8574          Biblioteca do Display
//   https://github.com/Chris--A/Keypad                          Biblioteca do teclado
//   https://github.com/Stan-Reifel/FlexyStepper                 Biblioteca FlexyStepper
//   https://github.com/mathertel/RotaryEncoder                  Biblioteca RotaryEncoder
//
//============================================================================================================================
// Bibliotecas

#include <Wire.h>                        // Biblioteca Wire 
#include <Keypad.h>                      // Biblioteca para teclado 
#include <LiquidCrystal_PCF8574.h>       // Biblioteca para LCD com I2C
#include <FlexyStepper.h>                // Biblioteca controle do motor de passo
#include <RotaryEncoder.h>               // Biblioteca do Encoder rotativo

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Variáveis globais

char  versao_SOLAR    =      " Versao 1_40e";

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

int LCD_address       = 0x3F;                // Endereço I2C - LCD PCF8574 = 0x27 / Gustavo = 0x3F
LiquidCrystal_PCF8574 lcd(LCD_address);      // configura endereço do LCD

const byte linhas  = 4;                      // linhas do teclado
const byte colunas = 4;                      // colunas do teclado
char Keys[linhas][colunas] =                 // Definicao dos valores das 16 teclas
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte LinhaPINO[linhas] =   {38, 40, 42, 44};      // Linhas do teclado D38 a D44
byte ColunaPINO[colunas] = {46, 48, 50, 52};      // Colunas do teclado D46 a D52

Keypad keypad = Keypad( makeKeymap(Keys), LinhaPINO, ColunaPINO, linhas, colunas );  // configuração do teclado

RotaryEncoder encoder(6, 7);              // pin CLK(A)= D6    pin DT(B)= D7  Encoder Rotativo

int newPos = 0;                           // posição do Encoder Rotativo

float     percurso_Ambu;                  // percurso do movimento da aba em mm
int       StepsPerRevolution;             // passos por volta do motor (depende do modo de passo)
int       sentidoAmbu;                    // sentido do motor = -1 para Ambu(anti-horário), 1 para posição inicial (horário)
float     velocidade_mm_por_seg;          // velocidade em mm por segundos

FlexyStepper stepper;                     // cria objeto stepper

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Rótulos das pinagens

const int   pot_MP            =  A0;         // POTENCIÔMETRO VEL MOTOR DE PASSO
const int   pot_pressao_AMBU  =  A1;         // Leitura de pressão do Ambu
const int   pot_paciente      =  A2;         // Leitura de pressão do Paciente
const int   pot_vacuo         =  A3;         // Leitura de vácuo
const int   pot_Plato         =  A4;         // Regulagem de tempo do platô

// TX0                           0           // FUTURO
// RX0                           1           // FUTURO
const int     Btn_Avanca_MP   =  2;          // Botão de avanço do motor de passo (pino 2)
const int     Btn_Recua_MP    =  3;          // Botão de recuo do motor de passo (pino  3)
const int     pulso_MP        =  5;          // Trem de pulso pra ajuste de velocidade (pino 5)
const int     Btn_enter       =  8;          // botão enter
const int     led_pino        = 13;          // Pino do LED Arduino Mega

// Rotary Encoder A             06
// Rotary Encoder B             07
// RS485                        14           // FUTURO
// RS485                        15           // FUTURO

// const int  Btn_Liga_MP     = 18;          // Botão para ligar o motor de passo
// const int  Btn_Para_MP     = 19;          // Botão de parada do motor de passo

// DISPLAY SDA                  20           // DISPLAY SDA
// DISPLAY SCL                  21           // DISPLAY SCL

const int     enable_MP       = 24;          // Habilita o driver do motor de passo (pino 24)
const int     dir_MP          = 25;          // Define a direçao do motor de passo  (pino 25)

const int     Falha_E         = 28;          // Sinal de falha de energia (ligar nobreak)
const int     TESTE_G         = 29;          // Botão de teste geral
const int     R1_GER          = 30;          // Relé Geral
const int     R2_AVAN         = 31;          // Relé de Avanço do motor de passo
const int     R3_RET          = 32;          // Relé de retorno do motor de passo
const int     R4_V_AMBU       = 33;          // Relé de Valvula do Ambu
const int     R5_V_02         = 34;          // Relé de oxigênio
const int     R6_V_AR         = 35;          // Relé de Ar
const int     R7_BYPASS       = 36;          // Relé de By-pass

// TECLADO  LIN1(D38)           38
// TECLADO  LIN2(D40)           40
// TECLADO  LIN3(D42)           42
// TECLADO  LIN4(D44)           44
// TECLADO  COL1(D46)           46
// TECLADO  COL2(D48)           48
// TECLADO  COL3(D50)           50
// TECLADO  COL4(D52)           52

const int     CFC_Inicio      = 39;          // Chave fim de curso início (home position)
const int     CFC_Fim         = 41;          // Chave fim de curso fim (limte pressão do AMBU)

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Declaração das funções

void procedimento_ligar();                        // Liga o motor
void procedimento_parar();                        // Para o fuso
void procedimento_avanca_MP();                    // Avançar o fuso
void procedimento_recua_MP();                     // Recua o fuso

void LCD_Inicial (char versao_SOLAR);
void LCD_Teste_MP ();
void LCD_Teste_Potenciometro_TP ();     // Tela de teste do potenciometro do Tempo de Plato
void procedimento_ler_pot_TP(float valor_pot_TP, float potenciometro_Plato, float Tempo_seg_Plato_NOK); //Lê a entrada do potenciômetro do Tempo de Plato
void LCD_Mostra_Valor_Potenciometro_TP (float Tempo_seg_Plato_NOK); // Mostra o valor do potenciômetro para o plato

void keypadEvent(KeypadEvent key, float Tempo_seg_Plato_OK, float Tempo_seg_Plato_NOK);

void LCD_TECLA_A_Pressionada(float Tempo_seg_Plato_OK); // Confirma tecla A pressionada para aceitar o valor do Tempo de Plato

void procedimento_tag_causa(int  cod_causa,  char tag_causa,  char nome_causa_LN2,  char nome_causa_LN3);  // Lê a tipo de causa
void procedimento_tag_efeito(int cod_efeito, char tag_efeito, char nome_efeito_LN2, char nome_efeito_LN3); // Lê o tipo de defeito
void procedimento_mostrar_causas_na_tela_do_micro(int cod_causa);                                          // mostrar causas na tela do micro

void LCD_Mostra_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3);
void LCD_Mostra_Lista_Causas(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3);

void LCD_HomePosition ();
void LCD_iniciaMotor ();
void LCD_CFC_inicio ();
void LCD_CFC_fim ();
void LCD_pressionaAmbu ();
void LCD_soltaAmbu ();
void LCD_Ligando_MP();
void LCD_Desligando_MP();

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

  int error;                             // variável de erro para o Display LCD

  Wire.begin();                          // Inicializa interface do LCD
  Wire.beginTransmission(LCD_address);   // incializa interface I2C
  error = Wire.endTransmission();        // termina transmissão se houver erro

  lcd.begin(20, 4);                      // inicializa Display LCD 20x4

  LCD_Inicial (versao_SOLAR);            // Tela inicial no display LCD
  delay (500);                           // atraso 2 segundos

  keypad.addEventListener(keypadEvent);  // evento para leitura do teclado

  // attachInterrupt(digitalPinToInterrupt(Btn_Avanca_MP), procedimento_avanca_MP, FALLING);             // Habilita interrupção no botão de avanço
  // attachInterrupt(digitalPinToInterrupt(Btn_Recua_MP) , procedimento_recua_MP, FALLING);              // Habilita interrupção no botão de recuo

} //FIM do void setup

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Declaração das Funções ----------------------------------------------------------------------
void leitura_Encoder_Rotativo ()
{
  static int pos = 0;                     // lendo os contatos do  encoder
  encoder.tick();
  newPos = encoder.getPosition();

  if (pos != newPos)                      // se a posição foi alterada
  {
    if (ponteiro_selecao == 1) configura_frequencia_respiratoria ();
    if (ponteiro_selecao == 2) configura_pressao_ambu ();
    if (ponteiro_selecao == 3) configura_tempo_inspiracao ();
    if (ponteiro_selecao == 4) configura_relacao_IE ();
    if (ponteiro_selecao == 5) configura_tempo_plato ();
    pos = newPos;
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ligar()
{
  digitalWrite(enable_MP, LOW);          // Habilita driver
  Serial.println("Motor Ligado");        // Mensagem no monitor: "Motor ligado"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_parar()
{
  digitalWrite(enable_MP, HIGH);         // Desabilita driver
  Serial.println("Motor Parado");        // Mensagem no monitor: "Motor parado"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ler_pot_MP()
{
  valor_pot_MP  = analogRead(pot_MP);                        // Lê o valor da tensão no potenciometro
  frequencia_MP = map(valor_pot_MP, 0, 1023, 2000, 4000);    // Converte a tensão em frequencia
}

//------------------------------------------------------------------------------------------------------------------------------------
void leitura_pressao_Ambu ()                                                // leitura da pressão inspiratória de pico em cmH2O
{
  valor_pot_pressao_AMBU = analogRead(pot_pressao_AMBU);                    // medição na porta analogica A1 - Leitura de pressão do Ambu
  valor_pressao_Ambu = valor_pot_pressao_AMBU * 0.05859375 ;                // 60 dividido por 1024 => sensor de pressão em até 60 cmH20
  return valor_pressao_Ambu;                                                // retorna o valor da pressão inspiratória de pico
}

//---------------------------------------- Configuração dos Parâmetros ------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------
void configura_frequencia_respiratoria ()                                 // configuração usando o rotary encoder
{
  lcd.setCursor(10, 0);                                                   // coluna 9 e linha 0
  lcd.print(">");                                                         // mostra no LCD
  frequencia_respiratoria = 16 + newPos;                                  // frequencia_respiratoria inicial = 16
  if (frequencia_respiratoria < 10 )
  {
    if (frequencia_respiratoria < 5 ) frequencia_respiratoria = 5 ;       // frequencia minima = 5
    lcd.setCursor(11, 0);                                                 // coluna 11 e linha 0
    lcd.print("     ");                                                   // mostra no LCD
    lcd.setCursor(12, 0);                                                 // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria, 1);                                // mostra no LCD
  }
  else
  {
    if (frequencia_respiratoria > 30) frequencia_respiratoria = 30 ;      // frequencia maxima 30 (acho que 60 não vai dar)
    lcd.setCursor(11, 0);                                                 // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria, 1);                                // mostra no LCD
  }
  calculo_Parametros ();                                                  // efetua calculo dos parmetros
  delay (50);                                                             // atraso 50 ms
}

//---------------------------------------------------------------------------------------------------------------------------------------
void configura_pressao_ambu ()                                           // configuração usando o rotary encoder
{
  lcd.setCursor(9, 1);                                                   // coluna 9 e linha 1
  lcd.print(">");                                                        // mostra no LCD
  valor_pressao_Ambu  = 20 + newPos;                                     // pressão inicial = 20

  if (valor_pressao_Ambu  < 10 ) valor_pressao_Ambu = 10 ;               // pressão no Ambu minima = 10
  if (valor_pressao_Ambu > 60) valor_pressao_Ambu = 60 ;                 // pressão no Ambu maxima 60

  lcd.setCursor(10, 1);                                                  // coluna 10 e linha 1
  lcd.print(valor_pressao_Ambu, 1);                                      // mostra no LCD
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_inspiracao ()                                       // configuração usando o rotary encoder
{
  lcd.setCursor(4, 2);                                                   // coluna 4 e linha 2
  lcd.print(">");                                                        // mostra no LCD
  tempo_inspiracao  = 1.0 + (newPos * 0.1) ;                             // tempo_inspiracao inicial = 1.0

  if (tempo_inspiracao  < 0.9 ) tempo_inspiracao = 0.9 ;                 // tempo_inspiracao minima = 0.9
  if (tempo_inspiracao  > 1.4 ) tempo_inspiracao = 1.4 ;                 // tempo_inspiracao maxima = 1.4

  lcd.setCursor(5, 2);                                                   // coluna 5 e linha 2
  lcd.print(tempo_inspiracao, 1);                                        // mostra no LCD
  calculo_Parametros ();                                                 // efetua calculo dos parmetros
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_relacao_IE ()                                             // configuração usando o rotary encoder
{
  lcd.setCursor(14, 2);                                                  // coluna 14 e linha 2
  lcd.print(">");                                                        // mostra no LCD
  relacao_insp_exp  = 2.5 + (newPos * 0.5) ;                             // relacao_insp_exp inicial = 2.5

  if (relacao_insp_exp  < 1 ) relacao_insp_exp = 1 ;                     // relacao_insp_exp minima = 1
  if (relacao_insp_exp  > 4.5 ) relacao_insp_exp = 4.5 ;                 // relacao_insp_exp maxima = 4.5

  lcd.setCursor(17, 2);                                                  // coluna 17 e linha 2
  lcd.print(relacao_insp_exp, 1);                                        // mostra no LCD
  calculo_Parametros ();                                                 // efetua calculo dos parmetros
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_plato ()                                           // configuração usando o rotary encoder
{
  lcd.setCursor(6, 3);                                                  // coluna 6 e linha 3
  lcd.print(">");                                                       // mostra no LCD
  tempo_plato  = 0.25 + (newPos * 0.05) ;                               // tempo_plato inicial = 0.25

  if (tempo_plato  < 0.1 ) tempo_plato = 0.1 ;                          // tempo_plato minima = 0.1
  if (tempo_plato  > 0.4 ) tempo_plato = 0.4 ;                          // tempo_plato maxima = 0.4

  lcd.setCursor(7, 3);                                                  // coluna 17 e linha 2
  lcd.print(tempo_plato, 2);                                            // mostra no LCD
  calculo_Parametros ();                                                // efetua calculo dos parmetros
  delay (50);                                                           // atraso 50 ms
}

//---------------------------------------- Cálculo dos Parâmetros ------------------------------------------------------------------
/*
   frequencia_respiratoria - 5 a 30 irpm (acho que não vai chegar no 60)
   valor_pressao_Ambu - 10 a 60 cmH2O
   tempo_inspiracao  - 0,9 a 1,4 seg
   relacao_insp_exp - 1 - 1,5 - 2,0 - 2,5 - 3,0 - 3,5 - 4,0 - 4,5
   tempo_plato - 0,1 - 0,15 - 0,2 - 0,25 - 0,3 - 0,35 - 0,40
*/
//----------------------------------------------------------------------------------------------------------------------------------
void calculo_Parametros ()
{
  pausa_expiratoria = 0.05;                                                               // intervalo medido entre EXP e INSP - 50 ms

  ciclo_inspiratorio = tempo_inspiracao + tempo_plato;                                    // calculo do ciclo inspiratorio em segundos
  ciclo_expiratorio = (relacao_insp_exp * tempo_inspiracao) + pausa_expiratoria;          // calculo do ciclo expiratorio em segundos

  periodo_respiratorio = ciclo_inspiratorio + ciclo_expiratorio;                          // calculo do período respiratório em segundos
  frequencia_respiratoria = 60 / periodo_respiratorio;                                    // calculo da frequencia respiratória em irpm

  lcd.setCursor(13, 3);                                                                   // coluna 6 e linha 3
  lcd.print("FR:");                                                                       // mostra no LCD
  lcd.setCursor(16, 3);                                                                   // coluna 17 e linha 2
  lcd.print(frequencia_respiratoria, 1);                                                  // mostra no LCD - frequencia calculada
}

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Controle do Motor de Passo ------------------------------------------------------------------
void desativa_Driver_Motor()
{
  digitalWrite(enable_MP, HIGH);          // Desativa o driver WD2404 = HIGH
  delay (10);                             // Atraso de 10 milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void ativa_Driver_Motor()
{
  digitalWrite(enable_MP, LOW);          // Ativa o driver WD2404 = LOW
  delay (10);                            // Atraso de 10  milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
//  sentido do motor = -1 para Ambu(anti-horário), 1 para posição inicial (horário)
//  Comprimento do eixo sem fim = 200 mm
//  Curso para compressão do AMBU = 100 mm ou 130 mm
//  FUSO 25 mm - (25 mm de avanço por volta)
//  Driver WED2404 configurado para meio passo 2/A  = 400 passos/volta
//  Para avançar os 100 mm são necessários 4 voltas no motor ou 4 x 400 passos = 1600 passos
//  Para avançar os 130 mm são necessários 5,2 voltas no motor ou 5,2 x 400 passos = 2080 passos

//--------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void inicializa_Ambu()                                                            // posiciona a aba na posição inicial
{
  LCD_inicializa_Respirador ();                                                   // mostra inicializa Ambu no LCD
  sentidoAmbu = 1;                                                                // sentido do motor = 1 para posição inicial
  percurso_Ambu = 100;                                                            // maxima distancia percorrida em milimetros
  stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em milimetros
  bool limitSwitchFlag = false;                                                   // flag para switch de limite
  ativa_Driver_Motor();                                                           // ativa driver do motor

  if (digitalRead(CFC_Inicio) == HIGH)                                            // se chave CFC_Inicio não foi acionada
  {
    stepper.setSpeedInMillimetersPerSecond (50);                                  // configura velocidade do motor em mm/segundo
    stepper.setAccelerationInMillimetersPerSecondPerSecond(150);                  // configura aceleraçao do motor - mm/s2
    stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );        // maxima distancia percorrida em milimetros = 100 (4 voltas)

    while (!stepper.motionComplete())                                             // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                  // gira o motor

      if (digitalRead(CFC_Inicio ) == LOW && (limitSwitchFlag == false))          // se a chave CFC_Inicio for acionada
      {
        stepper.setTargetPositionToStop();                                        // para o motor
        limitSwitchFlag = true;                                                   // muda o estado do flag
        lcd.setCursor(3, 3);                                                      // LCD coluna 3 e linha 3
        lcd.print("CFC Inicio OK    ");                                           // mostra no LCD
      }
    }
    if ((stepper.motionComplete () == true) && (limitSwitchFlag == false))        // se a chave CFC_Inicio nunca for acionada
    {
      lcd.setCursor(1, 3);                                                        // LCD coluna 1 e linha 3
      lcd.print("CFC Inicio not OK");                                             // mostra no LCD
    }
  }
  else                                                                            // se a chave CFC_Inicio estiver acionada
  {
    stepper.setCurrentPositionInMillimeters (0);                                  // zera a posição em mm
    limitSwitchFlag = true;                                                       // muda o estado do flag
    lcd.setCursor(3, 3);                                                          // LCD coluna 3 e linha 3
    lcd.print("CFC Inicio OK    ");                                               // mostra no LCD
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void motorPressionaAmbu ()                                                                  // movimenta a aba de pressão do Ambu
{
  ativa_Driver_Motor();                                                                     // ativa driver do motor
  stepper.setCurrentPositionInMillimeters (0);                                              // zera a posição em milimetros
  bool stopFlag = false;

  stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg);                           // configura velocidade do motor em mm/segundo
  //stepper.setAccelerationInMillimetersPerSecondPerSecond(velocidade_mm_por_seg * 3);        // configura aceleraçao do motor - mm/seg 2
  stepper.setAccelerationInMillimetersPerSecondPerSecond (480);
  stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );                    // maxima distancia percorrida em milimetros = 100 (4 voltas)

  if ((digitalRead(CFC_Fim) == HIGH) && (sentidoAmbu == -1))                                // se chave CFC_Fim não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      /*if (stepper.getCurrentPositionInMillimeters() == 100)                               // se a posição precorrida for igual a 90 mm
        {
        stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg/2);                   // diminua a  velocidade do motor em mm/segundo
        }*/

      if ((digitalRead(CFC_Fim) == LOW) && (stopFlag == false))                             // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                  // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                      // zera a posição em mm
        stopFlag = true;                                                                    // altera o estado do fla
      }
    }
  }

  if ((digitalRead(CFC_Inicio) == HIGH) && (sentidoAmbu == 1))                              // se chave CFC Inicio não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      if ((digitalRead(CFC_Inicio) == LOW) && (stopFlag == false))                          // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                  // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                      // zera a posição em mm
        stopFlag = true;                                                                    // altera o estado do fla
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------
void respiradorAmbu ()                                                    // simulação do movimento do Ambu - Tecla B
{
  percurso_Ambu = 100;                                                    // percurso do movimento da aba em mm
  for (int i = 0; i <= 3; i++)                                            // repete 4 vezes para teste
  {
    //leitura_pressao_Ambu ();                                            // leitura de pressão do Ambu
    sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
    velocidade_mm_por_seg = 134;                                          // INSP = 1,0s
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    delay(10) ;
    //delay(tempo_plato * 1000) ;                                         // atraso do tempo plato em ms (ex:0,25 x 1000) menos ajuste fino
    sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
    velocidade_mm_por_seg = 41;                                           // EXP = 2,5 s
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    //leitura_Encoder_Rotativo ();                                          // faz a leitura do Encoder Rotativo
  }
  digitalWrite(dir_MP, HIGH);
}

//---------------------------------------------------------------------------------------------------------------------------------------
void calibracao_velocidade_motor ()                                     // Calibração de velocidade de INS - Tecla D
{
  percurso_Ambu = 100;                                                  // percurso do movimento da aba em mm
  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 180 ;                                   // INS = 0,9 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 134;                                 // EXP = 1,0
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 134 ;                                   // INS = 1,0 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 73;                                  // EXP = 1,5
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 112 ;                                  // INS = 1,1 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 52;                                  // EXP = 2,0
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 98 ;                                    // INS = 1,2 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 41;                                  // EXP = 2,5
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 88 ;                                     // INS = 1,3
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 34;                                  // EXP = 3,0
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  digitalWrite(dir_MP, HIGH);
}

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Telas do Display LCD -------------------------------------------------------------------------------------

void LCD_Inicial (char versao_SOLAR)    // Tela inicial no display LCD
{
  lcd.setBacklight(255);                                  // apaga backlight
  lcd.home(); lcd.clear();                                // limpa display LCD
  lcd.setCursor(0, 0);                                    // coluna 0 e linha 0
  lcd.print("*    RESPIRADOR    *");                      // mostra no LCD
  lcd.setCursor(0, 1);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(0, 2);                                    // coluna 1 e linha 2
  lcd.print("*   Versao 1.40e   *");                      // mostra no LCD
  lcd.setCursor(0, 3);                                    // coluna 0 e linha 3
  lcd.print("*   Abril - 2020   *");                      // mostra no LCD
  delay (500);                                            // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_inicializa_Respirador ()                         // Print inicializa Ambu no LCD
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(2, 2);                                    // coluna 0 e linha 2
  lcd.print("Inicializa AMBU");                           // mostra no LCD
  delay (500);                                            // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_Parametros ()                             // Tela de configuração dos Parametros (favor não alterar)
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 0 e linha 0
  lcd.print("FREQ.RESP:");                                // mostra no LCD
  lcd.setCursor(11, 0);                                   // coluna 11 e linha 0
  lcd.print(frequencia_respiratoria, 1);                  // mostra no LCD
  lcd.setCursor(16, 0);                                   // coluna 0 e linha 0
  lcd.print("irpm");                                      // mostra no LCD
  lcd.setCursor(0, 1);                                    // coluna 0 e linha 1
  lcd.print("PRESSAO: ");                                 // mostra no LCD
  lcd.setCursor(10, 1);                                   // coluna 10 e linha 1
  lcd.print(valor_pressao_Ambu, 1);                       // mostra no LCD
  lcd.setCursor(15, 1);                                   // coluna 15 e linha 1
  lcd.print("cmH2O");                                     // mostra no LCD
  lcd.setCursor(0, 2);                                    // coluna 10 e linha 2
  lcd.print("INS:");                                      // mostra no LCD
  lcd.setCursor(5, 2);                                    // coluna 10 e linha 2
  lcd.print(tempo_inspiracao, 1);                         // mostra no LCD
  lcd.setCursor(8, 2);                                    // coluna 10 e linha 2
  lcd.print("s");                                         // mostra no LCD
  lcd.setCursor(10, 2);                                   // coluna 10 e linha 2
  lcd.print("I/E: 1:");                                   // mostra no LCD
  lcd.setCursor(17, 2);                                   // coluna 0 e linha 2
  lcd.print(relacao_insp_exp, 1);                         // mostra no LCD
  lcd.setCursor(0, 3);                                    // coluna 0 e linha 3
  lcd.print("Plato:");                                    // mostra no LCD
  lcd.setCursor(7, 3);                                    // coluna 0 e linha 3
  lcd.print(tempo_plato, 2);                              // mostra no LCD
  lcd.setCursor(11, 3);                                   // coluna 0 e linha 2
  lcd.print("s");                                         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_selecao_parametro ()
{
  if (ponteiro_selecao == 1) {
    lcd.setCursor(10, 0); lcd.print(">");         // coluna 10 e linha 0
  }
  if (ponteiro_selecao == 2) {
    lcd.setCursor(10, 0); lcd.print(" ");         // coluna 10 e linha 0
    lcd.setCursor(9, 1); lcd.print(">");          // coluna 9 e linha 1
  }
  if (ponteiro_selecao == 3) {
    lcd.setCursor(9, 1); lcd.print(" ");          // coluna 9 e linha 1
    lcd.setCursor(4, 2); lcd.print(">");          // coluna 4 e linha 2
  }
  if (ponteiro_selecao == 4) {
    lcd.setCursor(4, 2); lcd.print(" ");          // coluna 4 e linha 2
    lcd.setCursor(14, 2); lcd.print(">");         // coluna 14 e linha 2
  }
  if (ponteiro_selecao == 5) {
    lcd.setCursor(14, 2); lcd.print(" ");         // coluna 14 e linha 2
    lcd.setCursor(6, 3); lcd.print(">");          // coluna 6 e linha 3
  }
  if (ponteiro_selecao == 0) {
    lcd.setCursor(6, 3); lcd.print(" ");          // coluna 6 e linha 3
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_Valor_Pressao_Ambu ()        // Mostra valor do sensor de Pressão do Ambu
{
  lcd.setBacklight(255);                     // apaga backlight
  lcd.home(); lcd.clear();                   // limpa display LCD
  lcd.setCursor(3, 2);                       // coluna 3 e linha 2
  lcd.print("Pressao AMBU:");                // mostra no LCD
  lcd.setCursor(4, 3);                       // coluna 4 e linha 3
  lcd.print(valor_pressao_Ambu, 1);          // mostra no LCD
  lcd.setCursor(10, 3);                      // coluna 10 e linha 3
  lcd.print("cm H2O");                       // mostra no LCD
  delay(500);                                // atraso 0,5 seg
}



//  LCD DO SERGIO

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_inicio ()                       // mostra CFC_Início
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("*                  *");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("* CFC Inicio = ON  *");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_fim ()                          // mostra CFC_Fim
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("*                  *");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("*   CFC Fim = ON   *");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
}

// CAUSA E EFEITO


//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Função principal

void loop()
{
  char key = keypad.getKey();            // faz varredura dos botões do teclado
  leitura_Encoder_Rotativo ();           // faz a leitura do Encoder Rotativo
}

// ------------------------- Funções do Teclado ---------------------------------------------

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
        LCD_Mostra_Valor_Pressao_Ambu ();     // linha 467
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
        calibracao_velocidade_motor ();
      }
      //-------------------------------------------------------------------------------
  }
}
