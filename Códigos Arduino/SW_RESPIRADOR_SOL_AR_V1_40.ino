//============================================================================================================================
//
//           PROGRAMA RESPIRADOR SOL AR
//
//           https://github.com/tiagocriaar/AMBU-SOL-AR
//
//           Arduino Mega 2560  - Arduino IDE 1.8.10
//           Motor de Passo NEMA23 15 kgf.cm / Driver de motor WD2404
//           Display LCD 20x4 I2C / teclado com 16 Teclas -  Keypad 4x4 - Rotary Encoder
//
//============================================================================================================================
//
//   Versao 1.40
//   DATA: 14/04/20
//   HORA: 23:00    revisão e alterações Gustavo
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
//   http://playground.arduino.cc/Code/Keypad                    Biblioteca do teclado
//   https://github.com/Stan-Reifel/FlexyStepper                 Biblioteca FlexyStepper
//   http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx   Biblioteca RotaryEncoder
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

char  versao_SOLAR    =      " Versao 1_40";

int   frequencia_MP                 = 2000;  // valor de frequencia para ajuste de velocidade

float potenciometro_Plato           = 1000;  // leitura do potenciometro do Tempo de Plato
int   valor_pot_MP                  = 1000;  // leitura do potenciometro do MP
float valor_pot_TP                  =  200;  // leitura do potenciometro do Tempo de Plato
int   valor_pot_pressao_AMBU        = 0;     // leitura do Pot de pressão do Ambu - A1
float valor_pressao_Ambu            = 20;    // leitura da pressão do Ambu
float valor_PPI                     = 0;     // leitura da pressão inspiratória de pico em cmH2O
int   frequencia_respiratoria       = 16;    // frequencia respiratoria
float tempo_inspiracao              = 1.0;   // tempo de inspiração
float tempo_expiracao               = 2.5;   // tempo de expiração
float relacao_insp_exp              = 2.5;   // tempo de expiração
float tempo_plato                   = 0.25;  // tempo de plato

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

int LCD_address       = 0x27;                // Endereço I2C - LCD PCF8574 = 0x27 / Gustavo = 0x3F
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

float     percurso_Ambu         = 100;    // percurso do movimento da aba em mm
int       StepsPerRevolution    = 400;    // passos por volta do motor - modo 1/2 micropasso
int       sentidoAmbu   = -1;             // sentido do motor = 1 para Ambu, -1 para posição inicial
float     velocidade_mm_por_seg = 0;      // velocidade em mm por segundos

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

  pinMode (Btn_Avanca_MP, INPUT_PULLUP); // Define o pino 21 como entrada digital do botão Avança
  pinMode (Btn_Recua_MP, INPUT_PULLUP);  // Define o pino 20 como entrada digital do botão Recua
  pinMode (CFC_Inicio, INPUT_PULLUP);    // Define o pino 39 como entrada digital da chave CFC_Inicio
  pinMode (CFC_Fim , INPUT_PULLUP);      // Define o pino 41 como entrada digitalda chave CFC_Fim
  pinMode (enable_MP, OUTPUT);           // Define o pino 24 como saída digital de habilitação do driver
  pinMode (dir_MP, OUTPUT);              // Define o pino 25 como saída digital de sentido de giro
  pinMode (pulso_MP, OUTPUT);            // Define 3 pino como saída do trem de pulso

  digitalWrite  (dir_MP, HIGH);          // Define sentido de giro inicial como avançar

  stepper.setStepsPerRevolution (StepsPerRevolution);    // configura passos por volta para o Driver do motor
  stepper.setStepsPerMillimeter(16);                     // meio micropasso - 400 passos/25mm => 16 passos/mm
  stepper.connectToPins(pulso_MP , dir_MP);              // configura pinos no driver WD2404

  int error;                             // variável de erro para o Display LCD

  Wire.begin();                          // Inicializa interface do LCD
  Wire.beginTransmission(LCD_address);   // incializa interface I2C
  error = Wire.endTransmission();        // termina transmissão se houver erro

  lcd.begin(20, 4);                      // inicializa Display LCD 20x4

  LCD_Inicial (versao_SOLAR);            // Tela inicial no display LCD
  delay (1000);                          // atraso 2 segundos

  keypad.addEventListener(keypadEvent);  // evento para leitura do teclado

  attachInterrupt(digitalPinToInterrupt(Btn_Avanca_MP), procedimento_avanca_MP, FALLING);             // Habilita interrupção no botão de avanço
  attachInterrupt(digitalPinToInterrupt(Btn_Recua_MP) , procedimento_recua_MP, FALLING);              // Habilita interrupção no botão de recuo

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
    // Serial.println(newPos);            // imprime a nova posição
    pos = newPos;
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------
void procedimento_avanca_MP()
{
  digitalWrite(dir_MP, HIGH);            // Define sentido de giro para avançar fuso
  Serial.println("Fuso avançando");      // Mensagem no monitor: "Fuso avançando"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_recua_MP()
{
  digitalWrite(dir_MP, LOW);             // Define sentido de giro para recuar fuso
  frequencia_MP = frequencia_MP;
  Serial.println("Fuso recuando");       // Mensagem no monitor: "Fuso recuando"
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

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ler_pot_TP(float valor_pot_TP, float potenciometro_Plato, float Tempo_seg_Plato_NOK) //Lê a entrada do potenciômetro do Tempo de Plato
{
  valor_pot_TP  = analogRead(pot_Plato);                           // Lê o valor da tensão no potenciometro do tempo de platô A4
  //  Tempo_seg_Plato_NOK = map(valor_pot_TP, 0, 1023, 0, 3000);   // Converte a tensão em frequencia
  Tempo_seg_Plato_NOK = valor_pot_TP;
  // Tempo_seg_Plato_NOK=50;
}

//------------------------------------------------------------------------------------------------------------------------------------
void leitura_pressao_Ambu ()                                                // leitura da pressão inspiratória de pico em cmH2O
{
  valor_pot_pressao_AMBU = analogRead(pot_pressao_AMBU);                    // medição na porta analogica A1 - Leitura de pressão do Ambu
  valor_pressao_Ambu = valor_pot_pressao_AMBU * 0.05859375 ;                // 60 dividido por 1024 => sensor de pressão em até 60 cmH20
  return valor_pressao_Ambu;                                                // retorna o valor da pressão inspiratória de pico
}

//---------------------------------------------------------------------------------------------------------------------------------------
void configura_Parametros_Respirador ()
{
  frequencia_respiratoria = frequencia_respiratoria + newPos;
  Serial.print("Freq. Resp: "); Serial.println (frequencia_respiratoria);
  newPos = 0;


  /*float valor_pressao_Ambu            = 20;    // leitura da pressão do Ambu
    float valor_PPI                     = 0;     // leitura da pressão inspiratória de pico em cmH2O
    int   frequencia_respiratoria       = 16;    // frequencia respiratoria
    float tempo_inspiracao              = 1.0;   // tempo de inspiração
    float tempo_expiracao               = 2.5;   // tempo de expiração
    float relacao_insp_exp              = 2.5;   // tempo de expiração
    float tempo_plato                   = 0.25;  // tempo de plato*/

}

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Controle do Motor de Passo ------------------------------------------------------------------
void desativa_Driver_Motor()
{
  digitalWrite(enable_MP, LOW);           // Desativa o driver WD2404 = HIGH
  delay (10);                             // Atraso de 10 milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void ativa_Driver_Motor()
{
  digitalWrite(enable_MP, HIGH);         // Ativa o driver WD2404 = LOW
  delay (10);                            // Atraso de 10  milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
//  Comprimento do eixo sem fim = 200 mm
//  Curso para compressão do AMBU = 100 mm ou 130 mm
//  FUSO 25 mm - (25 mm de avanço por volta)
//  Driver WED2404 configurado para 1/2 passo = 400 passos/volta
//  Para avançar os 100 mm são necessários 4 voltas no motor ou 4 x 400 passos = 1600 passos
//  Para avançar os 130 mm são necessários 5,2 voltas no motor ou 5,2 x 400 passos = 2080 passos

//--------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void inicializa_Ambu()                                                            // posiciona a aba na posição inicial
{
  LCD_inicializa_Respirador ();                                                   // Print inicializa Ambu no LCD
  sentidoAmbu = -1;                                                               // sentido do motor = -1 para posição inicial
  ativa_Driver_Motor();                                                           // ativa driver do motor
  stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em milimetros
  percurso_Ambu = 100;                                                            // maxima distancia percorrida em milimetros
  bool limitSwitchFlag = false;                                                   // flag para switch de limite

  if (digitalRead(CFC_Inicio) == HIGH)                                            // se chave CFC_Inicio não foi acionada
  {
    stepper.setSpeedInMillimetersPerSecond (50);                                  // configura velocidade do motor em mm/segundo
    stepper.setAccelerationInMillimetersPerSecondPerSecond(200);                  // configura aceleraçao do motor - mm/s2
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

void motorPressionaAmbu ()                                                          // movimenta a aba de pressão do Ambu
{
  ativa_Driver_Motor();                                                             // ativa driver do motor
  stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg);                   // configura velocidade do motor em mm/segundo
  stepper.setAccelerationInMillimetersPerSecondPerSecond(velocidade_mm_por_seg);    // configura aceleraçao do motor - mm/seg 2
  stepper.setCurrentPositionInMillimeters (0);                                      // zera a posição em milimetros
  bool stopFlag = false;
  stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );            // maxima distancia percorrida em milimetros = 100 (4 voltas)

  while (!stepper.motionComplete())                               // enquanto o motor não avançar todo percurso
  {
    stepper.processMovement();                                    // gira o motor

    /*if (stepper.getCurrentPositionInMillimeters() == 100)          // se a posição precorrida for igual a 90 mm
      {
      stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg/2);              // diminua a  velocidade do motor em mm/segundo
      }*/

    if ((digitalRead(CFC_Fim) == LOW) && (sentidoAmbu == 1))      // se a chave CFC_Fim for acionada
    {
      stepper.setTargetPositionToStop();                           // para o motor
      //stepper.setCurrentPositionInMillimeters (0);                 // zera a posição em mm
      stopFlag = true;                                             // altera o estado do flag
      LCD_CFC_fim ();                                              // mostra chave CFC fim ativada
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------
void respiradorAmbu ()                                                    // simulação do movimento do Ambu
{
  for (int i = 0; i <= 10; i++)                                           // repete 10 vezes para teste
  {
    leitura_pressao_Ambu ();                                              // leitura de pressão do Ambu
    sentidoAmbu = 1;                                                      // sentido do motor = 1 para Ambu
    velocidade_mm_por_seg = 125.0;                                        // velocidade do motor em mm por segundos
    percurso_Ambu = 100;                                                  // percurso do movimento da aba em mm
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    delay(tempo_plato * 1000);                                            // atraso do tempo plato em ms
    sentidoAmbu = -1;                                                     // sentido do motor = -1 oposto ao Ambu
    velocidade_mm_por_seg = velocidade_mm_por_seg / relacao_insp_exp;     // velocidade do motor em mm por segundos
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    //delay(100);                                                         // atraso 100 ms - tempo T2 para T1
  }
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
  lcd.print("*   Versao 1.40    *");                      // mostra no LCD
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
  lcd.print("FREQ.RESP: ");                               // mostra no LCD
  lcd.setCursor(12, 0);                                   // coluna 0 e linha 0
  lcd.print(frequencia_respiratoria);                     // mostra no LCD
  lcd.setCursor(15, 0);                                   // coluna 0 e linha 0
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
void LCD_Teste_MP ()                                     // Tela de testes do MP
{
  lcd.setBacklight(255);                                 // luz de fundo LCD
  lcd.home(); lcd.clear();                               // limpa tela LCD
  lcd.setCursor(0, 0);                                   // coluna 0 e linha 0
  lcd.print("--------------------");                     // mostra no LCD
  lcd.setCursor(0, 1);                                   // coluna 0 e linha 1
  lcd.print("    1 - Liga        ");                     // mostra no LCD
  lcd.setCursor(0, 2);                                   // coluna 0 e linha 2
  lcd.print("    2 - Parar       ");                     // mostra no LCD
  lcd.setCursor(0, 3);                                   // coluna 0 e linha 3
  lcd.print("--------------------");                     // mostra no LCD
  delay (500);                                           // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Teste_Potenciometro_TP ()    // Tela de teste do potenciometro do Tempo de Plato
{
  lcd.setBacklight(255);                                // luz de fundo LCD
  lcd.home(); lcd.clear();                              // limpa tela LCD
  lcd.setCursor(0, 0);                                  // coluna 0 e linha 0
  lcd.print("      AJUSTE O      ");                    // mostra no LCD
  lcd.setCursor(0, 1);                                  // coluna 0 e linha 1
  lcd.print("   POTENCIOMETRO    ");                    // mostra no LCD
  lcd.setCursor(0, 2);                                  // coluna 0 e linha 2
  lcd.print(" e depois Tecle em A");                    // mostra no LCD
  lcd.setCursor(0, 3);                                  // coluna 0 e linha 3
  lcd.print("   Para aceitar     ");                    // mostra no LCD
  delay (500);                                          // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_Valor_Potenciometro_TP (float Tempo_seg_Plato_NOK)      // Mostra o valor do potenciômetro para o plato
{
  procedimento_ler_pot_TP(valor_pot_TP, potenciometro_Plato, Tempo_seg_Plato_NOK);

  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("     O tempo       ");          // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print(" de plato e: " + String(Tempo_seg_Plato_NOK));                    // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("     Tecle em A     ");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("   Para aceitar     ");         // mostra no LCD
  delay (500);                               // atraso 0,5 seg
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

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_TECLA_A_Pressionada (float Tempo_seg_Plato_OK)
{
  LCD_Mostra_Valor_Potenciometro_TP (Tempo_seg_Plato_NOK);           // Mostra o valor do potenciômetro para o plato
  delay (500);                               // atraso 0,5 segundos
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("     A tecla A      ");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("  foi pressionada   ");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print(" O tempo aceito e: ");          // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("        " + String(Tempo_seg_Plato_OK));                // mostra no LCD
  delay (500);                               // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_iniciaMotor ()                      // Print Inicia Motor
{
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("START_1 para iniciar");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("STOP_2  para parar ");          // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
  delay (500);                               // atraso 0,5 segundos
}

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

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_pressionaAmbu ()                    // mostra Pressiona Ambu
{
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(3, 1);                       // coluna 0 e linha 1
  lcd.print("Pressiona AMBU");               // mostra no LCD
  lcd.setCursor(5, 2);                       // coluna 0 e linha 2
  lcd.print("1,25 seg");                     // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_tempo_Plato ()                     // mostra Tempo de Plato
{
  lcd.setCursor(3, 1);                      // coluna 0 e linha 1
  lcd.print("Aguarda    ");                 // mostra no LCD
  lcd.setCursor(5, 2);                      // coluna 0 e linha 2
  lcd.print("0,25 seg");                    // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_soltaAmbu ()                        // mostra Solta Ambu
{
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(3, 1);                       // coluna 0 e linha 1
  lcd.print("Solta AMBU");                   // mostra no LCD
  lcd.setCursor(5, 2);                       // coluna 0 e linha 2
  lcd.print("1,25 seg");                     // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Ligando_MP()                        // mostra tela ligando motor de passo
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("*      Ligando     *");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("*  Motor de Passo  *");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Desligando_MP()                     // mostra tela desligando motor de passo
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("*    Desligando    *");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("*  Motor de Passo  *");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3)
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("cod= " + (cod_causa));          // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("tag= " + (tag_causa));          // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("" + (nome_causa_LN2));          // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("" + (nome_causa_LN3));          // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_Lista_Causas(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3)
{
  cod_causa     = 0;
  tag_causa     = "";
  nome_causa_LN2 = "";
  nome_causa_LN3 = "";

  for (cod_causa == 0; cod_causa < 15; cod_causa++)
  {
    procedimento_tag_causa;
    LCD_Mostra_causa(cod_causa, tag_causa, nome_causa_LN2, nome_causa_LN3);
    delay(2000);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------
void Demo_Telas (char versao_SOLAR)           // Demo das telas de testes
{
  lcd.begin(20, 4);                           // inicializa Display LCD 20x4
  LCD_Inicial (versao_SOLAR);                 // Tela inicial no display LCD
  delay (2000);                               // atraso 2 segundos
  LCD_HomePosition ();
  delay (3000);                               // atraso 2 segundos
  LCD_iniciaMotor ();
  delay (3000);                               // atraso 2 segundos
  LCD_CFC_inicio ();
  delay (3000);                               // atraso 2 segundos
  LCD_CFC_fim ();
  delay (3000);                               // atraso 2 segundos
  LCD_pressionaAmbu ();
  delay (3000);                               // atraso 2 segundos
  LCD_soltaAmbu ();
  delay (3000);                               // atraso 2 segundos
  LCD_Teste_MP ();                            // Tela de testes do MP
  delay (3000);                               // atraso 2 segundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
void procedimento_tag_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3) // Lê o tipo de causa
{
  // Mostra o Tag da causa
  //limite de 20 caracteres                             12345678901234567890                   12345678901234567890
  if (cod_causa = 0) {
    tag_causa = "SAL"  ;
    nome_causa_LN2 = "    SEM ALARMES     ";
    nome_causa_LN3 = "                    ";
  }
  if (cod_causa = 1) {
    tag_causa = "SITN" ;
    nome_causa_LN2 = "  Situação normal   ";
    nome_causa_LN3 = "                    ";
  }
  if (cod_causa = 2) {
    tag_causa = "FET"  ;
    nome_causa_LN2 = "Falta energia tomada";
    nome_causa_LN3 = "                    ";
  }
  if (cod_causa = 3) {
    tag_causa = "AEQ"  ;
    nome_causa_LN2 = "Apnéia - equipamento";
    nome_causa_LN3 = "                    ";
  }
  if (cod_causa = 4) {
    tag_causa = "DCR"  ;
    nome_causa_LN2 = " Desconexão circuito";
    nome_causa_LN3 = "   respiratório     ";
  }
  if (cod_causa = 5) {
    tag_causa = "OCR"  ;
    nome_causa_LN2 = " Obstrução circuito ";
    nome_causa_LN3 = "   respiratório     ";
  }
  if (cod_causa = 6) {
    tag_causa = "DPEEP";
    nome_causa_LN2 = "  Válvula do PEEP   ";
    nome_causa_LN3 = "    com defeito     ";
  }
  if (cod_causa = 7) {
    tag_causa = "PNEG" ;
    nome_causa_LN2 = " Pressão_negativa   ";
    nome_causa_LN3 = "Paciente não sedado ";
  }
  if (cod_causa = 8) {
    tag_causa = "VCAB" ;
    nome_causa_LN2 = " Volume corrente    ";
    nome_causa_LN3 = " abaixo do ajustado ";
  }
  if (cod_causa = 9) {
    tag_causa = "VMAB" ;
    nome_causa_LN2 = "  Volume minuto     ";
    nome_causa_LN3 = " abaixo do ajustado ";
  }
  if (cod_causa = 10) {
    tag_causa = "VMACL";
    nome_causa_LN2 = "  Volume minuto     ";
    nome_causa_LN3 = "  acima do limite   ";
  }
  if (cod_causa = 11) {
    tag_causa = "VCAL" ;
    nome_causa_LN2 = " Volume corrente    ";
    nome_causa_LN3 = "  acima do limite   ";
  }
  if (cod_causa = 12) {
    tag_causa = "MOTL" ;
    nome_causa_LN2 = "   Motor_lento      ";
    nome_causa_LN3 = "                    ";
  }
  if (cod_causa = 13) {
    tag_causa = "MOTD" ;
    nome_causa_LN2 = " Motor_disparado    ";
    nome_causa_LN3 = "                    ";
  }
  if (cod_causa = 14) {
    tag_causa = "PO2B" ;
    nome_causa_LN2 = "  Baixa pressão     ";
    nome_causa_LN3 = " de entrada de 02   ";
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------
void procedimento_tag_efeito(int cod_efeito, char tag_efeito, char nome_efeito_LN2, char nome_efeito_LN3)  // Lê o tipo de defeito
{
  // Mostra o Tag do efeito
  //limite de 20 caracteres                                    12345678901234567890                    12345678901234567890

  if (cod_efeito =  0) {
    tag_efeito = "SEM_EF"  ;
    nome_efeito_LN2 = "    Sem efeitos     ";
    nome_efeito_LN3 = "                    ";
  }
  if (cod_efeito =  1) {
    tag_efeito = "AL_VD"   ;
    nome_efeito_LN2 = "    Alarme Verde    ";
    nome_efeito_LN3 = "                    ";
  }
  if (cod_efeito =  2) {
    tag_efeito = "AL_VM "  ;
    nome_efeito_LN2 = "  Alarme vermelho   ";
    nome_efeito_LN3 = "                    ";
  }
  if (cod_efeito =  3) {
    tag_efeito = "AL_AM"   ;
    nome_efeito_LN2 = "  Alarme amarelo    ";
    nome_efeito_LN3 = "                    ";
  }
  if (cod_efeito =  4) {
    tag_efeito = "LG_NBK"  ;
    nome_efeito_LN2 = "   Liga Nobreak     ";
    nome_efeito_LN3 = "                    ";
  }
  if (cod_efeito =  5) {
    tag_efeito = "AL_LNBK" ;
    nome_efeito_LN2 = "Alarme liga nobreak ";
    nome_efeito_LN3 = "                    ";
  }
  if (cod_efeito =  6) {
    tag_efeito = "AL_MQB"  ;
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = "   motor quebrado   ";
  }
  if (cod_efeito =  7) {
    tag_efeito = "AL_OCA"  ;
    nome_efeito_LN2 = " Alarme obstrução na";
    nome_efeito_LN3 = " compressão do ambu ";
  }
  if (cod_efeito =  8) {
    tag_efeito = "AL_VZAM" ;
    nome_efeito_LN2 = " Alarme vazamento   ";
    nome_efeito_LN3 = "     no Ambu        ";
  }
  if (cod_efeito =  9) {
    tag_efeito = "AL_D_PB" ;
    nome_efeito_LN2 = " Alarme desconexão  ";
    nome_efeito_LN3 = "   (pressão baixa)  ";
  }
  if (cod_efeito = 10) {
    tag_efeito = "AL_PB"   ;
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = " pressão baixa      ";
  }
  if (cod_efeito = 11) {
    tag_efeito = "AL_PP<PL";
    nome_efeito_LN2 = " Alarme pressão de  ";
    nome_efeito_LN3 = "   Ppico < Plimite  ";
  }
  if (cod_efeito = 12) {
    tag_efeito = "AL_PP>PL";
    nome_efeito_LN2 = " Alarme pressão de  ";
    nome_efeito_LN3 = "   Ppico > Plimite  ";
  }
  if (cod_efeito = 13) {
    tag_efeito = "AC_VMP"  ;
    nome_efeito_LN2 = "  Acionar válvula   ";
    nome_efeito_LN3 = " mecânica de pressão";
  }
  if (cod_efeito = 14) {
    tag_efeito = "RED_P<10";
    nome_efeito_LN2 = "  Reduzir pressão   ";
    nome_efeito_LN3 = " P menos de 10 cmH20";
  }
  if (cod_efeito = 15) {
    tag_efeito = "AL_PEEP" ;
    nome_efeito_LN2 = "    Alarme PEEP     ";
    nome_efeito_LN3 = "  acima do ajustado ";
  }
  if (cod_efeito = 16) {
    tag_efeito = "AL_PNEG" ;
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = "  pressão negativa  ";
  }
  if (cod_efeito = 17) {
    tag_efeito = "ST_MAE"  ;
    nome_efeito_LN2 = "Status de movimento ";
    nome_efeito_LN3 = "   ambu => esvaziar ";
  }
  if (cod_efeito = 18) {
    tag_efeito = "AL_VMABA";
    nome_efeito_LN2 = "Alarme Volume mínimo";
    nome_efeito_LN3 = "  abaixo do ajustado";
  }
  if (cod_efeito = 19) {
    tag_efeito = "CVF"     ;
    nome_efeito_LN2 = "  Complementar o    ";
    nome_efeito_LN3 = "  volume fornecido  ";
  }
  if (cod_efeito = 20) {
    tag_efeito = "AL_VMACL";
    nome_efeito_LN2 = " Alarme Volume mim  ";
    nome_efeito_LN3 = "  acima do limite   ";
  }
  if (cod_efeito = 21) {
    tag_efeito = "AL_VCACL";
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = "Vcorrente >  Vlimite";
  }
  if (cod_efeito = 22) {
    tag_efeito = "AL_C02VZ";
    nome_efeito_LN2 = "AL cilindro O2 vazio";
    nome_efeito_LN3 = "   falência de gás  ";
  }
  if (cod_efeito = 23) {
    tag_efeito = "AL_FR_AUM";
    nome_efeito_LN2 = "Alarme de frequência";
    nome_efeito_LN3 = "respiratória aument.";
  }
  if (cod_efeito = 24) {
    tag_efeito = "RED_VMOT";
    nome_efeito_LN2 = "Reduzir a velocidade";
    nome_efeito_LN3 = "     do motor       ";
  }
  if (cod_efeito = 25) {
    tag_efeito = "AUM_VMOT";
    nome_efeito_LN2 = "Aumentar  velocidade";
    nome_efeito_LN3 = "     do motor       ";
  }
}

//----------------------------------------------------------------------------------------------------------------
void procedimento_mostrar_causas_na_tela_do_micro(int cod_causa)  // mostrar causas na tela do micro
{
  // cod_causa=0;
  // Serial.print("CÓDIGO DA CAUSA = ");
  for (cod_causa = 0; cod_causa < 15; cod_causa = cod_causa++)
  {
    // Serial.println("-------------------");
    // Serial.print  (cod_causa, HEX);
    // Serial.print("\t"); // imprime uma tabulação (TAB)
    // Serial.println("-------------------");
    delay(1000);
  }
  delay(1000);
}

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
        // LCD_Ligando_MP();
        // delay(3000);
        // procedimento_ligar();
      }
      //-------------------------------------------------------------------------------
      if (key == '2')                         // Botao 2 - PARAR
      {
        Serial.println("tecla 2");
        // LCD_Desligando_MP();
        // delay(1000);
        // procedimento_parar();
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
        //respiradorAmbu ();                   // em andamento
      }
      //-------------------------------------------------------------------------------
      if (key == '7')                         // Botao 7
      {
        Serial.println("tecla 7");
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
        Serial.println("tecla *");
      }
      //-------------------------------------------------------------------------------
      if (key == '#')                         // Botao #
      {
        Serial.println("tecla #");
      }
      //-------------------------------------------------------------------------------
      if (key == 'A')                         // Botao A
      {
        Serial.println("tecla A");
      }
      //-------------------------------------------------------------------------------
      if (key == 'B')                         // Botao B
      {
        Serial.println("tecla B");
        LCD_mostra_Parametros ();             // simulaçao da tela
        // respiradorAmbu ();                    // simulação do movimento do Ambu - em testes ainda 
      }
      //-------------------------------------------------------------------------------
      if (key == 'C')                         // Botao C
      {
        Serial.println("tecla C");
        LCD_mostra_Parametros ();             // linha 471
      }
      //-------------------------------------------------------------------------------
      if (key == 'D')                         // Botao D
      {
        Serial.println("tecla D");
        configura_Parametros_Respirador ();   // linha 380 - em andamento 
      }
      //-------------------------------------------------------------------------------
  }
}
