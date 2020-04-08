
//============================================================================================================================
//
//                                        PROGRAMA RESPIRADOR SOL AR
//
//           https://github.com/tiagocriaar/AMBU-SOL-AR
//
//           Arduino Mega 2560  - Arduino IDE 1.8.10
//           Motor de Passo NEMA23 15 kgf.cm / Driver de motor WD2404
//           Display LCD 20x4 I2C / teclado com 16 Teclas -  Keypad 4x4
//
//============================================================================================================================
//   VERSÃO: 1.19a
//     DATA: 07/04/20
//     HORA: 24:00
//
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
//   IMPLEMENTAÇÃO:
//
//   INSERÇÃO DO NOVO CÓDIGO DE CONTROLE DO MOTOR DE PASSO E TESTE OPCIONAL FLEXYSTEPPER
//   obs: a Biblioteca SpeedyStepper não permite mudança de velocidade durante o percurso, por isso mudei
//
//   https://github.com/mathertel/LiquidCrystal_PCF8574   Biblioteca do Display
//   http://playground.arduino.cc/Code/Keypad             Biblioteca do teclado
//   https://github.com/Stan-Reifel/FlexyStepper          Biblioteca FlexyStepper
//
//============================================================================================================================

// Bibliotecas
#include  <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>                                  // Biblioteca Wire 
#include <LiquidCrystal_PCF8574.h>                 // Biblioteca para LCD com I2C
#include <Keypad.h>                                // Biblioteca para teclado 
#include <FlexyStepper.h>                          // Biblioteca controle do motor de passo

//------------------------------------------------------------------------------------------------------------------------------------------
// Variáveis globais
int         valor_pot     = 1000;                  // Vai receber a leitura do potenciometro
int         frequencia_MP = 2000;                  // Vai receber valor de frequencia para ajuste de velocidade

LiquidCrystal_PCF8574 lcd(0x3F);                   // Endereço I2C - LCD PCF8574

const byte linhas = 4;                             // linhas do teclado
const byte colunas = 4;                            // colunas do teclado

char Keys[linhas][colunas] =                       // Definicao dos valores dos botoes 1 a 6
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte LinhaPINO[linhas] = {38, 40, 42, 44};         // Linhas do teclado D38 a D44
byte ColunaPINO[colunas] = {46, 48, 50, 52};       // Colunas do teclado D46 a D52

Keypad keypad = Keypad( makeKeymap(Keys), LinhaPINO, ColunaPINO, linhas, colunas );  // configuração do teclado
boolean blink = false;                             // variavel blink

int       StepsPerRevolution = 400;                // passos por volta do motor - modo 1/2 micropasso
int       directionTowardHome = -1;                // sentido do motor = 1 para Ambu, -1 para home
float     speedInStepsPerSecond;                   // velocidade em passos/segundo
bool      limitSwitchFlag;                         // flag para switch de limite

FlexyStepper stepper;                             // cria objeto stepper

//------------------------------------------------------------------------------------------------------------------------------------------
// Rótulos das pinagens

#define     potenciometro_MP A0          // POTENCIÔMETRO VEL MOTOR DE PASSO
#define     P_AMBU           A1          // Leitura de pressão do Ambu
#define     P_paciente       A2          // Leitura de pressão do Paciente
#define     P_vacuo          A3          // Leitura de vácuo

// TX0                        0 // FUTURO
// RX0                        1 // FUTURO
// RS485                      2 // FUTURO

#define     pulso_MP          3          // Trem de pulso pra ajuste de velocidade (pino 3)

// RS485                     14 // FUTURO
// RS485                     15 // FUTURO
//                                          veja a explicação sobre essas alterações nas funções
#define     Btn_Liga_MP      18          // Botão para ligar o motor de passo (pino 18) Alterei essa porta 
#define     Btn_Para_MP      19          // Botão de parada do motor de passo (pino 19) Alterei essa porta 
// DISPLAY SDA               20          // DISPLAY SDA
// DISPLAY SCL               21          // DISPLAY SCL
#define     Btn_Avanca_MP    24          // Botão de avanço do motor de passo (pino 24)
#define     Btn_Recua_MP     25          // Botão de recuo do motor de passo (pino 25)
#define     enable_MP        26          // Habilita o driver do motor de passo (pino 26)
#define     dir_MP           27          // Define a direçao do motor de passo (pino 27)
#define     Falha_E          28          // Sinal de falha de energia (ligar nobreak)

#define     TESTE_G          29          // Botão de teste geral
#define     R1_GER           30          // Relé Geral
#define     R2_AVAN          31          // Relé de Avanço do motor de passo
#define     R3_RET           32          // Relé de retorno do motor de passo
#define     R4_V_AMBU        33          // Relé de Valvula do Ambu
#define     R5_V_02          34          // Relé de oxigênio
#define     R6_V_AR          35          // Relé de Ar 
#define     R7_BYPASS        36          // Relé de By-pass

// TECLADO  LIN1(D38)        38
// TECLADO  LIN2(D40)        40
// TECLADO  LIN3(D42)        42
// TECLADO  LIN4(D44)        44
// TECLADO  COL1(D46)        46
// TECLADO  COL2(D48)        48
// TECLADO  COL3(D50)        50
// TECLADO  COL4(D52)        52

#define     CFC_Inicio       39          // Chave fim de curso início (home position) 
#define     CFC_Fim          41          // Chave fim de curso fim (limte pressão do AMBU) 

//------------------------------------------------------------------------------------------------------------------------------------------
// Declaração das funções]

void procedimento_avanca_MP();                                                                             // Avançar o fuso
void procedimento_recua_MP();                                                                              // Recua o fuso
void procedimento_parar();                                                                                 // Para o fuso
void procedimento_ligar();                                                                                 // Liga o motor
void procedimento_ler_pot();                                                                               // Lê o potenciometro de ajuste de velocidade
void procedimento_tag_causa(int  cod_causa,  char tag_causa,  char nome_causa_LN2,  char nome_causa_LN3);  // Lê a tipo de causa
void procedimento_tag_efeito(int cod_efeito, char tag_efeito, char nome_efeito_LN2, char nome_efeito_LN3); // Lê o tipo de defeito
void procedimento_mostrar_causas_na_tela_do_micro(int cod_causa);                                          // mostrar causas na tela do micro


//------------------------------------------------------------------------------------------------------------------------------------------
// Configuração do Hardware

void setup()
{
  Serial.begin(9600);                          // Habilita a comunicação com a velocidade de 9600 bits por segundo
  pinMode (potenciometro_MP, INPUT);           // Define o pino A0 como entrada analógica do potenciometro
  pinMode (Btn_Avanca_MP, INPUT);              // Define o pino 21 como entrada digital do botão Avança
  pinMode (Btn_Recua_MP, INPUT);               // Define o pino 20 como entrada digital do botão Recua
  pinMode (Btn_Para_MP, INPUT);                // Define o pino 19 como entrada digital do botão Parar
  pinMode (Btn_Liga_MP, INPUT);                // Define o pino 18 como entrada digital do botão Recua
  pinMode (enable_MP, OUTPUT);                 // Define o pino 24 como saída digital de habilitação do driver
  pinMode (dir_MP, OUTPUT);                    // Define o pino 25 como saída digital de sentido de giro
  pinMode (pulso_MP, OUTPUT);                  // Define 3 pino como saída do trem de pulso

  digitalWrite  (Btn_Avanca_MP, INPUT_PULLUP); // Ativa pull-up do pino 21 (Avança)
  digitalWrite  (Btn_Recua_MP, INPUT_PULLUP);  // Ativa pull-up do pino 20 (Recua)
  digitalWrite  (Btn_Para_MP, INPUT_PULLUP);   // Ativa pull-up do pino 19 (Parar)
  digitalWrite  (Btn_Liga_MP, INPUT_PULLUP);   // Ativa pull-up do pino 18 (Ligar)

  digitalWrite  (enable_MP, LOW);              // Desabilita driver
  digitalWrite  (dir_MP, HIGH);                // Define sentido de giro inicial como avançar

  stepper.setStepsPerRevolution (StepsPerRevolution);    // configura passos por volta para o Driver do motor
  stepper.connectToPins(pulso_MP , dir_MP);              // configura pinos no driver WD2404

  int error;                                       // variável de erro para o Display LCD

  Wire.begin();                                    // Inicializa interface do LCD
  Wire.beginTransmission(0x3F);                    // incializa interface I2C
  error = Wire.endTransmission();                  // termina transmissão se houver erro

  lcd.begin(20, 4);                                // inicializa Display LCD 20x4
  LCD_Inicial ();                                  // Tela inicial no display LCD

  keypad.addEventListener(keypadEvent);            // evento para leitura do teclado

  //------------------------------------------------------------------------------------------------------------------------------------------
  // Prototipo de interrupções

  // Observação - os pinos D22, D23, D24 e D25 não suportam interrupções usando Arduino IDE
  // Para Arduino MEGA somente esses pinos podem ser usados  - 2, 3, 18, 19, 20, 21
  // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // attachInterrupt(digitalPinToInterrupt(Btn_Avanca_MP), procedimento_avanca_MP, FALLING);   // Habilita interrupção no botão de avanço
  // attachInterrupt(digitalPinToInterrupt(Btn_Recua_MP) , procedimento_recua_MP, FALLING);    // Habilita interrupção no botão de recuo

  attachInterrupt(digitalPinToInterrupt(Btn_Para_MP)  , procedimento_parar, FALLING);       // Habilita interrupção no botão de parada
  attachInterrupt(digitalPinToInterrupt(Btn_Liga_MP)  , procedimento_ligar, FALLING);       // Habilita interrupção no botão para ligar

  Timer1.attachInterrupt(procedimento_ler_pot);                                                // Habilita interrupção no Timer1
  Timer1.initialize(200000);                                                                   // Carrega tempo no timer (em microssegundos)

} //FIM do void setup

//------------------------------------------------------------------------------------------------------------------------------------------
// Função principal
void loop()
{
  //tone(pulso_MP, frequencia_MP);              // Envia trem de pulso para o driver]
  char key = keypad.getKey();                  // faz varredura dos botões do teclado

} // FIM do void loop

//------------------------------------------------------------------------------------------------------------------------------------------
// Descrição das funções
void procedimento_avanca_MP()
{
  digitalWrite(dir_MP, HIGH);                  // Define sentido de giro para avançar fuso
  Serial.println("Fuso avançando");            // Mensagem no monitor: "Fuso avançando"
}
//------------------------------------------------------------------------------------------------------------------------------------------
void procedimento_recua_MP()
{
  digitalWrite(dir_MP, LOW);                   // Define sentido de giro para recuar fuso
  frequencia_MP = frequencia_MP / 2;
  Serial.println("Fuso recuando");             // Mensagem no monitor: "Fuso recuando"
}
//------------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ligar()
{
  digitalWrite(enable_MP, HIGH);               // Habilita driver
  Serial.println("Motor Ligado");             // Mensagem no monitor: "Motor ligado"
}
//------------------------------------------------------------------------------------------------------------------------------------------
void procedimento_parar()
{
  digitalWrite(enable_MP, LOW);              // Desabilita driver
  Serial.println("Motor Parado");             // Mensagem no monitor: "Motor parado"
}
//------------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ler_pot()
{
  valor_pot = analogRead(potenciometro_MP);   // Lê o valor da tensão no potenciometro
  frequencia_MP = map(valor_pot, 0, 1023, 2000, 4000);   // Converte a tensão em frequencia

  //  lixo
  //  frequencia_MP = ((valor_pot + 1000) * 2);   // Converte a tensão em frequencia
  //  if (valor_pot > 1000)                       // Limita a leitura de tensão no potenciometro em 1000
  //  {
  //    valor_pot = 1000;
  //  }

}

// ------------------------------------------------------------------------------------------------------------------------------------------
// Controle do Motor de Passo

void disableWD2404()
{
  digitalWrite(enable_MP, LOW);                // Desativa o driver WD2404 (verifique se o nivel logico esta correto)
  delay (10);                                  // Atraso de 10 milisegundos
}
//-------------------------------------------------------------------------------------------------------------------------------------------
void enableWD2404()
{
  digitalWrite(enable_MP, HIGH);               // Ativa o driver WD2404   (verifique se o nivel logico esta correto)
  delay (10);                                  // Atraso de 10  milisegundos
}
//-------------------------------------------------------------------------------------------------------------------------------------------
//  Comprimento do eixo sem fim = 200 mm
//  Curso para compressão do AMBU = 100 mm
//  Driver WED2404 configurado para 1/2 passo = 400 passos/volta
//  Velocidade 400 passos/segundo corresponde à uma Volta/segundo (altere essa velocidade depois dos testes)
//  Para avançar os 100 mm são necessários 20 voltas no motor ou 20 x 400 passos = 8000 passos
//  Faça testes iniciais do motor sem o mecanismo!

void motorCFCinicio()                                              // posiciona a aba na posição inicial
{
  directionTowardHome = -1;                                        // sentido do motor = -1 para posição inicial
  enableWD2404();                                                  // ativa driver do motor
  stepper.setAccelerationInStepsPerSecondPerSecond(400);           // configura aceleraçao do motor          (tem que ajustar esses valores)
  long maxDistanceToMoveInSteps = 8000;                            // maxima distancia percorrida em passos  (100 mm = 8000 passos - Atenção no teste)

  if (digitalRead(CFC_Inicio) == HIGH)                             // se chave CFC_Inicio não foi acionada
  {
    stepper.setSpeedInStepsPerSecond(400);                                                        // configura velocidade do motor em passos/segundo (tem que ajustar esses valores)
    stepper.setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);     // movimenta motor no sentido para o início

    limitSwitchFlag = false;
    while (!stepper.processMovement())                             // enquanto estiver girando o motor
    {
      if (digitalRead(CFC_Inicio ) == LOW)                         // se a chave CFC_Inicio for acionada
      {
        limitSwitchFlag = true;                                    // muda o estado do flag
        lcd.setCursor(2, 3);                                       // LCD coluna 2 e linha 3
        lcd.print("CFC Inicio OK    ");                            // mostra no LCD
        break;                                                     // para de girar motor
      }
    }
    if (limitSwitchFlag == false)                                  // se a chave CFC_Inicio nunca for acionada
    {
      lcd.setCursor(2, 3);                                         // LCD coluna 0 e linha 0
      lcd.print("CFC Inicio not OK");                              // mostra no LCD
    }
  }
  stepper.setCurrentPositionInSteps(0L);                           // encontrou home - zera a posição em passos
  delay(500);                                                      // atraso 500 ms
}
//-----------------------------------------------------------------------------------------------------------------------------------------
void motorPressionaAmbu ()                                               // movimenta o motor no sentido do Ambu - Faça testes iniciais do motor sem o mecanismo!
{
  bool stopFlag = false;
  enableWD2404();                                                        // ativa driver do motor

  directionTowardHome = 1;                                               // sentido do motor = 1 para Ambu
  stepper.setCurrentPositionInSteps(0);                                  // zera o contador de passos - para o motor
  stepper.setTargetPositionInSteps(8000);                                // objetivo do percurso 100 mm (8000 passos) - Atenção no teste
  stepper.setSpeedInStepsPerSecond(400);                                 // velocidade do motor em passos/segundo  (tem que ajustar esses valores)
  stepper.setAccelerationInStepsPerSecondPerSecond(600);                 // configura aceleraçao do motor          (tem que ajustar esses valores)

  while (!stepper.motionComplete())                                      // enquanto o motor não avançar todo percurso
  {
    stepper.processMovement();                                           // gira o motor

    long currentPosition = stepper.getCurrentPositionInSteps();          // verifica a posição percorrida em passos
    if (currentPosition == 400)                                          // se a posição precorrida for igual a 400 (tem que ajustar esses valores)
      stepper.setSpeedInStepsPerSecond(800);                             // aumente a velocidade (tem que ajustar esses valores)

    if ((digitalRead(CFC_Fim) == LOW) && (stopFlag == false))            // se a chave CFC_Fim for acionada
    {
      stepper.setTargetPositionToStop();
      stopFlag = true;                                                   // altera o estado do flag
    }
  }
  delay (2000) ;                                                         // atraso 500 ms
}

//------------------------------------------------------------------------------------------------------------------------------------------
void procedimento_tag_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3) // Lê o tipo de causa
{
  cod_causa     = 0;
  tag_causa     = "";
  nome_causa_LN2 = "";
  nome_causa_LN3 = "";

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
  cod_efeito     = 0;
  tag_efeito     = "";
  nome_efeito_LN2 = "";
  nome_efeito_LN3 = "";

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
    //   Serial.println("-------------------");
    //  Serial.print  (cod_causa, HEX);
    //Serial.print("\t"); // imprime uma tabulação (TAB)
    //  Serial.println("-------------------");
    delay(1000);
  }
  delay(1000);
}

//---------------------------Telas do Display LCD -------------------- Sugestões de Telas para Display

void LCD_Inicial ()                          // Tela inicial no display LCD
{
  lcd.setBacklight(255);                     // apaga backlight
  lcd.home(); lcd.clear();                   // limpa display LCD
  lcd.setCursor(2, 0);                       // coluna 0 e linha 0
  lcd.print("AMBU Sol e AR");                // mostra no LCD
  lcd.setCursor(2, 1);                       // coluna 1 e linha 1
  lcd.print("RESPIRADOR");                   // mostra no LCD
  lcd.setCursor(2, 2);                       // coluna 1 e linha 1
  lcd.print("Versao 1.0");                   // mostra no LCD
  lcd.setCursor(2, 3);                       // coluna 0 e linha 2
  lcd.print("abril - 2020");                 // mostra no LCD
  delay (2000);                              // atraso 2 segundos
}

void LCD_HomePosition ()                     // Print Home Position no LCD
{
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(2, 0);                       // coluna 0 e linha 0
  lcd.print("AMBU Sol e AR");                // mostra no LCD
  lcd.setCursor(2, 2);                       // coluna 0 e linha 2
  lcd.print("Home Position");                // mostra no LCD
  delay (500);                               // atraso 0,5 segundos
}

void LCD_iniciaMotor ()                      // Print Inicia Motor
{
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("START_1 para iniciar");         // mostra no LCD
  lcd.setCursor(6, 1);                       // coluna 0 e linha 1
  lcd.print("ou");                           // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("STOP_2 para parar");            // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  delay (500);                               // atraso 0,5 segundos
}

void LCD_CFC_inicio ()                       // mostra CFC_Início
{
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("CFC_Início = ON ");             // mostra no LCD
}

void LCD_CFC_fim ()                           // mostra CFC_Fim
{
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("CFC_Fim = ON       ");          // mostra no LCD
}

void LCD_pressionaAmbu ()                    // mostra Pressiona Ambu
{
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("                  ");           // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("Pressiona Ambu    ");           // mostra no LCD
}

void LCD_soltaAmbu ()                        // mostra Solta Ambu
{
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("                  ");           // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("Solta Ambu        ");           // mostra no LCD
}


// ------------------------- Funções do Teclado ----------------------

void keypadEvent(KeypadEvent key)
{
  switch (keypad.getState())                  // verifica se algum botão foi pressionado
  {
    case PRESSED:                             // identifica o botão pressionado

      if (key == '1')                         // Botão 1 - START
      {
        // motorPressionaAmbu ();                // movimenta o motor no sentido do Ambu (precisa correções)
      }

      if (key == '2')                         // Botao 2 - STOP
      {
        // insira funções aqui
      }

      if (key == '3')                         // Botao 3 - Motor gira para a posição inicial
      {
        motorCFCinicio();                     // posiciona a aba na posição inicial
      }

      if (key == '4')                         // Botao 4
      {
        // insira funções aqui
      }

      if (key == '5')                         // Botao 5
      {
        // insira funções aqui
      }

      if (key == '6')                         // Botao 6
      {
        // insira funções aqui
      }
  }
}
