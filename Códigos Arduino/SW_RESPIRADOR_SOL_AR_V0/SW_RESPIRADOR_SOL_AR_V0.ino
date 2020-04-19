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
//   Versao 1.40a
//   DATA: 14/04/20
//   HORA: 17:00    revisão e alterações Gustavo
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
#include <FlexyStepper.h>                // Biblioteca controle do motor de passo
#include <RotaryEncoder.h>               // Biblioteca do Encoder rotativo
#include "config.h"
#include "main.h"
#include "IHM.h"
#include "MotorPasso.h"
#if defined(IHM_RAMPS_128X64)
  #include <U8g2lib.h>
  U8G2_ST7920_128X64_F_SW_SPI lcd(U8G2_R0, /* clock=*/ 23, /* data=*/ 17, /* CS=*/ 16);
#else
  #include <LiquidCrystal_PCF8574.h>       // Biblioteca para LCD com I2C
  LiquidCrystal_PCF8574 lcd(LCD_address);  // configura endereço do LCD
#endif

int   frequencia_MP                 = 2000;  // valor de frequencia para ajuste de velocidade
float potenciometro_Plato           = 1000.0;  // leitura do potenciometro do Tempo de Plato
int   valor_pot_MP                  = 1000;  // leitura do potenciometro do MP
float valor_pot_TP                  = 200.0;  // leitura do potenciometro do Tempo de Plato
int   valor_pot_pressao_AMBU        = 0;     // leitura do Pot de pressão do Ambu - A1
float valor_pressao_Ambu            = 20.0;    // leitura da pressão do Ambu
float valor_PPI                     = 0.0;     // leitura da pressão inspiratória de pico em cmH2O
int   frequencia_respiratoria       = 16;    // frequencia respiratoria
float tempo_inspiracao              = 1.0;   // tempo de inspiração
float tempo_expiracao               = 2.5;   // tempo de expiração
float relacao_insp_exp              = 2.5;   // tempo de expiração
float tempo_plato                   = 0.25;  // tempo de plato

float Tempo_seg_Plato_NOK =  300.0;  // Tempo de platô  - convertido em segundos antes do confirma
float Tempo_seg_Plato_OK  =  301.0;  // Tempo de platô  - convertido em segundos após o confirma

float Tins            =    0.0;      // Tempo de inspiração  geralmente 1.0 s
float Texp            =    0.0;      // Tempo de expiração   geralmente 0.5 s
float TPlato          =    0.0;      // Tempo de platô  - estado entre inspiração e expiração - ver uma variação 0s ou ... ou .. 0.7 s

int   cod_causa       =    0;
char  tag_causa       =    0;
char  nome_causa_LN2  =    0;
char  nome_causa_LN3  =    0;
int   cod_efeito      =    0;
char  tag_efeito      =    0;
char  nome_efeito_LN2 =    0;
char  nome_efeito_LN3 =    0;

/* Objeto para controle do teclado matricial */
Keypad keypad = Keypad( makeKeymap(Keys), LinhaPINO, ColunaPINO, linhas, colunas );  // configuração do teclado

/* Objeto para controle do encoder rotativo */
RotaryEncoder encoder(port_Roraty_Encoder1, port_Roraty_Encoder2);              // pin CLK(A)= D6    pin DT(B)= D7  Encoder Rotativo
long newPos = 0;                           // posição do Encoder Rotativo

/* Variáveis para controle da compressão do Ambu */
float velocidade_mm_por_seg = 0.0; // velocidade em mm por segundos
float percurso_Ambu = 100.0;       // percurso do movimento da aba em mm
int StepsPerRevolution = 400;      // passos por volta do motor - modo 1/2 micropasso
int sentidoAmbu = -1;              // sentido do motor = 1 para Ambu, -1 para posição inicial

/* Objeto para controle do motor */
FlexyStepper stepper;                     // cria objeto stepper

void setup() {

  /* Inicia LCD */  
  #if defined(IHM_RAMPS_128X64)
    lcd.begin();
  #else
    lcd.begin(20, 4);     
  #endif  

  /* Tela incialdo IHM */
  LCD_Inicial (versao_SOLAR);            // Tela inicial no display LCD

  /* Inicia comunicação serial para debug pelo monitor serial */
  Serial.begin(9600);
  Serial.println(F("Iniciado"));

  /* Inicia portas de entrada */
  pinMode (Btn_Avanca_MP, INPUT_PULLUP); // Define o pino 21 como entrada digital do botão Avança
  pinMode (Btn_Recua_MP, INPUT_PULLUP);  // Define o pino 20 como entrada digital do botão Recua
  pinMode (CFC_Inicio, INPUT_PULLUP);    // Define o pino 39 como entrada digital da chave CFC_Inicio
  pinMode (CFC_Fim , INPUT_PULLUP);      // Define o pino 41 como entrada digitalda chave CFC_Fim

  /* Inicia portas de saída */
  pinMode (enable_MP, OUTPUT);           // Define o pino 24 como saída digital de habilitação do driver  
  digitalWrite  (enable_MP, HIGH);       // Inicia motor de passo desabilitado
  pinMode (dir_MP, OUTPUT);              // Define o pino 25 como saída digital de sentido de giro
  digitalWrite  (dir_MP, HIGH);          // Define sentido de giro inicial como avançar
  pinMode (pulso_MP, OUTPUT);            // Define 3 pino como saída do trem de pulso
  digitalWrite  (dir_MP, LOW);           // Inicia pulso em nivel baixo

  /* Configurações iniciais para o motor de passo */
  stepper.connectToPins(pulso_MP , dir_MP);              // configura pinos no driver WD2404
  stepper.setStepsPerRevolution (StepsPerRevolution);    // configura passos por volta para o Driver do motor
  stepper.setStepsPerMillimeter(16);                     // meio micropasso - 400 passos/25mm => 16 passos/mm

  /* Inicia teclado matricial */
  keypad.addEventListener(keypadEvent);  // evento para leitura do teclado

  /* Interrupções */
  attachInterrupt(digitalPinToInterrupt(Btn_Avanca_MP), procedimento_avanca_MP, FALLING); // Habilita interrupção no botão de avanço
  attachInterrupt(digitalPinToInterrupt(Btn_Recua_MP) , procedimento_recua_MP, FALLING);  // Habilita interrupção no botão de recuo

  /* Atraso 2 segundos, mostrando tela inicial do IHM */
  delay (1000); 
  Serial.println(F("Setup concluído"));
} 

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Função principal

void loop()
{
  //keypad.getKey();            // faz varredura dos botões do teclado
  leitura_Encoder_Rotativo();           // faz a leitura do Encoder Rotativo
  if (Serial.available()>0){
    executaComando( Serial.read() );  
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Declaração das Funções ----------------------------------------------------------------------
void leitura_Encoder_Rotativo ()
{
  static int pos = 0;                     // lendo os contatos do  encoder
  encoder.tick();
  newPos = encoder.getPosition();

  if (pos != newPos)                      // se a posição foi alterada
  {
    //Serial.println(newPos);            // imprime a nova posição
    pos = newPos;
  }
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
  valor_pressao_Ambu = (float)valor_pot_pressao_AMBU * 0.05859375 ;                // 60 dividido por 1024 => sensor de pressão em até 60 cmH20
  return;
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

//---------------------------------------------------------------------------------------------------------------------------------------
void respiradorAmbu ()                                                    // simulação do movimento do Ambu
{
  for (int i = 0; i <= 10; i++)                                           // repete 10 vezes para teste
  {
    leitura_pressao_Ambu ();                                              // leitura de pressão do Ambu
    sentidoAmbu = 1;                                                      // sentido do motor = 1 para Ambu
    velocidade_mm_por_seg = 125.0;                                        // velocidade do motor em mm por segundos
    percurso_Ambu = 100;                                                  // percurso do movimento da aba em mm
    motorPressionaAmbu();                                                 // movimenta a aba de pressão do Ambu
    delay((unsigned long )(tempo_plato * 1000.0));                                            // atraso do tempo plato em ms (ex:0,25 x 1000)
    sentidoAmbu = -1;                                                     // sentido do motor = -1 oposto ao Ambu
    velocidade_mm_por_seg = velocidade_mm_por_seg / relacao_insp_exp;     // velocidade do motor em mm por segundos
    motorPressionaAmbu();                                                 // movimenta a aba de pressão do Ambu
    //delay(100);                                                         // atraso 100 ms - tempo T2 para T1
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
  else if (cod_efeito =  1) {
    tag_efeito = "AL_VD"   ;
    nome_efeito_LN2 = "    Alarme Verde    ";
    nome_efeito_LN3 = "                    ";
  }
  else if (cod_efeito =  2) {
    tag_efeito = "AL_VM "  ;
    nome_efeito_LN2 = "  Alarme vermelho   ";
    nome_efeito_LN3 = "                    ";
  }
  else if (cod_efeito =  3) {
    tag_efeito = "AL_AM"   ;
    nome_efeito_LN2 = "  Alarme amarelo    ";
    nome_efeito_LN3 = "                    ";
  }
  else if (cod_efeito =  4) {
    tag_efeito = "LG_NBK"  ;
    nome_efeito_LN2 = "   Liga Nobreak     ";
    nome_efeito_LN3 = "                    ";
  }
  else if (cod_efeito =  5) {
    tag_efeito = "AL_LNBK" ;
    nome_efeito_LN2 = "Alarme liga nobreak ";
    nome_efeito_LN3 = "                    ";
  }
  else if (cod_efeito =  6) {
    tag_efeito = "AL_MQB"  ;
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = "   motor quebrado   ";
  }
  else if (cod_efeito =  7) {
    tag_efeito = "AL_OCA"  ;
    nome_efeito_LN2 = " Alarme obstrução na";
    nome_efeito_LN3 = " compressão do ambu ";
  }
  else if (cod_efeito =  8) {
    tag_efeito = "AL_VZAM" ;
    nome_efeito_LN2 = " Alarme vazamento   ";
    nome_efeito_LN3 = "     no Ambu        ";
  }
  else if (cod_efeito =  9) {
    tag_efeito = "AL_D_PB" ;
    nome_efeito_LN2 = " Alarme desconexão  ";
    nome_efeito_LN3 = "   (pressão baixa)  ";
  }
  else if (cod_efeito = 10) {
    tag_efeito = "AL_PB"   ;
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = " pressão baixa      ";
  }
  else if (cod_efeito = 11) {
    tag_efeito = "AL_PP<PL";
    nome_efeito_LN2 = " Alarme pressão de  ";
    nome_efeito_LN3 = "   Ppico < Plimite  ";
  }
  else if (cod_efeito = 12) {
    tag_efeito = "AL_PP>PL";
    nome_efeito_LN2 = " Alarme pressão de  ";
    nome_efeito_LN3 = "   Ppico > Plimite  ";
  }
  else if (cod_efeito = 13) {
    tag_efeito = "AC_VMP"  ;
    nome_efeito_LN2 = "  Acionar válvula   ";
    nome_efeito_LN3 = " mecânica de pressão";
  }
  else if (cod_efeito = 14) {
    tag_efeito = "RED_P<10";
    nome_efeito_LN2 = "  Reduzir pressão   ";
    nome_efeito_LN3 = " P menos de 10 cmH20";
  }
  else if (cod_efeito = 15) {
    tag_efeito = "AL_PEEP" ;
    nome_efeito_LN2 = "    Alarme PEEP     ";
    nome_efeito_LN3 = "  acima do ajustado ";
  }
  else if (cod_efeito = 16) {
    tag_efeito = "AL_PNEG" ;
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = "  pressão negativa  ";
  }
  else if (cod_efeito = 17) {
    tag_efeito = "ST_MAE"  ;
    nome_efeito_LN2 = "Status de movimento ";
    nome_efeito_LN3 = "   ambu => esvaziar ";
  }
  else if (cod_efeito = 18) {
    tag_efeito = "AL_VMABA";
    nome_efeito_LN2 = "Alarme Volume mínimo";
    nome_efeito_LN3 = "  abaixo do ajustado";
  }
  else if (cod_efeito = 19) {
    tag_efeito = "CVF"     ;
    nome_efeito_LN2 = "  Complementar o    ";
    nome_efeito_LN3 = "  volume fornecido  ";
  }
  else if (cod_efeito = 20) {
    tag_efeito = "AL_VMACL";
    nome_efeito_LN2 = " Alarme Volume mim  ";
    nome_efeito_LN3 = "  acima do limite   ";
  }
  else if (cod_efeito = 21) {
    tag_efeito = "AL_VCACL";
    nome_efeito_LN2 = "      Alarme:       ";
    nome_efeito_LN3 = "Vcorrente >  Vlimite";
  }
  else if (cod_efeito = 22) {
    tag_efeito = "AL_C02VZ";
    nome_efeito_LN2 = "AL cilindro O2 vazio";
    nome_efeito_LN3 = "   falência de gás  ";
  }
  else if (cod_efeito = 23) {
    tag_efeito = "AL_FR_AUM";
    nome_efeito_LN2 = "Alarme de frequência";
    nome_efeito_LN3 = "respiratória aument.";
  }
  else if (cod_efeito = 24) {
    tag_efeito = "RED_VMOT";
    nome_efeito_LN2 = "Reduzir a velocidade";
    nome_efeito_LN3 = "     do motor       ";
  }
  else if (cod_efeito = 25) {
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

void executaComando( char c ) {
  Serial.print("tecla ");
  Serial.print(c);
  if (c == '1') {                        // Botão 1 - LIGA
    // LCD_Ligando_MP();
    // delay(3000);
    // procedimento_ligar();
  } else if (c == '2') {                        // Botao 2 - PARAR
    // LCD_Desligando_MP();
    // delay(1000);
    // procedimento_parar();
  } else if (c == '4') {                                   // Botao 4 para Presionar Ambu
    Serial.print(" Inicializa Ambu");
    inicializa_Ambu();                               // posiciona a aba na posição inicial - linha 322
  } else if (c == '6') {                        // Botao 6
    respiradorAmbu ();                   // em andamento - linha 423
  } else if (c == '7') {                        // Botao 7
    leitura_pressao_Ambu () ;             // linha 276
    LCD_Mostra_Valor_Pressao_Ambu ();     // linha 467
  } else if (c == 'A') {                         // Botao A
    sentidoAmbu = 1;                                                   // sentido do motor = 1 para Ambu
    velocidade_mm_por_seg = 50;                                        // velocidade do motor em mm por segundos
    percurso_Ambu = 100;                                               // percurso do movimento da aba em mm
    motorPressionaAmbu();
  } else if (c == 'B') {                         // Botao B
    LCD_mostra_Parametros();             // simulaçao da tela
    respiradorAmbu();                    // simulação do movimento do Ambu - linha 423
  } else if (c == 'C') {                         // Botao C
    LCD_mostra_Parametros();             // linha 471
  } else if (c == 'D') {                        // Botao D
    configura_Parametros_Respirador();   // linha 380 - em andamento
  }
  Serial.println();
}

// ------------------------- Funções do Teclado ---------------------------------------------

void keypadEvent(KeypadEvent key)
{
  if ( keypad.getState() == PRESSED ) {
      executaComando(key);      
  }
}
