//======================================================================================================================================================================
//                                                                        PROGRAMA RESPIRADOR SOL AR
//   https://github.com/tiagocriaar/AMBU-SOL-AR
//
//   Arduino Mega 2560  - Arduino IDE 1.8.10
//   Motor de Passo NEMA23 15 kgf.cm / Driver de motor WD2404
//   Display LCD 20x4 I2C / teclado com 16 Teclas -  Keypad 4x4
//
//======================================================================================================================================================================
 long int versao_SOLAR     =  1;
 long int sub_versao_SOLAR = 51;

//   DATA: 20/04/20
//   HORA: 14:07
//
//   ANALISTA
//   DE SISTEMAS:   SÉRGIO GOULART ALVES PEREIRA
//
//   PROGRAMADORES: SÉRGIO GOULART ALVES PEREIRA
//                  FELIPE JONATHAN DIAS TOBIAS
//                  JOSÉ GUSTAVO ABREU MURTA
//
//   REVISORES:     JOSÉ GUSTAVO ABREU MURTA
//                  SÉRGIO GOULART ALVES PEREIRA
//
//   IMPLEMENTAÇÃO:
//
//   REVISADO PELO FELIPE - TIRAR GORDURINHAS DE ANTES DO ENCODER
//
//======================================================================================================================================================================
//   obs: a Biblioteca SpeedyStepper não permite mudança de velocidade durante o percurso, por isso mudei
//
//   https://github.com/mathertel/LiquidCrystal_PCF8574   Biblioteca do Display
//   http://playground.arduino.cc/Code/Keypad             Biblioteca do teclado
//   https://github.com/Stan-Reifel/FlexyStepper          Biblioteca FlexyStepper
//   https://github.com/mathertel/RotaryEncoder                  Biblioteca RotaryEncoder

//--------------------------------------------------------------------------------------------------------------------------------------
//   Comprimento do eixo sem fim               = 200 mm
//   Curso para compressão do AMBU             = 100 mm
//   Curso total                               = 130 mm
//   FUSO                                      =  25 mm - (25 mm de avanço por volta)
//   Driver WED2404 configurado para 1/2 passo = 400 passos/volta
//   Velocidade                                = 400 passos/segundo corresponde à uma Volta/segundo (altere essa velocidade depois dos testes)
//   Para avançar os 100 mm                    => são necessários 20   voltas no motor ou 20   x 400 passos = 8000 passos
//   Para avançar os 100 mm                    => são necessários  4   voltas no motor ou  4   x 400 passos = 1600 passos
//   Para avançar os 130 mm                    => são necessários  5,2 voltas no motor ou  5,2 x 400 passos = 2080 passos
//--------------------------------------------------------------------------------------------------------------------------------------
/*Diagramas dos circuitos :
https://github.com/tiagocriaar/AMBU-SOL-AR/tree/master/Documenta%C3%A7%C3%A3o/Diagramas%20de%20circuitos 

 Códigos Arduino:
https://github.com/tiagocriaar/AMBU-SOL-AR/tree/master/C%C3%B3digos%20Arduino  
*/
//--------------------------------------------------------------------------------------------------------------------------------------

//======================================================================================================================================================================
// Bibliotecas
//======================================================================================================================================================================

  #include <Wire.h>                                                              // Biblioteca Wire 
  #include <Key.h>
  #include <Keypad.h>
  #include <LiquidCrystal_PCF8574.h>                                             // Biblioteca para LCD com I2C
  #include <FlexyStepper.h>                                                      // Biblioteca controle do motor de passo
  #include <RotaryEncoder.h>

//======================================================================================================================================================================
// Variáveis globais
//======================================================================================================================================================================
// Mudança do endereço do LCD
  int LCD_address       = 0x27;                // Endereço I2C - LCD PCF8574 = 0x27 / Gustavo = 0x3F
  LiquidCrystal_PCF8574 lcd(LCD_address);      // configura endereço do LCD

  int valor_Tensao_P_AMBU        = 0;
  int valor_Conver_P_AMBU        = 0;
  int valor_Tensao_P_Paciente    = 0;
  int valor_Conver_P_Paciente    = 0;
  int valor_Tensao_P_Vacuo      =  0;
  int valor_Conver_P_Vacuo      =  0;
  int   valor_pot_pressao_AMBU  =  0;     // leitura do Pot de pressão do Ambu - A1
  float valor_pressao_Ambu      = 20;     // leitura da pressão do Ambu
  float valor_PPI               =  0;     // leitura da pressão inspiratória de pico em cmH2O
  int   frequencia_respiratoria = 16;     // frequencia respiratoria
  float tempo_inspiracao        = 1.0;    // tempo de inspiração
  float tempo_expiracao         = 2.5;    // tempo de expiração
  float relacao_insp_exp        = 2.5;    // tempo de expiração
  float tempo_plato             = 0.25;   // tempo de plato
  int ponteiro_selecao            = 0;    // ponteiro de seleção de parametros
  bool selecao_parametro_irpm   = false;  // parametro irpm
 
 RotaryEncoder encoder(6, 7);              // pin CLK(A)= D6    pin DT(B)= D7  Encoder Rotativo

 int newPos = 0;                           // posição do Encoder Rotativo

 float     percurso_Ambu         = 100;    // percurso do movimento da aba em mm
 int       StepsPerRevolution    = 400;                                         // Passos por volta do motor - modo 1/2 micropasso
 int       sentidoAmbu            = -1;    // sentido do motor = 1 para Ambu, -1 para posição inicial
 float     velocidade_mm_por_seg   = 0;    // velocidade em mm por segundos

 boolean blink = false;                                                          // Variavel blink
 int       directionTowardHome   =  -1;                                          // Sentido do motor = 1 para Ambu, -1 para home
 bool      limitSwitchFlag;                                                      // Flag para switch de limite

 FlexyStepper stepper;                     // cria objeto stepper

 const byte linhas  = 4;                                                         // Linhas do teclado
 const byte colunas = 4;                                                         // Colunas do teclado
 char Keys[linhas][colunas] =                                                    // Definicao dos valores dos botoes 1 a 6
  {
   {'1', '2', '3', 'A'},
   {'4', '5', '6', 'B'},
   {'7', '8', '9', 'C'},
   {'*', '0', '#', 'D'}
  };

 byte LinhaPINO[linhas] =   {38, 40, 42, 44};                                    // Linhas do teclado D38 a D44
 byte ColunaPINO[colunas] = {46, 48, 50, 52};                                    // Colunas do teclado D46 a D52
                                                                                // Configuração do teclado
 Keypad keypad = Keypad( makeKeymap(Keys), LinhaPINO, ColunaPINO, linhas, colunas );




 //?? USO FUTURO SÉRGIO
  float Tempo_ms_Plato_OK = 223;                                                  // Tempo de platô  - estado entre inspiração e expiração
  float Tins            =  1000;                                                  // Tempo de inspiração  geralmente 1.0 s
  float Texp            =   500;                                                  // Tempo de expiração   geralmente 0.5 s
  float TPlato          =   700;                                                  // Tempo de platô  - estado entre inspiração e expiração
                                                                                 // Ver uma variação 0s ou ... ou .. 0.7 s
  int   cod_causa       =     0;
  char  tag_causa       =   " ";
  char  nome_causa_LN2  =   " ";
  char  nome_causa_LN3  =   " ";
  int   cod_efeito      =     0;
  char  tag_efeito      =   " ";
  char  nome_efeito_LN2 =   " ";
  char  nome_efeito_LN3 =   " ";

//======================================================================================================================================================================
// Rótulos das pinagens
//======================================================================================================================================================================

 const int   P_AMBU            =  A1;         // Leitura de pressão do Ambu
 const int   P_paciente        =  A2;         // Leitura de pressão do Paciente
 const int   P_vacuo           =  A3;         // Leitura de vácuo

 // TX0                            0           // FUTURO
 // RX0                            1           // FUTURO
 // RS485                          2           // FUTURO                            // FUTURO
 // LIVRE                          3                                                // Trem de pulso pra ajuste de velocidade (pino 5)
 // LIVRE                          4                                                // Trem de pulso pra ajuste de velocidade (pino 5)
 // LIVRE                          5                                                // Trem de pulso pra ajuste de velocidade (pino 5)
  const int  Rotary_Encoder_A   =  6;
  const int  Rotary_Encoder_B   =  7;
  const int   Btn_enter         =  8;          // botão enter
 
  // LIVRE                         9
  // LIVRE                        10
  // LIVRE                        11
  // LIVRE                        12
 
  const int   led_pino          = 13;          // Pino do LED Arduino Mega
 
  // RS485                        14           // FUTURO                                                // FUTURO
  // RS485                        15           // FUTURO                                                // FUTURO
  // LIVRE                        17
  // LIVRE                        18
  // LIVRE                        19

  // DISPLAY SDA                  20                                                // DISPLAY SDA
  // DISPLAY SCL                  21                                                // DISPLAY SCL

  // LIVRE                        22
  // LIVRE                        23

  const int     Falha_E         = 24;          // Sinal de falha de energia (ligar nobreak)
  const int     TESTE_G         = 25;          // Botão de teste geral

  const int     CFC_Inicio_MP   = 26;          // Chave fim de curso início do MP
  const int     CFC_Fim_MP      = 27;          // Chave fim de curso início do MP

  const int     enable_MP       = 28;          // Habilita o driver do motor de passo (pino 24)
  const int     dir_MP          = 29;          // Define a direçao do motor de passo  (pino 25)
  const int     pul_MP          = 30;          // Define a direçao do motor de passo  (pino 25)
  
  const int     R1_GER          = 31;          // Relé Geral
  const int     R2_AVAN         = 32;          // Relé de Avanço do motor de passo
  const int     R3_RET          = 33;          // Relé de retorno do motor de passo
  const int     R4_V_AMBU       = 34;          // Relé de Valvula do Ambu
  const int     R5_V_02         = 35;          // Relé de oxigênio
  const int     R6_V_AR         = 36;          // Relé de Ar
  const int     R7_BYPASS       = 37;          // Relé de By-pass

  // TECLADO  LIN1(D38)           38
  // TECLADO  LIN2(D40)           40
  // TECLADO  LIN3(D42)           42
  // TECLADO  LIN4(D44)           44
  // TECLADO  COL1(D46)           46
  // TECLADO  COL2(D48)           48
  // TECLADO  COL3(D50)           50
  // TECLADO  COL4(D52)           52

//======================================================================================================================================================================
 // Declaração das funções - VOIDS
//======================================================================================================================================================================
    void LCD_Inicial ();

    void procedimento_SEMAFORO();                                                   // Teste de sinalizadores

//======================================================================================================================================================================
 // VOIDS DO TECLADO
 
    void LCD_Tecla_procedimento_Tecla_0();
    void LCD_Tecla_procedimento_Tecla_1();
    void LCD_Tecla_procedimento_Tecla_2();
    void LCD_Tecla_procedimento_Tecla_3();
    void LCD_Tecla_procedimento_Tecla_4();
    void LCD_Tecla_procedimento_Tecla_5();
    void LCD_Tecla_procedimento_Tecla_6();
    void LCD_Tecla_procedimento_Tecla_7();
    void LCD_Tecla_procedimento_Tecla_8();
    void LCD_Tecla_procedimento_Tecla_9();
    void LCD_Tecla_procedimento_Tecla_A();
    void LCD_Tecla_procedimento_Tecla_B();
    void LCD_Tecla_procedimento_Tecla_C();
    void LCD_Tecla_procedimento_Tecla_D();
    void LCD_Tecla_procedimento_Tecla_asterisco();
    void LCD_Tecla_procedimento_Tecla_jogo_da_velha();
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Declaração voids do motor de passo
 
    void LCD_procedimento_ligar_MP    ();                                        // Liga o motor
    void LCD_procedimento_desligar_MP ();                                        // Para o fuso
    void LCD_procedimento_avanca_MP   ();                                        // Avançar o fuso
    void LCD_procedimento_recua_MP    ();                                        // Recua o fuso

    void LCD_Ligando_MP               ();
    void LCD_Desligando_MP            ();
    
    void LCD_HomePosition             ();
    void LCD_iniciaMotor              ();
    void LCD_CFC_inicio               ();
    void LCD_CFC_fim                  ();

    void LCD_pressionaAmbu            (); 
    void LCD_soltaAmbu                ();
    void Velocidade_variavel_MP       ();                                        // Novo código para variar a velocidade do Motor de Passo

    void procedimento_ler_P_AMBU      ();                                        // Faz a leitura do sensor de pressão do ambu
    void LCD_procedimento_ler_P_AMBU  ();                                        // Mostra a leitura do sensor de pressão do ambu
   
    void procedimento_ler_P_paciente  ();                                        // Faz a leitura do sensor de pressão de ar do paciente
    void LCD_procedimento_ler_P_paciente  ();                                    // Mostra a leitura do sensor de pressão do paciente
    void procedimento_ler_P_vacuo     ();                                        // Faz a leitura do sensor de pressão do vacuo

//======================================================================================================================================================================
 // Declaração voids da versão 40.b do Murta
//======================================================================================================================================================================
    void leitura_Encoder_Rotativo     ();
    void leitura_pressao_Ambu         ();
    void configura_frequencia_respiratoria ();
    void configura_pressao_ambu       ();
    void configura_tempo_inspiracao   ();
    void configura_relacao_IE         ();
    void configura_tempo_plato        ();
    void desativa_Driver_Motor        ();
    void ativa_Driver_Motor           ();
    void inicializa_Ambu              ();
    void LCD_inicializa_Respirador    ();
    void LCD_mostra_Parametros        ();           
    void LCD_mostra_selecao_parametro ();
    void LCD_Mostra_Valor_Pressao_Amb ();
 
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 // Declaração voids do tempo de platô
   
    void LCD_Mostra_Valor_Pot_TP      ();                                        // Mostra o valor do potenciômetro para o plato   
    void LCD_Tecla_A_Aceitar          ();                                        // Tela mensagem Tecla A para aceitar
    void LCD_TECLA_A_Pressionada      ();                                        // Confirma tecla A pressionada para aceitar o valor do Tempo de Plato
    void LCD_Pausa_Plato              ();                                        // Mostra tela pausa de platô

    void procedimento_Tempo_Plato     ();                                        // Função para rodar todo o procedimento do tempo de platÔ

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 // Declaração voids do teclado

    void keypadEvent(KeypadEvent key);
 
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 // Declaração voids causas e efeitos
                                                                                 // Lê a tipo de causa
    void procedimento_tag_causa       ();
                                                                                 // Lê o tipo de defeito
    void procedimento_tag_efeito      ();
    
    void LCD_Mostra_causa             ();

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 // Declaração voids Demos

                                                                                 // Demo Lista de Causas
    void Demo_Lista_Causas();
                                                                                 // Demo das telas de testes MP
    void Demo_Telas_MP    ();                         
    //------------------------------ Tela Principal -------------------------------------------------------------------------------
    void LCD_MENU_0                   ();                                        // Tela Principal
    //------------------------------ Tela de Informações --------------------------------------------------------------------------
    void LCD_MENU_0_1                 ();                                        // Informações 
    //------------------------------- Tela de Contatos ----------------------------------------------------------------------------
    void LCD_MENU_0_1_1               ();                                        // Contatos
    //--------------------------- Tela de Desenvolvedores 1 -----------------------------------------------------------------------
    void LCD_MENU_0_1_2_1             ();                                        // Desenvolvedores
    //--------------------------- Tela de Desenvolvedores 2 -----------------------------------------------------------------------
    void LCD_MENU_0_1_2_2             ();                                        // Desenvolvedores 2 
    //------------------------------ Tela de Operador -----------------------------------------------------------------------------
    void LCD_MENU_0_2                 ();                                        // Operador
    //--------------------------------- Médico(a) ---------------------------------------------------------------------------------
    void LCD_MENU_0_2_1               ();                                        // Médico(a)
    //-------------------------------- Paciente -----------------------------------------------------------------------------------
    void LCD_MENU_0_2_1_1             ();                                        // Paciente
    //-------------------------------- Técnico ------------------------------------------------------------------------------------
    void LCD_MENU_0_2_2               ();                                        // Técnico
    //-------------------------------- Senha OK -----------------------------------------------------------------------------------
    void LCD_MENU_0_2_12_OK   ();                                                // Senha OK
    //-------------------------------- Senha NOK ----------------------------------------------------------------------------------
    void LCD_MENU_0_2_12_NOK  ();                                                // Senha errada - NOK

 
//======================================================================================================================================================================
 void setup()
//======================================================================================================================================================================
 {
  Serial.begin(9600);                                                            // Habilita a comunicação com a velocidade de 9600 bits por segundo

  pinMode (Btn_enter,     INPUT_PULLUP);              // Define o pino D8 como entrada digital do botão Enter
  pinMode (CFC_Inicio_MP, INPUT_PULLUP);              // Define o pino 39 como entrada digital da chave CFC_Inicio
  pinMode (CFC_Fim_MP ,   INPUT_PULLUP);              // Define o pino 41 como entrada digitalda chave CFC_Fim
  pinMode (enable_MP,     OUTPUT);                    // Define o pino 24 como saída digital de habilitação do driver
  pinMode (dir_MP,        OUTPUT);                    // Define o pino 25 como saída digital de sentido de giro
  pinMode (pul_MP,        OUTPUT);                    // Define 3 pino como saída do trem de pulso


  stepper.setStepsPerRevolution (StepsPerRevolution); // configura passos por volta para o Driver do motor
  stepper.setStepsPerMillimeter(16);                  // meio micropasso - 400 passos/25mm => 16 passos/mm
  stepper.connectToPins(pul_MP, dir_MP);              // configura pinos no driver WD2404

  // Display LCD
  int error;                                                                     // Variável de erro para o Display LCD
  Wire.begin                  ();                                                // Inicializa interface do LCD
  Wire.beginTransmission(0x27)  ;                                                // Incializa interface I2C
  error = Wire.endTransmission();                                                // Termina transmissão se houver erro
 
  
  lcd.begin              (20, 4);                                                // Inicializa Display LCD 20x4
  LCD_Inicial                 ();                                                // Tela inicial no display LCD  
  delay (2000);                                                                  // Atraso de 2 segundos
 
  char key = keypad.getKey();                                                    // Faz varredura dos botões do teclado
  keypadEvent(key);

  keypad.addEventListener(keypadEvent);                                          // Evento para leitura do teclado
                                                                                 // Habilita interrupção no Timer1
 } //FIM do void setup

//======================================================================================================================================================================
 void loop()
//======================================================================================================================================================================
 {
  lcd.home(); 
  lcd.clear();                                                                  // Limpa tela LCD
  lcd.begin(20, 4);                                                             // Inicializa Display LCD 20x4

  LCD_iniciaMotor();
  
  char key = keypad.getKey ();                                              // Faz varredura dos botões do teclado
  leitura_Encoder_Rotativo ();           // faz a leitura do Encoder Rotativo
 
 } // FIM do void loop
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

//======================================================================================================================================================================
//
// VOIDS - FUNÇÕES DIVERSAS
//
//======================================================================================================================================================================
  void LCD_procedimento_avanca_MP()
  {
  digitalWrite(dir_MP   , HIGH);                                                 // Define sentido de giro para avançar fuso
   lcd.home(); lcd.clear();                                                      // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("        Fuso        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("     avancando      ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                   // Atraso de 500 milisegundos
  }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_procedimento_recua_MP()
 {
  digitalWrite(dir_MP   , LOW);                                                  // Define sentido de giro para recuar o fuso
 
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                      // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("        Fuso        ");                      // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("      recuando      ");                      // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                      // Mostra no LCD
//  delay (500);                                                                   // Atraso de 500 milisegundos
  }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_procedimento_ligar_MP()
 {
  digitalWrite(enable_MP, LOW);                                                 // Habilita driver do MP

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                     lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Motor        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("       Ligado       ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_procedimento_desligar_MP()
 {
  digitalWrite(enable_MP, HIGH);                                                  // Desabilita driver do MP
 
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                     lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Motor        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("     desligado      ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                   // Atraso de 500 milisegundos
 } 
 
//---------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void motorPressionaAmbu ()                                                                  // movimenta a aba de pressão do Ambu
{
  ativa_Driver_Motor();                                                                     // ativa driver do motor
  stepper.setCurrentPositionInMillimeters (0);                                              // zera a posição em milimetros
  bool stopFlag = false;

  stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg);                           // configura velocidade do motor em mm/segundo
  stepper.setAccelerationInMillimetersPerSecondPerSecond(velocidade_mm_por_seg * 2);        // configura aceleraçao do motor - mm/seg 2
  stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );                    // maxima distancia percorrida em milimetros = 100 (4 voltas)

  if ((digitalRead(CFC_Fim_MP) == HIGH) && (sentidoAmbu == 1))                                 // se chave CFC_Fim não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      /*if (stepper.getCurrentPositionInMillimeters() == 100)                           // se a posição precorrida for igual a 90 mm
        {
        stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg/2);               // diminua a  velocidade do motor em mm/segundo
        }*/

      if ((digitalRead(CFC_Fim_MP) == LOW) && (stopFlag == false))                         // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em mm
        stopFlag = true;                                                                  // altera o estado do fla
      }
    }
  }

  if ((digitalRead(CFC_Inicio_MP) == HIGH) && (sentidoAmbu == -1))                           // se chave CFC Inicio não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      if ((digitalRead(CFC_Inicio_MP) == LOW) && (stopFlag == false))                            // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em mm
        stopFlag = true;                                                                  // altera o estado do fla
      }
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------
void respiradorAmbu ()                                                    // simulação do movimento do Ambu - Tecla B
{
  for (int i = 0; i <= 4; i++)                                           // repete 10 vezes para teste
  {
    leitura_pressao_Ambu ();                                              // leitura de pressão do Ambu
    sentidoAmbu = 1;                                                      // sentido do motor = 1 para Ambu
    velocidade_mm_por_seg = 125.0;                                        // velocidade do motor em mm por segundos
    percurso_Ambu = 100;                                                  // percurso do movimento da aba em mm
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    delay(tempo_plato * 1000);                                            // atraso do tempo plato em ms (ex:0,25 x 1000)
    sentidoAmbu = -1;                                                     // sentido do motor = -1 oposto ao Ambu
    velocidade_mm_por_seg = velocidade_mm_por_seg / relacao_insp_exp;     // velocidade do motor em mm por segundos
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    leitura_Encoder_Rotativo ();                                          // faz a leitura do Encoder Rotativo
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Inicial ()                                                             // Tela inicial no display LCD
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor( 0, 0);                      // Coluna 0 e linha 0
  lcd.print("*    Ventilador    *");                                             // Mostra no LCD
                                      lcd.setCursor( 0, 1);                      // Coluna 1 e linha 1
  lcd.print("*   Ambu Sol e AR  *");                                             // Mostra no LCD
                                      lcd.setCursor( 0, 2);                      // Coluna 1 e linha 2
  lcd.print("*   Versao: ");                                                     // Mostra no LCD
                                      lcd.setCursor(12, 2);                      // Coluna 1 e linha 2
  lcd.print(versao_SOLAR);
                                      lcd.setCursor(13, 2);                      // Coluna 1 e linha 2
  lcd.print(".");                                                                // Mostra no LCD
                                      lcd.setCursor(14, 2);                      // Coluna 1 e linha 2
  lcd.print(sub_versao_SOLAR);
                                      lcd.setCursor(19, 2);                      // Coluna 1 e linha 2
  lcd.print("*");                                                                // Mostra no LCD
                                      lcd.setCursor( 0, 3);                      // Coluna 0 e linha 3
  lcd.print("*   Abril - 2020   *");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_A_Aceitar ()                                                     // Tela mensagem Tecla A para aceitar
 {                                     
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("      Tecle A       ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("    para aceitar    ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_ler_P_AMBU      ()                                            // Faz a leitura do sensor de pressão do ambu
  {
   valor_Tensao_P_AMBU      = analogRead(P_AMBU);                                // Lê o valor pressão em volts
   valor_Conver_P_AMBU      = map(valor_Tensao_P_AMBU, 0, 1023, 2000, 4000);     // Converte o valor de pressão em cmH20
  }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_procedimento_ler_P_AMBU  ()                                            // Mostra a leitura do sensor de pressão do ambu
 {
  procedimento_ler_P_AMBU      ();                                                // Faz a leitura do sensor de pressão do ambu
 
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("====================");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("  :  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("    "+String(valor_Conver_P_AMBU)+" cmH20");                        // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("====================");                                           // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_ler_P_paciente  ()                                            // Faz a leitura do sensor de pressão de ar do paciente
  {
   valor_Tensao_P_Paciente = analogRead(P_paciente);                             // Lê o valor pressão em volts
   valor_Conver_P_Paciente = map(valor_Tensao_P_Paciente, 0, 1023, 2000, 4000);  // Converte o valor de pressão em cmH20
  }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_procedimento_ler_P_paciente  ()                                        // Mostra a leitura do sensor de pressão do paciente
 {
   procedimento_ler_P_paciente  ();                                              // Faz a leitura do sensor de pressão de ar do paciente
 
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("====================");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("  :  ");                                                            // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("    "+String(valor_Conver_P_Paciente)+" cmH20");                    // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("====================");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_ler_P_vacuo     ()                                            // Faz a leitura do sensor de pressão do vacuo
  {
   valor_Tensao_P_Vacuo     = analogRead(P_vacuo);                               // Lê o valor pressão em volts
   valor_Conver_P_Vacuo     = map(valor_Tensao_P_Vacuo, 0, 1023, 2000, 4000);    // Converte o valor de pressão em cmH20
  }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
  void LCD_procedimento_ler_P_vacuo  ()                                          // Mostra a leitura do sensor de pressão do vacuo
 {
   procedimento_ler_P_vacuo  ();                                                 // Faz a leitura do sensor de pressão de ar do paciente
 
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("====================");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("  :  ");                                                            // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("    "+String(valor_Conver_P_Vacuo)+" cmH20");                       // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("====================");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_HomePosition ()                        // Print Home Position no LCD
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("*  Ambu  Sol e AR  *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("*  Home  Position  *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_iniciaMotor              ()
 {
//   A inicializa_Ambu 
//   B motorPressionaAmbu   
//   C respiradorAmbu
//   D LCD_mostra_Parametros  
//   * LCD_mostra_selecao_parametro 
  
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
//  lcd.print("  Escolha a opcao:  ");                                             // Mostra no LCD
  lcd.print("[A] Inicia ambu     ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("[B] M pressiona ambu");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("[C] Respirador Ambu ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("[D] Param.   [#] SEL");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_inicio ()                          // Mostra CFC_Início
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("*                  *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("* CFC_Inicio = ON  *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                   // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_CFC_fim ()                             // Mostra CFC_Fim
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("*                  *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("*   CFC_Fim = ON   *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_pressionaAmbu ()                       // Mostra Pressiona Ambu
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("*    Pressiona     *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("*     o Ambu       *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_soltaAmbu ()                           // Mostra Solta Ambu
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                       lcd.setCursor(0, 0);                      // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                       lcd.setCursor(0, 1);                      // Coluna 0 e linha 1
  lcd.print("*      Solta       *");                                             // Mostra no LCD
                                       lcd.setCursor(0, 2);                      // Coluna 0 e linha 2
  lcd.print("*     o Ambu       *");                                             // Mostra no LCD
                                       lcd.setCursor(0, 3);                      // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Ligando_MP()                           // Mostra tela ligando motor de passo
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                       lcd.setCursor(0, 0);                      // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                       lcd.setCursor(0, 1);                      // Coluna 0 e linha 1
  lcd.print("*     Ligando o    *");                                             // Mostra no LCD
                                       lcd.setCursor(0, 2);                      // Coluna 0 e linha 2
  lcd.print("*  motor de Passo  *");                                             // Mostra no LCD
                                       lcd.setCursor(0, 3);                      // Coluna 0 e linha 3
 lcd.print("--------------------");                                              // Mostra no LCD
 delay (500);                                                                    // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Desligando_MP()                           // mostra tela desligando motor de passo
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("*   Desligando o   *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 2
  lcd.print("*  motor de Passo  *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
 lcd.print("--------------------");                                              // Mostra no LCD
 delay (500);                                                                    // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Pausa_Plato()                           // mostra tela pausa de platô
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                     lcd.setCursor(0, 0);                        // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
                                     lcd.setCursor(0, 1);                        // Coluna 0 e linha 1
  lcd.print("*    Parada do     *");                                             // Mostra no LCD
                                     lcd.setCursor(0, 2);                        // Coluna 0 e linha 2
  lcd.print("*  tempo de plato  *");                                             // Mostra no LCD
                                     lcd.setCursor(0, 3);                        // Coluna 0 e linha 3
 lcd.print("--------------------");                                              // Mostra no LCD
 delay (500);                                                                    // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela Principal -----------------------------

 void LCD_MENU_0 ()                                                              // Tela Principal
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("ACESSO A INFORMACOES");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("*    [1] SOBRE     *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("*    [2] ENTRAR    *");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 1 e linha 3
  lcd.print("* Versao: ");                                                     // Mostra no LCD
                                      lcd.setCursor(11, 3);                      // Coluna 1 e linha 3
  lcd.print(versao_SOLAR);                                                       // Mostra no LCD
                                      lcd.setCursor(12, 3);                      // Coluna 1 e linha 3
  lcd.print(".");                                                                // Mostra no LCD
                                      lcd.setCursor(13, 3);                      // Coluna 1 e linha 3
  lcd.print(sub_versao_SOLAR);                                                   // Mostra no LCD
                                      lcd.setCursor(19, 3);                      // Coluna 1 e linha 3
  lcd.print("- 2020");                                                                // Mostra no LCD
 
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela de Informações -----------------------------
 void LCD_MENU_0_1 ()                                                            // Informações 
 {
  lcd.home(); lcd.clear();                                                       // limpa display LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("Interface  Automacao");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1) ;                      // Coluna 1 e linha 1
  lcd.print("[1] Contato         ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("[2] Desenvolvedores ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela de Contatos -----------------------------
 void LCD_MENU_0_1_1  ()                                                         // Contatos
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("  (31) 3361 - 8503  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("  vendas@interface  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print(" elevadores.com.br  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela de Desenvolvedores 1 -----------------------------
 void LCD_MENU_0_1_2_1 ()                                                       // Desenvolvedores 1
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("  Desenvolvedores:  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("  SERGIO G. A. P.   ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("  FELIPE J. D . T.  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela de Desenvolvedores 2 -----------------------------
 void LCD_MENU_0_1_2_2  ()                                                       // Desenvolvedores 2 
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("  Desenvolvedores:  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("  GUSTAVO G. A. M.  ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("  WELBERT       ");                                                 // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela de Operador -----------------------------
 void LCD_MENU_0_2 ()                                                            // Operador
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("   Tecle a opcao:   ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("[1] Medico(a)       ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("[2] Tecnico(a)      ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------Médico(a)-----------------------------
 void LCD_MENU_0_2_1  ()                                                         // Médico(a)
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print(" Digite ID Medico(a)");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("      ########      ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print(" Tecle [C] confirma ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------Paciente-----------------------------
 void LCD_MENU_0_2_1_1   ()                                                      // Paciente
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print(" Digite ID Paciente ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("      ########      ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print(" Tecle [C] confirma ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Técnico -----------------------------
 void LCD_MENU_0_2_2 ()                                                          // Técnico
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("Digite ID Tecnico(a)");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("      ########      ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print(" Tecle [C] confirma ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print(" * Voltar Proximo # ");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela Senha OK -----------------------------
 void LCD_MENU_0_2_12_OK   ()                                                    // Tela senha OK
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD

  lcd.print("====================");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("       Senha        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("        OK          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("====================");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Tela Senha NOK -----------------------------
 void LCD_MENU_0_2_12_NOK   ()                                                    // Tela senha NOK
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD

  lcd.print("====================");                                             // Mostra no LCD
                                      lcd.setCursor(0, 1);                       // Coluna 1 e linha 1
  lcd.print("    Senha errada    ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 1 e linha 2
  lcd.print("# tentativa(s) em 3 ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("====================");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------








//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Mostra_Causa()
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                     lcd.setCursor(0, 0);                        // Coluna 0 e linha 0
  lcd.print("cod= "+(cod_causa));                                                // Mostra no LCD
                                     lcd.setCursor(0, 1);                        // Coluna 0 e linha 1
  lcd.print("tag= "+(tag_causa));                                                // Mostra no LCD
                                     lcd.setCursor(0, 2);                        // Coluna 0 e linha 2
  lcd.print(""+(nome_causa_LN2));                                                // Mostra no LCD
                                     lcd.setCursor(0, 3);                        // Coluna 0 e linha 3
  lcd.print(""+(nome_causa_LN3));                                                // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_tag_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3) // Lê o tipo de causa
 {
  // Mostra o Tag da causa

  if (cod_causa = 0) {
    tag_causa = "SAL"  ;
    nome_causa_LN2 = "    Sem alarmes     ";
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
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Mostra_Efeito()
 {
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                     lcd.setCursor(0, 0);                        // Coluna 0 e linha 0
  lcd.print("cod= "+(cod_efeito));                                                // Mostra no LCD
                                     lcd.setCursor(0, 1);                        // Coluna 0 e linha 1
  lcd.print("tag= "+(tag_efeito));                                                // Mostra no LCD
                                     lcd.setCursor(0, 2);                        // Coluna 0 e linha 2
  lcd.print(""+(nome_efeito_LN2));                                                // Mostra no LCD
                                     lcd.setCursor(0, 3);                        // Coluna 0 e linha 3
  lcd.print(""+(nome_efeito_LN3));                                                // Mostra no LCD
  delay (500);                                                                   // Atraso 0,5 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_tag_efeito(int cod_efeito, char tag_efeito, char nome_efeito_LN2, char nome_efeito_LN3)  // Lê o tipo de defeito
 {
  // Mostra o Tag do efeito

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
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void Demo_Telas_MP ()                                                           // Demo das telas de testes
 {
  lcd.begin(20, 4);                                                              // Inicializa Display LCD 20x4

  lcd.home(); lcd.clear();                                                       // Limpa display LCD

  LCD_Inicial       ();                                                          // Tela inicial no display LCD
  delay (3000);                                                                  // Atraso de 3 segundos
  LCD_HomePosition  (); 
  delay (3000);                                                                  // Atraso de 3 segundos
  LCD_iniciaMotor   ();
  delay (3000);                                                                  // Atraso de 3 segundos
  LCD_CFC_inicio    (); 
  delay (3000);                                                                  // Atraso de 3 segundos
  LCD_CFC_fim       ();
  delay (3000);                                                                  // Atraso de 3 segundos
  LCD_pressionaAmbu ();
  delay (3000);                                                                  // Atraso de 3 segundos
  LCD_soltaAmbu ();      
  delay (3000);                                                                  // Atraso de 3 segundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void Demo_Lista_Causas() 
 {
  cod_causa      =  0;
  tag_causa      = "";
  nome_causa_LN2 = "";
  nome_causa_LN3 = "";

 for (cod_causa==0; cod_causa<15;cod_causa++)
  {
   procedimento_tag_causa();
   LCD_Mostra_causa      ();
   delay(2000);
  }
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void Velocidade_variavel_MP()                                                   // Novo código para variar a velocidade do Motor de Passo
 {



 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_SEMAFORO()
 {

//  R1_GER                       //SINAL VERDE
//  R2_AVAN                      //SINAL AMARELO
//  R3_RET                       //SINAL VERMELHO

 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_0()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                                                           // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         0          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_1()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         1          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_2()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         2          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_3()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         3          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_4()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         4          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_5()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         5          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_6()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         6          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_7()
  {
 
  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         7          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_8()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         8          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                 // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_9()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         9          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_A()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         A          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                 // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_B()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         B          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_C()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         C          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
//  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_D()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         D          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_asterisco()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         *          ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void LCD_Tecla_procedimento_Tecla_jogo_da_velha()
  {

  lcd.home(); lcd.clear();                                                       // Limpa tela LCD
                                      lcd.setCursor(0, 0);                       // Coluna 0 e linha 0
  lcd.print("--------------------");                                             // Mostra no LCD
  lcd.setCursor(0, 1);                       // Coluna 0 e linha 1
  lcd.print("       Tecla        ");                                             // Mostra no LCD
                                      lcd.setCursor(0, 2);                       // Coluna 0 e linha 3
  lcd.print("         #         ");                                              // Mostra no LCD
                                      lcd.setCursor(0, 3);                       // Coluna 0 e linha 3
  lcd.print("--------------------");                                             // Mostra no LCD
  delay (500);                                                                   // Atraso de 500 milisegundos
 }

//======================================================================================================================================================================
//  NOVAS VOIDS DO MURTA - Versao
//  MP com encoder
//  INSERIDO AQUI EM 17.04.20 - 12 HORAS
//======================================================================================================================================================================
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
//------------------------------------------------------------------------------------------------------------------------------------
void leitura_pressao_Ambu ()                                                // Leitura da pressão inspiratória de pico em cmH2O
{
  valor_pot_pressao_AMBU = analogRead(P_AMBU);                             // medição na porta analogica A1 - Leitura de pressão do Ambu
  valor_pressao_Ambu = valor_pot_pressao_AMBU * 0.05859375 ;                // 60 dividido por 1024 => sensor de pressão em até 60 cmH20
  return valor_pressao_Ambu;                                                // retorna o valor da pressão inspiratória de pico
}

//---------------------------------------------------------------------------------------------------------------------------------------
void configura_frequencia_respiratoria ()
{
  lcd.setCursor(11, 0);                                                   // coluna 11 e linha 0
  lcd.print(">");                                                         // mostra no LCD
  frequencia_respiratoria = 16 + newPos;                                  // frequencia_respiratoria inicial = 16
  if (frequencia_respiratoria < 10 )
  {
    if (frequencia_respiratoria < 5 ) frequencia_respiratoria = 5 ;       // frequencia minima = 5
    lcd.setCursor(12, 0);                                                 // coluna 12 e linha 0
    lcd.print(" ");                                                       // mostra no LCD
    lcd.setCursor(13, 0);                                                 // coluna 13 e linha 0
    lcd.print(frequencia_respiratoria);                                   // mostra no LCD
  }
  else
  {
    if (frequencia_respiratoria > 60) frequencia_respiratoria = 60 ;      // frequencia maxima 60
    lcd.setCursor(12, 0);                                                 // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria);                                   // mostra no LCD
  }
  delay (50);                                                             // atraso 50 ms
}
//---------------------------------------------------------------------------------------------------------------------------------------
void configura_pressao_ambu ()
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
void configura_tempo_inspiracao ()
{
  lcd.setCursor(4, 2);                                                   // coluna 4 e linha 2
  lcd.print(">");                                                        // mostra no LCD
  tempo_inspiracao  = 1.0 + (newPos * 0.1) ;                             // tempo_inspiracao inicial = 1.0

  if (tempo_inspiracao  < 0.9 ) tempo_inspiracao = 0.9 ;                 // tempo_inspiracao minima = 0.9
  if (tempo_inspiracao  > 1.3 ) tempo_inspiracao = 1.3 ;                 // tempo_inspiracao maxima = 1.3

  lcd.setCursor(5, 2);                                                   // coluna 5 e linha 2
  lcd.print(tempo_inspiracao, 1);                                        // mostra no LCD
  delay (50);                                                            // atraso 50 ms
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void configura_relacao_IE ()
{
  lcd.setCursor(14, 2);                                                  // coluna 14 e linha 2
  lcd.print(">");                                                        // mostra no LCD
  relacao_insp_exp  = 2.5 + (newPos * 0.5) ;                             // relacao_insp_exp inicial = 2.5

  if (relacao_insp_exp  < 1 ) relacao_insp_exp = 1 ;                     // relacao_insp_exp minima = 1
  if (relacao_insp_exp  > 3 ) relacao_insp_exp = 3 ;                     // relacao_insp_exp maxima = 3

  lcd.setCursor(17, 2);                                                  // coluna 17 e linha 2
  lcd.print(relacao_insp_exp, 1);                                        // mostra no LCD
  delay (50);                                                            // atraso 50 ms
}
//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_plato ()
{
  lcd.setCursor(6, 3);                                                  // coluna 6 e linha 3
  lcd.print(">");                                                       // mostra no LCD
  tempo_plato  = 0.25 + (newPos * 0.05) ;                               // tempo_plato inicial = 0.25

  if (tempo_plato  < 0.1 ) tempo_plato = 0.1 ;                          // tempo_plato minima = 0.1
  if (tempo_plato  > 0.3 ) tempo_plato = 0.3 ;                          // tempo_plato maxima = 0.3

  lcd.setCursor(7, 3);                                                  // coluna 17 e linha 2
  lcd.print(tempo_plato, 2);                                            // mostra no LCD
  delay (50);                                                           // atraso 50 ms
}
//--------------------------------------------------------------------------------------------------------------------------------------
void desativa_Driver_Motor()
{
  digitalWrite(enable_MP, LOW);           // Desativa o driver
  delay (10);                             // Atraso de 10 milisegundos
}
//---------------------------------------- Controle do Motor de Passo ------------------------------------------------------------------
void ativa_Driver_Motor()
{
  digitalWrite(enable_MP, HIGH);         // Ativa o driver
  delay (10);                            // Atraso de 10  milisegundos
}
//--------------------------------------------------------------------------------------------------------------------------------------
void inicializa_Ambu()                                                            // posiciona a aba na posição inicial
{
  LCD_inicializa_Respirador ();                                                   // Print inicializa Ambu no LCD
  sentidoAmbu = -1;                                                               // sentido do motor = -1 para posição inicial
  percurso_Ambu = 100;                                                            // maxima distancia percorrida em milimetros
  ativa_Driver_Motor();                                                           // ativa driver do motor
  stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em milimetros
  bool limitSwitchFlag = false;                                                   // flag para switch de limite

  if (digitalRead(CFC_Inicio_MP) == HIGH)                                            // se chave CFC_Inicio não foi acionada
  {
    stepper.setSpeedInMillimetersPerSecond (50);                                  // configura velocidade do motor em mm/segundo
    stepper.setAccelerationInMillimetersPerSecondPerSecond(200);                  // configura aceleraçao do motor - mm/s2
    stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );        // maxima distancia percorrida em milimetros = 100 (4 voltas)

    while (!stepper.motionComplete())                                             // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                  // gira o motor

      if (digitalRead(CFC_Inicio_MP) == LOW && (limitSwitchFlag == false))          // se a chave CFC_Inicio for acionada
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
//--------------------------------------------------------------------------------------------------------------------------------------
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
void LCD_mostra_selecao_parametro ()
{
  if (ponteiro_selecao == 1) {
    lcd.setCursor(11, 0); lcd.print(">");         // coluna 11 e linha 0
  }
  if (ponteiro_selecao == 2) {
    lcd.setCursor(11, 0); lcd.print(" ");         // coluna 11 e linha 0
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

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------- Funções do Teclado ----------------------
  void keypadEvent(KeypadEvent key)
  {
  // Murta ver padronização das funções para o teclado
  
  /*
    *  
    *  // Mostra no LCD
 XX
 
 // 0 - // Reservado futuro
// 1 - // Reservado futuro
// 3 - // Reservado futuro
// 4 - // Reservado futuro
// 5 - // Reservado futuro
// 6 - // Reservado futuro
// 7 - // Reservado futuro
// 8 - // Reservado futuro
// 9 - // Reservado futuro

// A - Inicializa_Ambu();     
// B - MotorPressionaAmbu ();
// C - respiradorAmbu ();    
// D - LCD_mostra_Parametros (); 
// * - SELECAO DE PONTEIRO        
// # - RSERVADO PARA O MURTA
  */
   switch (keypad.getState())                                                     // Verifica se algum botão foi pressionado
  { 
    case PRESSED:                                                                // Identifica o botão pressionado

     //------------------------------------------------------------------------------------------------------------------------------------
     if (key == '0')                                                            // Botão 1 - LIGA
       {
        LCD_Tecla_procedimento_Tecla_0; 
       
        
     }
    //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '1')                                                            // Botão 1 - LIGA
       {
        LCD_Tecla_procedimento_Tecla_1; 


       }
   //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '2')                                                            // Botao 2 - PARAR
       {
         LCD_Tecla_procedimento_Tecla_2; 


         
       
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '3')                                                            // Botao 3 
       {
         LCD_Tecla_procedimento_Tecla_3; 

         }
      
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '4')                                                            // Botao 4
       {
         LCD_Tecla_procedimento_Tecla_4; 

       LCD_procedimento_ler_P_AMBU;
      }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '5')                                                            // Botao 5
       {
          LCD_Tecla_procedimento_Tecla_5; 
      LCD_procedimento_ler_P_paciente;
              }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '6')                                                            // Botao 6
       {
            LCD_Tecla_procedimento_Tecla_6; 
      LCD_procedimento_ler_P_vacuo;
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '7')                                                            // Botao 7
       {
          LCD_Tecla_procedimento_Tecla_7; 

         LCD_iniciaMotor;    
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '8')                                                            // Botao 8
       {
           LCD_Tecla_procedimento_Tecla_8; 
      
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '9')                                                            // Botao 9
       {

   
      }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == 'A')                                                            // Botao *
       {
        LCD_Tecla_procedimento_Tecla_A; 

       inicializa_Ambu();                               // posiciona a aba na posição inicial - linha 322

       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == 'B')                                                            // Botao #
       {
        LCD_Tecla_procedimento_Tecla_B; 

        sentidoAmbu = 1;                                                   // sentido do motor = 1 para Ambu
        velocidade_mm_por_seg = 50;                                        // velocidade do motor em mm por segundos
        percurso_Ambu = 100;                                               // percurso do movimento da aba em mm
        motorPressionaAmbu ();
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == 'C')                                                            // Botao A - ACEITA O TEMPO DE PLATÔ
       {
        LCD_Tecla_procedimento_Tecla_C;

        LCD_mostra_Parametros ();             // simulaçao da tela
        respiradorAmbu ();                    // simulação do movimento do Ambu - linha 423
       
 //LCD_TECLA_C_Pressionada ();
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == 'D')                                                            // Botao B
       {
        LCD_Tecla_procedimento_Tecla_D;

        LCD_mostra_Parametros ();
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '*')                                                            // Botao C
       {
        LCD_Tecla_procedimento_Tecla_asterisco;
 
        ponteiro_selecao = ++ ponteiro_selecao;                 // adiciona 1 ao ponteiro de seleçao dos parametros
        if (ponteiro_selecao > 5) ponteiro_selecao = 0;         // de 0 a 5 somente
        LCD_mostra_selecao_parametro ();                        // mostra o parametro selecionado
       }
      //------------------------------------------------------------------------------------------------------------------------------------
      if (key == '#')                                                            // Botao D
       {
        LCD_Tecla_procedimento_Tecla_jogo_da_velha;
      
       }
      //------------------------------------------------------------------------------------------------------------------------------------
  } 
 }
  
