
//============================================================================================================================
//
//                                        PROGRAMA RESPIRADOR SOL AR
//
//============================================================================================================================
//   VERSÃO: 1.17
//     DATA: 07/04/20
//     HORA: 13:00
//
//
//   ANALISTA 
//   DE SISTEMAS:   SÉRGIO GOULART ALVES PEREIRA
//
//   PROGRAMADORES: SÉRGIO GOULART ALVES PEREIRA
//                  FELIPE
//   REVISORES:
//                  GUSTAVO MURTA
//                  SÉRGIO GOULART ALVES PEREIRA
//
//   IMPLEMENTAÇÃO:
//
//   MOSTRAR CAUSAS NA TELA DO MICRO
//
//
//============================================================================================================================

// Bibliotecas
#include  <Arduino.h>
#include <TimerOne.h>

//------------------------------------------------------------------------------------------------------------------------------------------

// Variáveis globais
 int         valor_pot = 1000;                     // Vai receber a leitura do potenciometro
 int         frequencia_MP = 2000;                 // Vai receber valor de frequencia para ajuste de velocidade
//------------------------------------------------------------------------------------------------------------------------------------------
// Rótulos das pinagens

  #define     potenciometro_MP    A0          // Poteciometro para ajuste de velocidade (pino A0)
  #define     pulso_MP            3           // Trem de pulso pra ajuste de velocidade (pino 3)
  #define     Btn_Ligar           18          // Botão para ligar o motor de passo (pino 18)
  #define     Btn_Parar           19          // Botão de parada do motor de passo (pino 19)
  #define     Btn_Recua_MP        20          // Botão de recuo do motor de passo (pino 20)
  #define     Btn_Avanca_MP       21          // Botão de avanço do motor de passo (pino 21)
  #define     enable_MP           24          // Habilita o driver do motor de passo (pino 24)
  #define     dir_MP              25          // Define a direçao do motor de passo (pino 25)

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
  pinMode (Btn_Parar, INPUT);                  // Define o pino 19 como entrada digital do botão Parar
  pinMode (Btn_Ligar, INPUT);                  // Define o pino 18 como entrada digital do botão Recua
  pinMode (enable_MP, OUTPUT);                 // Define o pino 24 como saída digital de habilitação do driver
  pinMode (dir_MP, OUTPUT);                    // Define o pino 25 como saída digital de sentido de giro
  pinMode (pulso_MP, OUTPUT);                  // Define 3 pino como saída do trem de pulso
  
  digitalWrite  (Btn_Avanca_MP, INPUT_PULLUP); // Ativa pull-up do pino 21 (Avança)
  digitalWrite  (Btn_Recua_MP, INPUT_PULLUP);  // Ativa pull-up do pino 20 (Recua)
  digitalWrite  (Btn_Parar, INPUT_PULLUP);     // Ativa pull-up do pino 19 (Parar)
  digitalWrite  (Btn_Ligar, INPUT_PULLUP);     // Ativa pull-up do pino 18 (Ligar)

  digitalWrite  (enable_MP, LOW);              // Desabilita driver
  digitalWrite  (dir_MP, HIGH);                // Define sentido de giro inicial como avançar

//------------------------------------------------------------------------------------------------------------------------------------------
// Prototipo de interrupções  
  attachInterrupt(digitalPinToInterrupt(Btn_Avanca_MP), procedimento_avanca_MP, FALLING);   // Habilita interrupção no botão de avanço
  attachInterrupt(digitalPinToInterrupt(Btn_Recua_MP) , procedimento_recua_MP, FALLING);    // Habilita interrupção no botão de recuo
  attachInterrupt(digitalPinToInterrupt(Btn_Parar)    , procedimento_parar, FALLING);       // Habilita interrupção no botão de parada
  attachInterrupt(digitalPinToInterrupt(Btn_Ligar)    , procedimento_ligar, FALLING);       // Habilita interrupção no botão para ligar
  Timer1.attachInterrupt(procedimento_ler_pot);                                                          // Habilita interrupção no Timer1
  Timer1.initialize(200000);                                                                // Carrega tempo no timer (em microssegundos)

 } //FIM do void setup

//------------------------------------------------------------------------------------------------------------------------------------------
// Função principal
 void loop()
 {
  tone(pulso_MP, frequencia_MP);                // Envia trem de pulso para o driver]

 

  
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
  frequencia_MP = ((valor_pot + 1000) * 2);   // Converte a tensão em frequencia
  
  if (valor_pot > 1000)                       // Limita a leitura de tensão no potenciometro em 1000
  { 
    valor_pot = 1000;
  }
 }  
//------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_tag_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3) // Lê o tipo de causa
 {
  cod_causa     = 0; 
  tag_causa     ="";
  nome_causa_LN2="";
  nome_causa_LN3="";
  
  // Mostra o Tag da causa
  //limite de 20 caracteres                             12345678901234567890                   12345678901234567890 
  if (cod_causa= 0) {tag_causa="SAL"  ; nome_causa_LN2="    SEM ALARMES     "; nome_causa_LN3="                    ";}
  if (cod_causa= 1) {tag_causa="SITN" ; nome_causa_LN2="  Situação normal   "; nome_causa_LN3="                    ";}
  if (cod_causa= 2) {tag_causa="FET"  ; nome_causa_LN2="Falta energia tomada"; nome_causa_LN3="                    ";}
  if (cod_causa= 3) {tag_causa="AEQ"  ; nome_causa_LN2="Apnéia - equipamento"; nome_causa_LN3="                    ";}
  if (cod_causa= 4) {tag_causa="DCR"  ; nome_causa_LN2=" Desconexão circuito"; nome_causa_LN3="   respiratório     ";}
  if (cod_causa= 5) {tag_causa="OCR"  ; nome_causa_LN2=" Obstrução circuito "; nome_causa_LN3="   respiratório     ";}
  if (cod_causa= 6) {tag_causa="DPEEP"; nome_causa_LN2="  Válvula do PEEP   "; nome_causa_LN3="    com defeito     ";}
  if (cod_causa= 7) {tag_causa="PNEG" ; nome_causa_LN2=" Pressão_negativa   "; nome_causa_LN3="Paciente não sedado ";}
  if (cod_causa= 8) {tag_causa="VCAB" ; nome_causa_LN2=" Volume corrente    "; nome_causa_LN3=" abaixo do ajustado ";}
  if (cod_causa= 9) {tag_causa="VMAB" ; nome_causa_LN2="  Volume minuto     "; nome_causa_LN3=" abaixo do ajustado ";}
  if (cod_causa=10) {tag_causa="VMACL"; nome_causa_LN2="  Volume minuto     "; nome_causa_LN3="  acima do limite   ";}
  if (cod_causa=11) {tag_causa="VCAL" ; nome_causa_LN2=" Volume corrente    "; nome_causa_LN3="  acima do limite   ";}
  if (cod_causa=12) {tag_causa="MOTL" ; nome_causa_LN2="   Motor_lento      "; nome_causa_LN3="                    ";}
  if (cod_causa=13) {tag_causa="MOTD" ; nome_causa_LN2=" Motor_disparado    "; nome_causa_LN3="                    ";}
  if (cod_causa=14) {tag_causa="PO2B" ; nome_causa_LN2="  Baixa pressão     "; nome_causa_LN3=" de entrada de 02   ";}
 }
//------------------------------------------------------------------------------------------------------------------------------------------
 void procedimento_tag_efeito(int cod_efeito, char tag_efeito, char nome_efeito_LN2, char nome_efeito_LN3)  // Lê o tipo de defeito 
 {
  cod_efeito     = 0; 
  tag_efeito     ="";
  nome_efeito_LN2="";
  nome_efeito_LN3="";
  
  // Mostra o Tag do efeito
  //limite de 20 caracteres                                    12345678901234567890                    12345678901234567890 

  if (cod_efeito=  0) {tag_efeito="SEM_EF"  ; nome_efeito_LN2="    Sem efeitos     "; nome_efeito_LN3="                    ";}
  if (cod_efeito=  1) {tag_efeito="AL_VD"   ; nome_efeito_LN2="    Alarme Verde    "; nome_efeito_LN3="                    ";}
  if (cod_efeito=  2) {tag_efeito="AL_VM "  ; nome_efeito_LN2="  Alarme vermelho   "; nome_efeito_LN3="                    ";}
  if (cod_efeito=  3) {tag_efeito="AL_AM"   ; nome_efeito_LN2="  Alarme amarelo    "; nome_efeito_LN3="                    ";}
  if (cod_efeito=  4) {tag_efeito="LG_NBK"  ; nome_efeito_LN2="   Liga Nobreak     "; nome_efeito_LN3="                    ";}
  if (cod_efeito=  5) {tag_efeito="AL_LNBK" ; nome_efeito_LN2="Alarme liga nobreak "; nome_efeito_LN3="                    ";}
  if (cod_efeito=  6) {tag_efeito="AL_MQB"  ; nome_efeito_LN2="      Alarme:       "; nome_efeito_LN3="   motor quebrado   ";}
  if (cod_efeito=  7) {tag_efeito="AL_OCA"  ; nome_efeito_LN2=" Alarme obstrução na"; nome_efeito_LN3=" compressão do ambu ";}
  if (cod_efeito=  8) {tag_efeito="AL_VZAM" ; nome_efeito_LN2=" Alarme vazamento   "; nome_efeito_LN3="     no Ambu        ";}
  if (cod_efeito=  9) {tag_efeito="AL_D_PB" ; nome_efeito_LN2=" Alarme desconexão  "; nome_efeito_LN3="   (pressão baixa)  ";}
  if (cod_efeito= 10) {tag_efeito="AL_PB"   ; nome_efeito_LN2="      Alarme:       "; nome_efeito_LN3=" pressão baixa      ";}
  if (cod_efeito= 11) {tag_efeito="AL_PP<PL"; nome_efeito_LN2=" Alarme pressão de  "; nome_efeito_LN3="   Ppico < Plimite  ";}
  if (cod_efeito= 12) {tag_efeito="AL_PP>PL"; nome_efeito_LN2=" Alarme pressão de  "; nome_efeito_LN3="   Ppico > Plimite  ";}
  if (cod_efeito= 13) {tag_efeito="AC_VMP"  ; nome_efeito_LN2="  Acionar válvula   "; nome_efeito_LN3=" mecânica de pressão";}
  if (cod_efeito= 14) {tag_efeito="RED_P<10"; nome_efeito_LN2="  Reduzir pressão   "; nome_efeito_LN3=" P menos de 10 cmH20";}
  if (cod_efeito= 15) {tag_efeito="AL_PEEP" ; nome_efeito_LN2="    Alarme PEEP     "; nome_efeito_LN3="  acima do ajustado ";}
  if (cod_efeito= 16) {tag_efeito="AL_PNEG" ; nome_efeito_LN2="      Alarme:       "; nome_efeito_LN3="  pressão negativa  ";}
  if (cod_efeito= 17) {tag_efeito="ST_MAE"  ; nome_efeito_LN2="Status de movimento "; nome_efeito_LN3="   ambu => esvaziar ";}
  if (cod_efeito= 18) {tag_efeito="AL_VMABA"; nome_efeito_LN2="Alarme Volume mínimo"; nome_efeito_LN3="  abaixo do ajustado";}
  if (cod_efeito= 19) {tag_efeito="CVF"     ; nome_efeito_LN2="  Complementar o    "; nome_efeito_LN3="  volume fornecido  ";}
  if (cod_efeito= 20) {tag_efeito="AL_VMACL"; nome_efeito_LN2=" Alarme Volume mim  "; nome_efeito_LN3="  acima do limite   ";}
  if (cod_efeito= 21) {tag_efeito="AL_VCACL"; nome_efeito_LN2="      Alarme:       "; nome_efeito_LN3="Vcorrente >  Vlimite";}
  if (cod_efeito= 22) {tag_efeito="AL_C02VZ"; nome_efeito_LN2="AL cilindro O2 vazio"; nome_efeito_LN3="   falência de gás  ";}
  if (cod_efeito= 23) {tag_efeito="AL_FR_AUM";nome_efeito_LN2="Alarme de frequência"; nome_efeito_LN3="respiratória aument.";}
  if (cod_efeito= 24) {tag_efeito="RED_VMOT"; nome_efeito_LN2="Reduzir a velocidade"; nome_efeito_LN3="     do motor       ";}
  if (cod_efeito= 25) {tag_efeito="AUM_VMOT"; nome_efeito_LN2="Aumentar  velocidade"; nome_efeito_LN3="     do motor       ";}
}
//----------------------------------------------------------------------------------------------------------------
void procedimento_mostrar_causas_na_tela_do_micro(int cod_causa)  // mostrar causas na tela do micro 
{
// cod_causa=0;  
// Serial.print("CÓDIGO DA CAUSA = ");
 for (cod_causa = 0; cod_causa < 15; cod_causa=cod_causa++) 
  {
//   Serial.println("-------------------");
 //  Serial.print  (cod_causa, HEX);
   //Serial.print("\t"); // imprime uma tabulação (TAB)
 //  Serial.println("-------------------");
   delay(1000);  
   }
   delay(1000);  
   
}
//----------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------------------


 
