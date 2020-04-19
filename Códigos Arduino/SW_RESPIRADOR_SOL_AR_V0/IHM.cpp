#include "IHM.h"

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Telas do Display LCD -------------------------------------------------------------------------------------

void LCD_limpa() {
  lcd.home(); lcd.clear();                                // limpa display LCD
  #if defined(IHM_RAMPS_128X64)
    lcd.setFont(u8g2_font_profont11_mr);
  #else
    lcd.setBacklight(255);                                  // apaga backlight
  #endif
}

void LCD_mostra(){
  #if defined(IHM_RAMPS_128X64)
    lcd.sendBuffer();
  #endif
}

void LCD_posiciona( byte c, byte l ) {
  #if defined(IHM_RAMPS_128X64)
    lcd.setCursor((c+1)*6, (l+1)*9);                                    // coluna 0 e linha 0
  #else
    lcd.setCursor(c, l);                                    // coluna 0 e linha 0
  #endif
}

// Tela inicial no display LCD
void LCD_Inicial (char versao_SOLAR) {
  LCD_limpa();  
  LCD_posiciona(0, 0);                                    // coluna 0 e linha 0
  lcd.print("*    RESPIRADOR    *");                      // mostra no LCD
  LCD_posiciona(0, 1);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  LCD_posiciona(0, 2);                                    // coluna 1 e linha 2
  lcd.print("*   Versao Zero    *");                      // mostra no LCD
  LCD_posiciona(0, 3);                                    // coluna 0 e linha 3
  lcd.print("*   Abril - 2020   *");                      // mostra no LCD
  LCD_mostra();  
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_inicializa_Respirador ()                         // Print inicializa Ambu no LCD
{
  LCD_limpa();
  LCD_posiciona(0, 0);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  LCD_posiciona(2, 2);                                    // coluna 0 e linha 2
  lcd.print("Inicializa AMBU");                           // mostra no LCD
  LCD_mostra();
  delay (500);                                            // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_Parametros()                             // Tela de configuração dos Parametros (favor não alterar)
{
  LCD_limpa();
  LCD_posiciona(0, 0);                                    // coluna 0 e linha 0
  lcd.print("FREQ.RESP: ");                               // mostra no LCD
  LCD_posiciona(12, 0);                                   // coluna 0 e linha 0
  lcd.print(frequencia_respiratoria);                     // mostra no LCD
  LCD_posiciona(15, 0);                                   // coluna 0 e linha 0
  lcd.print("irpm");                                      // mostra no LCD
  LCD_posiciona(0, 1);                                    // coluna 0 e linha 1
  lcd.print("PRESSAO: ");                                 // mostra no LCD
  LCD_posiciona(10, 1);                                   // coluna 10 e linha 1
  lcd.print(valor_pressao_Ambu, 1);                       // mostra no LCD
  LCD_posiciona(15, 1);                                   // coluna 15 e linha 1
  lcd.print("cmH2O");                                     // mostra no LCD
  LCD_posiciona(0, 2);                                    // coluna 10 e linha 2
  lcd.print("INS:");                                      // mostra no LCD
  LCD_posiciona(5, 2);                                    // coluna 10 e linha 2
  lcd.print(tempo_inspiracao, 1);                         // mostra no LCD
  LCD_posiciona(8, 2);                                    // coluna 10 e linha 2
  lcd.print("s");                                         // mostra no LCD
  LCD_posiciona(10, 2);                                   // coluna 10 e linha 2
  lcd.print("I/E: 1:");                                   // mostra no LCD
  LCD_posiciona(17, 2);                                   // coluna 0 e linha 2
  lcd.print(relacao_insp_exp, 1);                         // mostra no LCD
  LCD_posiciona(0, 3);                                    // coluna 0 e linha 3
  lcd.print("Plato:");                                    // mostra no LCD
  LCD_posiciona(7, 3);                                    // coluna 0 e linha 3
  lcd.print(tempo_plato, 2);                              // mostra no LCD
  LCD_posiciona(11, 3);                                   // coluna 0 e linha 2
  lcd.print("s");                                         // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Teste_MP ()                                     // Tela de testes do MP
{
  LCD_limpa();
  LCD_posiciona(0, 0);                                   // coluna 0 e linha 0
  lcd.print("--------------------");                     // mostra no LCD
  LCD_posiciona(0, 1);                                   // coluna 0 e linha 1
  lcd.print("    1 - Liga        ");                     // mostra no LCD
  LCD_posiciona(0, 2);                                   // coluna 0 e linha 2
  lcd.print("    2 - Parar       ");                     // mostra no LCD
  LCD_posiciona(0, 3);                                   // coluna 0 e linha 3
  lcd.print("--------------------");                     // mostra no LCD
  LCD_mostra();
  delay (500);                                           // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Teste_Potenciometro_TP ()    // Tela de teste do potenciometro do Tempo de Plato
{
  LCD_limpa();
  LCD_posiciona(0, 0);                                  // coluna 0 e linha 0
  lcd.print("      AJUSTE O      ");                    // mostra no LCD
  LCD_posiciona(0, 1);                                  // coluna 0 e linha 1
  lcd.print("   POTENCIOMETRO    ");                    // mostra no LCD
  LCD_posiciona(0, 2);                                  // coluna 0 e linha 2
  lcd.print(" e depois Tecle em A");                    // mostra no LCD
  LCD_posiciona(0, 3);                                  // coluna 0 e linha 3
  lcd.print("   Para aceitar     ");                    // mostra no LCD
  LCD_mostra();
  delay (500);                                          // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_Valor_Potenciometro_TP (float Tempo_seg_Plato_NOK)      // Mostra o valor do potenciômetro para o plato
{
  procedimento_ler_pot_TP(valor_pot_TP, potenciometro_Plato, Tempo_seg_Plato_NOK);
  LCD_limpa();
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("     O tempo       ");          // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print(" de plato e: " + String(Tempo_seg_Plato_NOK));                    // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("     Tecle em A     ");         // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("   Para aceitar     ");         // mostra no LCD
  LCD_mostra();
  delay (500);                               // atraso 0,5 seg
}

// Mostra valor do sensor de Pressão do Ambu
void LCD_Mostra_Valor_Pressao_Ambu () {
  LCD_limpa();
  LCD_posiciona(3, 2);                       // coluna 3 e linha 2
  lcd.print("Pressao AMBU:");                // mostra no LCD
  LCD_posiciona(4, 3);                       // coluna 4 e linha 3
  lcd.print(valor_pressao_Ambu, 1);          // mostra no LCD
  LCD_posiciona(10, 3);                      // coluna 10 e linha 3
  lcd.print("cm H2O");                       // mostra no LCD
  LCD_mostra();
  delay(500);                                // atraso 0,5 seg
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_TECLA_A_Pressionada (float Tempo_seg_Plato_OK)
{
  LCD_Mostra_Valor_Potenciometro_TP (Tempo_seg_Plato_NOK);           // Mostra o valor do potenciômetro para o plato
  delay (500);                               // atraso 0,5 segundos
  LCD_limpa();
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("     A tecla A      ");         // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("  foi pressionada   ");         // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print(" O tempo aceito e: ");          // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("        " + String(Tempo_seg_Plato_OK));                // mostra no LCD
  LCD_mostra();
  delay (500);                               // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_iniciaMotor ()                      // Print Inicia Motor
{
  LCD_limpa();
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("START_1 para iniciar");         // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("STOP_2  para parar ");          // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
  LCD_mostra();
  delay (500);                               // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_inicio ()                       // mostra CFC_Início
{
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("*                  *");         // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("* CFC Inicio = ON  *");         // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_fim ()                          // mostra CFC_Fim
{
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("*                  *");         // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("*   CFC Fim = ON   *");         // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_pressionaAmbu ()                    // mostra Pressiona Ambu
{
  LCD_limpa();
  LCD_posiciona(3, 1);                       // coluna 0 e linha 1
  lcd.print("Pressiona AMBU");               // mostra no LCD
  LCD_posiciona(5, 2);                       // coluna 0 e linha 2
  lcd.print("1,25 seg");                     // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_tempo_Plato ()                     // mostra Tempo de Plato
{
  LCD_posiciona(3, 1);                      // coluna 0 e linha 1
  lcd.print("Aguarda    ");                 // mostra no LCD
  LCD_posiciona(5, 2);                      // coluna 0 e linha 2
  lcd.print("0,25 seg");                    // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_soltaAmbu ()                        // mostra Solta Ambu
{
  LCD_limpa();
  LCD_posiciona(3, 1);                       // coluna 0 e linha 1
  lcd.print("Solta AMBU");                   // mostra no LCD
  LCD_posiciona(5, 2);                       // coluna 0 e linha 2
  lcd.print("1,25 seg");                     // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Ligando_MP()                        // mostra tela ligando motor de passo
{
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("*      Ligando     *");         // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("*  Motor de Passo  *");         // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Desligando_MP()                     // mostra tela desligando motor de passo
{
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("*    Desligando    *");         // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("*  Motor de Passo  *");         // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
  LCD_mostra();
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3)
{
  LCD_posiciona(0, 0);                       // coluna 0 e linha 0
  lcd.print("cod= " + (cod_causa));          // mostra no LCD
  LCD_posiciona(0, 1);                       // coluna 0 e linha 1
  lcd.print("tag= " + (tag_causa));          // mostra no LCD
  LCD_posiciona(0, 2);                       // coluna 0 e linha 2
  lcd.print("" + (nome_causa_LN2));          // mostra no LCD
  LCD_posiciona(0, 3);                       // coluna 0 e linha 3
  lcd.print("" + (nome_causa_LN3));          // mostra no LCD
  LCD_mostra();
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
#if defined(IHM_RAMPS_128X64)
  lcd.begin();
#else
  lcd.begin(20, 4);                           // inicializa Display LCD 20x4
#endif
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
  LCD_mostra();
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

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ler_pot_TP(float valor_pot_TP, float potenciometro_Plato, float Tempo_seg_Plato_NOK) //Lê a entrada do potenciômetro do Tempo de Plato
{
  valor_pot_TP  = analogRead(pot_Plato);                           // Lê o valor da tensão no potenciometro do tempo de platô A4
  //  Tempo_seg_Plato_NOK = map(valor_pot_TP, 0, 1023, 0, 3000);   // Converte a tensão em frequencia
  Tempo_seg_Plato_NOK = valor_pot_TP;
  // Tempo_seg_Plato_NOK=50;
}
