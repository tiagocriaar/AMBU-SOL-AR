#include "sensores.h"

//------------------------------------------------------------------------------------------------------------------------------------
void leitura_pressao_Ambu ()                                                // leitura da pressão inspiratória de pico em cmH2O
{
  valor_pot_pressao_AMBU = analogRead(pot_pressao_AMBU);                    // medição na porta analogica A1 - Leitura de pressão do Ambu
  valor_pressao_Ambu = valor_pot_pressao_AMBU * 0.05859375 ;                // 60 dividido por 1024 => sensor de pressão em até 60 cmH20
  return;
}
