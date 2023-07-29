#include <stdint.h>
#include <stdbool.h>

void Puertos_SPI(void);
void SSI0_Init(void);

void Puertos_SPI(void){
     //-----------Configuracion de puerto asociado al modulo SPI-----------------
     //--------------------------------------------------------------------------

          if ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0){                   // Evalua reloj activado
             SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;                       // Activa reloj del GPIO A
          while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0);}               // Espera a que este listo

             GPIO_PORTA_AFSEL_R |= 0x3C;                                // Seleciona la función alterna de PA[2:5].
             GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0XFF0000FF) | 0x00FFFF00;
             // Configura las terminales de PA a su función de SSI0.
             GPIO_PORTA_AMSEL_R = 0x00;                                 // Deshabilita la función analogica
             GPIO_PORTA_DIR_R = (GPIO_PORTA_DIR_R & ~0x3C) | 0x1C;  // Puerto como salida
             GPIO_PORTA_DEN_R |= 0x3C;                                  // Habilita la función digital del puerto
 }

void SSI0_init (void) {

//-----------Configuracion del propio modulo SPI-----------------------
//----------------------------------------------------------------------
        SYSCTL_RCGCSSI_R = SYSCTL_RCGCSSI_R0;                          // Activa reloj al SSI0
        while ((SYSCTL_PRSSI_R & SYSCTL_PRSSI_R0) == 0);               // Espera a que este listo

        SSI0_CR1_R = 0x00;                                             // Selecciona modo maestro/deshabilita SSI0. (p. 1247)
        SSI0_CPSR_R = 0x02;                                            // preescalador (CPSDVSR) del reloj SSI (p. 1252)
        // configura para Freescale SPI; 16bit; 4 Mbps; SPO = 0; SPH = 0 (p. 1245)
        //Potenciometro hasta 10 MHz
        SSI0_CR0_R = (0x0100 | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16) & ~(SSI_CR0_SPO | SSI_CR0_SPH);

        SSI0_CR1_R |= SSI_CR1_SSE;                                     // Habilita SSI0.

 }
