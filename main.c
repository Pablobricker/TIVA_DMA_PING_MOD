#include <stdint.h>
#include "tm4c1294ncpdt.h"


#define CH20 (20*4)
#define CH20ALT (20*4+128)
void Puertos(void);
void ConfigurarUART(void);
void uDMA(void);
void DMA_Start(volatile uint8_t *source,uint8_t *destination, uint32_t count);

int cuenta = 0;

#define A ((volatile uint32_t *)0x40013000);

uint32_t DMA_Memoria[256] __attribute__((aligned(1024)));

uint8_t MiTabla[256];

int B = 0;

int main(void){
    Puertos();          // Función que inicializa los puertos
    ConfigurarUART();   // Configura el UART a 9600 baudios
                        // y habilita la fifo de recepción para solicitudes del DMA
    uDMA();             // Configura el uDMA
    DMA_Start(0x40013000, MiTabla,256);
    while(1){           // Programa principal
        cuenta++;
    }

    return 0;
}
void Puertos(void){
    SYSCTL_RCGCGPIO_R |= 0x04;      // Habilita el Reloj del Puerto C (p.382)
    GPIO_PORTC_AFSEL_R |= 0x30;     // Habilita Función Alterna de los pines PC4 y PC5 (p.770)
    GPIO_PORTC_DEN_R |= 0x30;       // Habilita a los pins PC4 y PC5 como señales digitales (p.781)
    GPIO_PORTC_PCTL_R |= 0x00110000;   // Habilita PC4 y PC5 como UART (p.787)
    GPIO_PORTC_AMSEL_R &= ~0x30;    // Deshabilita la funcionabilidad analogica de PC4 y PC5 (p.786)
}
void ConfigurarUART(void){
    // Se configura un Baude Rate de 9600
    SYSCTL_RCGCUART_R |=  0x00000080; // Habilita el reloj para el UART7
    while((SYSCTL_PRUART_R&0x80) == 0){}; // Se espera a que el reloj se estabilice (p.505)
    UART7_CTL_R &= ~0x00000001; // Se deshabilita el UART (p.1188)
    UART7_IBRD_R = 104;  // IBRD = int(16,000,000 / (16 * 9,600)) = int(104.16666) (p.1184)
    UART7_FBRD_R = 11; // FBRD = round(0.1667 * 64) = 11 (p.1185)
    UART7_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);    // Configura la palabra a 8 bits y habilita el modo FIFO (p.1186)
    UART7_CC_R = (UART7_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;   // Fuente alterna de reloj ALTCLKCFG para el UART (p.1213)
    SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC; // La fuente de reloj alterna es el PIOSC

    UART7_CTL_R &= ~0x00000020;     // El UART se cronometra utilizando el reloj del sistema dividido por 16.

    UART7_DMACTL_R |= UART_DMACTL_RXDMAE;   // Habilitamos el uDMA para la recepción de la FIFO (p.1208)

    UART7_CTL_R |= 0x00000001;      // El UART está habilitado.

}
void uDMA(void){
    SYSCTL_RCGCDMA_R |= 0x01;       // Habilita el reloj del uDMA (p.385)
    UDMA_CFG_R = 0x01;             // Habilita el controlador uDMA, MASTEN (p.712)
    UDMA_CTLBASE_R = (uint32_t)DMA_Memoria;        // Ubicación en Memoria 0x20000000 (p.713)
    UDMA_PRIOSET_R = 0x00;          // Nivel de prioridad predeterminado (p.725)
    UDMA_PRIOCLR_R = 0xFFFFFFFF;    // Borra los registros para prioridad determinada (p.726)
    UDMA_ALTCLR_R = 0x00100000;     // Canal 20 utiliza la estructura de control primaria (p.723)
    UDMA_USEBURSTCLR_R = 0x00100000;// Canal 20 responde a solicitudes unicas y en rafagas (p.718)
    UDMA_REQMASKCLR_R = 0x00100000; // Canal 20 habilidado para solicitar al uDMA transferencias (p.721)

    UDMA_CHMAP2_R = 0x00020000;    // Selecionamos el periferico 2 del Canal 20 (p.731)

}

volatile uint8_t *SourcePt;               // Apuntador de origen
 uint8_t *DestinationPt;  // Apuntador de Dedtino
uint32_t Count;                    // numero de bytes a transmitir
// private function used to reprogram regular channel control structure
void static setRegular(void){
  DMA_Memoria[CH20]  = (uint32_t)SourcePt;           // Primera direccion alternativa
  DMA_Memoria[CH20+1] = (uint32_t)DestinationPt;      // Ultima direccion
  DMA_Memoria[CH20+2] = 0x0C00C003+((Count-1)<<4);         // DMA Channel Control Word (DMACHCTL)
/* DMACHCTL          Bits    Value Description
   DSTINC            31:30   00    incrementa la direccion final en
   DSTSIZE           29:28   00    8-bit Tamaño del dato destino
   SRCINC            27:26   11    8-bit No incrementa la direccion de origen
   SRCSIZE           25:24   00    8-bit Tamaño del dato de origen
   reserved          23:22   0     Reserved
   DSTPROT0          21      0     Proteccion de datos
   reserved          20:19   0     Reservado
   SRCPROT0          18      0     Proteccion de datos
   ARBSIZE           17:14   011     Arbitraje despues de 1 transferencia
   XFERSIZE          13:4  count-1 Numero de items a transferir 1024
   NXTUSEBURST       3       0     No aplica para modo ping-pong
   XFERMODE          2:0     011   Modo Ping-Pong
  */
}
void static setAlternate(void){
  DMA_Memoria[CH20ALT]   = (uint32_t)SourcePt;               // Primera direccion alternativa
  DMA_Memoria[CH20ALT+1] = (uint32_t)DestinationPt;          // Ultima direccion
  DMA_Memoria[CH20ALT+2] = 0x0C00C003+((Count-1)<<4);             // DMA Channel Control Word (DMACHCTL)
  /* DMACHCTL          Bits    Value Description
     DSTINC            31:30   00    No incrementa la direccion final
     DSTSIZE           29:28   00    8-bit Tamaño del dato destino
     SRCINC            27:26   11    8-bit No incrementa la direccion de origen
     SRCSIZE           25:24   00    8-bit Tamaño del dato de origen
     reserved          23:22   0     Reserved
     DSTPROT0          21      0     Proteccion de datos
     reserved          20:19   0     Reservado
     SRCPROT0          18      0     Proteccion de datos
     ARBSIZE           17:14   0     Arbitraje despues de 1 transferencia
     XFERSIZE          13:4  count-1 Numero de items a transferir 1024
     NXTUSEBURST       3       0     No aplica para modo ping-pong
     XFERMODE          2:0     011   Modo Ping-Pong
    */
}

void DMA_Start(volatile uint8_t *source,uint8_t *destination, uint32_t count){
  SourcePt = source;  // Apuntador a la direccion origen
  DestinationPt = destination+count-1;
  Count = count;  // Numero de bytes
  setRegular();     // Canal primario
  setAlternate();   // Canal alternativo
  UDMA_ENASET_R = 0x00100000;     // El canal 20 esta habilitado
}


