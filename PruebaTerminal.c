#include <stdint.h>
#include "tm4c1294ncpdt.h"


#define CH8 (8*4)
#define CH8ALT (8*4+128)        //tiene que estar en la segunda mitad de la tabla DMA_Memoria[]
void Puertos(void);
void ConfigurarUART(void);
void uDMA(void);
void DMA_Start(volatile uint8_t *source,uint8_t *destination, uint32_t count);

int cuenta = 0;

#define A ((volatile uint32_t *)0x4000C000);

uint32_t DMA_Memoria[256] __attribute__((aligned(1024)));

uint8_t MiTabla[256];

int B = 0;

int main(void){
    Puertos();          // Finción que inicializa los puertos
    ConfigurarUART();   // Configura el UART a 9600 baudios y habilita la fifo de recepción y el DMA
    uDMA();             // Configura el uDMA
    DMA_Start(0x4000C000, MiTabla,159);
    //DMA_Start(0x4000C000, MiTabla,256);
    int i;
    while(1){           // Programa principal
        for (i=0;i<1000000;i++);
        cuenta++;
    }

    return 0;
}
void Puertos(void){
    SYSCTL_RCGCGPIO_R |= 0x01;      // Habilita el Reloj del Puerto A
    GPIO_PORTA_AFSEL_R |= 0x03;     // Habilita Función Externa PA0 y PA1
    GPIO_PORTA_DEN_R |= 0x03;       // Habilita entradas o salidas digitales PA0 y PA1
    GPIO_PORTA_PCTL_R |= 0x00000011;   // Habilita PA0 y PA1 como UART
    GPIO_PORTA_AMSEL_R &= ~0x03;    // Deshabilita la funcionabilidad analogica de PA0 y PA1
}
void ConfigurarUART(void){
    // Se configura un Baude Rate de 9600
    SYSCTL_RCGCUART_R |=  0x00000001; // Habilita el reloj para el UART0
    while((SYSCTL_PRUART_R&0x01) == 0){}; // Se espera a que el reloj se estabilice (p.505)
    UART0_CTL_R &= ~0x00000001; // Se deshabilita el UART
    UART0_IBRD_R = 104;  // IBRD = int(16,000,000 / (16 * 9,600)) = int(104.16666)
    UART0_FBRD_R = 11; // FBRD = round(0.1667 * 64) = 11
    UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);    // Configura la palabra a 8 bits y habilita el modo FIFO
    UART0_CC_R = (UART0_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;   // Fuente alterna de reloj ALTCLKCFG para el UART
    SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC; // La fuente de reloj alterna es el PIOSC

    UART0_CTL_R &= ~0x00000020;     // El UART se cronometra utilizando el reloj del sistema dividido por 16.

    UART0_DMACTL_R |= UART_DMACTL_RXDMAE;   // Habilitamos el uDMA para la recepción de la FIFO

    UART0_CTL_R |= 0x00000001;      // El UART está habilitado.

}
void uDMA(void){
    SYSCTL_RCGCDMA_R |= 0x01;       // Habilita el reloj del uDMA
    UDMA_CFG_R = 0x01;             // Habilita el controlador uDMA, MASTEN
    UDMA_CTLBASE_R = (uint32_t)DMA_Memoria;        // Ubicación en Memoria 0x0000;
    UDMA_PRIOSET_R = 0x00;          // Nivel de prioridad predeterminado
    UDMA_PRIOCLR_R = 0xFFFFFFFF;    // Borra los registros para prioridad determinada
    UDMA_ALTCLR_R = 0x00000100;     // Canal 8 utiliza la estructura de control primaria
    UDMA_USEBURSTCLR_R = 0x00000100;// Canal 8 responde a solicitudes unicas y en rafagas
    UDMA_REQMASKCLR_R = 0x00000100; // Canal 8 habilidado para solicitar al uDMA transferencias

    UDMA_CHMAP1_R = 0x00000000;    // Selecionamos el periferico 0 del Canal 8

}

volatile uint8_t *SourcePt;               // Apuntador de origen
 uint8_t *DestinationPt;  // Apuntador de Dedtino
uint32_t Count;                    // numero de bytes a transmitir
// private function used to reprogram regular channel control structure
void static setRegular(void){
  DMA_Memoria[CH8]  = (uint32_t)SourcePt;           // Primera direccion alternativa
  DMA_Memoria[CH8+1] = (uint32_t)DestinationPt;      // Ultima direccion
  DMA_Memoria[CH8+2] = 0x0C00C003+((Count-1)<<4);         // DMA Channel Control Word (DMACHCTL)
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
   XFERSIZE          13:4  count-1 Numero de items a transferir 255 (max 1024)
   NXTUSEBURST       3       0     No aplica para modo ping-pong
   XFERMODE          2:0     011   Modo Ping-Pong
  */
}
void static setAlternate(void){
  DMA_Memoria[CH8ALT]   = (uint32_t)SourcePt;               // Primera direccion alternativa
  DMA_Memoria[CH8ALT+1] = (uint32_t)DestinationPt;          // Ultima direccion
  DMA_Memoria[CH8ALT+2] = 0x0C00C003+((Count-1)<<4);             // DMA Channel Control Word (DMACHCTL)
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
  UDMA_ENASET_R = 0x00000100;     // El canal 8 esta habilitado
}


