//Drivers para realizar transferencias por DMA usando el periferico UART
//Se reciben rafagas de 4 bytes por cada rafaga (4 transferencias)
//Se activa una interrupcion de transferencia completa cuando se hayan recibido #TOTALBYTES

#include <stdint.h>
#include "tm4c1294ncpdt.h"

#define CH8 (8*4)
#define CH8ALT (8*4+128)        //tiene que estar en la segunda mitad de la tabla DMA_Memoria[]
#define BIT8  0X00000100


void Puertos(void);
void ConfigurarUART(void);
void uDMA(void);
void DMA_Start(volatile uint8_t *source,uint8_t *destination, uint32_t count);

#define A ((volatile uint32_t *)0x4000C000);

uint32_t DMA_Memoria[256] __attribute__((aligned(1024)));

int cuenta = 0;
int Buff_Rx_Count = 0;
//Rafagas recibidas
//Con interrupciones de llegada de info


void Puertos(void){
    SYSCTL_RCGCGPIO_R |= 0x01;          //0) Reloj Port_A
    GPIO_PORTA_AFSEL_R |= 0x03;         //1) Funcion Alterna - Pines A0 y A1
    GPIO_PORTA_DEN_R |= 0x03;           //3) Modo digital - Pines A0 y A1
    GPIO_PORTA_PCTL_R |= 0x00000011;    //4) Funcion Alterna para A0/A1 = UART
    GPIO_PORTA_AMSEL_R &= ~0x03;        //5) Deshabilita modo analogico de Pines A0 y A1
}


void ConfigurarUART(void){
    SYSCTL_RCGCUART_R |=  0x00000001;        //0) Reloj UART_0 (p.505)
    while((SYSCTL_PRUART_R&0x01) == 0){};    //retardo de activacion
    UART0_CTL_R &= ~0x00000001;              //1) Deshabilita el UART durante configuracion
                                             //2) Configura baud rate = 9600 bauds
    UART0_IBRD_R = 104;                      // Valor entero       IBRD = int(16,000,000 / (16 * 9,600)) = int(104.16666)
    UART0_FBRD_R = 11;                       // Valor fraccionario FBRD = round(0.1667 * 64) = 11
    UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);            //3) Configura tamaño de palabra = 8 bits y habilita el modo FIFO
    UART0_CC_R = (UART0_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;   //4)  Fuente alterna de reloj ALTCLKCFG para el UART
    SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC;
    //5) Fuente de reloj alterna = PIOSC
    UART0_CTL_R &= ~0x00000020;     //6) Divisor de frecuencia = x16 para UART
    UART0_ICR_R = 0X00010000;       //7) Limpia la bandera de nivel de FIFO de recepcion
    UART0_IM_R |= 0X00010000;       //8) Desenmascara interrupcion
    NVIC_PRI1_R |= 0x0000E000;      //9) Habilita al NVIC a recibir interrupciones del UART0
    UART0_IFLS_R = 0x08;            //10) Nivel de FIFO = 1/8 de capacidad = 16 bits = Rafagas de 2 bytes
    UART0_CTL_R |= 0x00000001;      //8) Habilita UART_0
}




void uDMA(void){
    SYSCTL_RCGCDMA_R |= 0x01;                   //0) Reloj uDMA 1
    UDMA_CFG_R = 0x01;                          //1) Habilitar operacion de DMA
    UDMA_CTLBASE_R = (uint32_t)DMA_Memoria;     //2) Ubicar la tabla de control para los canales
    UDMA_CHMAP1_R = 0x00000000;                 //3) Fuente de solicitud. Canal 8 = UART0Rx
    UDMA_PRIOCLR_R = 0xFF;                      //4) No hay Prioridades
    UDMA_ALTCLR_R = BIT8;                       //5) Canal 8 usa la estructura de control primaria
    UDMA_USEBURSTSET_R = BIT8;                  //6) Canal 8 responde a solicitudes en rafagas
    UDMA_REQMASKCLR_R = BIT8;                   //7) Deshabilita al dma a responder solicitudes del canal 8

}



volatile uint8_t *SourcePt;         //Direccion fija del origen
uint8_t *DestinationPt;             //Direccion fija del destino
uint32_t Count;                     //Cantidad de transferencias segun el tamaño de palabra

//Funcion privada para reprogramar la parte principal de un canal de la estructura de control
void static setRegular(void){
  DMA_Memoria[CH8]  = (uint32_t)SourcePt;           //0) Direccion origen
  DMA_Memoria[CH8+1] = (uint32_t)DestinationPt;     //1) Direccion destino
  DMA_Memoria[CH8+2] = 0x0C008003+((Count-1)<<4);   //2) DMA Channel Control Word (DMACHCTL)
/* DMACHCTL          Bits    Value Description
   DSTINC            31:30   00    incremento en la direccion destino = 8-bit
   DSTSIZE           29:28   00    Tamaño del dato = 8-bit en destino
   SRCINC            27:26   11    No incrementa la direccion de origen
   SRCSIZE           25:24   00    Tamaño del dato = 8-bit en origen
   reserved          23:18   0     Reservado
   ARBSIZE           17:14   0011  Arbitraje cada 4 transferencia
   XFERSIZE          13:4  count-1 Total de transferencias
   NXTUSEBURST       3       0     No aplica para modo ping-pong
   XFERMODE          2:0     011   Modo Ping Pong
  */
}
//Funcion privada para reprogramar la parte alterna de un canal de la estructura de control
void static setAlternate(void){
  DMA_Memoria[CH8ALT]   = (uint32_t)SourcePt;               //0) Direccion origen
  DMA_Memoria[CH8ALT+1] = (uint32_t)DestinationPt;          //1) Direccion destino
  DMA_Memoria[CH8ALT+2] = 0x0C008003+((Count-1)<<4);        //2) DMA Channel Control Word (DMACHCTL)
  /* DMACHCTL          Bits    Value Description
     DSTINC            31:30   00    incremento en la direccion destino = 8-bit
     DSTSIZE           29:28   00    Tamaño del dato = 8-bit en destino
     SRCINC            27:26   11    No incrementa la direccion de origen
     SRCSIZE           25:24   00    Tamaño del dato = 8-bit en origen
     reserved          23:18   0     Reservado
     ARBSIZE           17:14   0011  Arbitraje cada 4 transferencia
     XFERSIZE          13:4  count-1 Total de transferencias
     NXTUSEBURST       3       0     No aplica para modo ping-pong
     XFERMODE          2:0     011   Modo Ping Pong
    */
}

// ************DMA_Star*****************
// configura las direcciones de origen y destino de las transferencias solicitadas
// transferencias de 8 bits en rafagas de 4 bytes
// La direccion del origen se incrementa en 8 bits
// Entradas: source Registro FIFO de recepcion UART0Rx
//          destination Apuntador de un buffer en RAM que contiene valores de 0 a 255
//          count numero de bytes a transferir (max 1024 palabras)
// Esta rutina no espera la finalizacion de la transferencia
// pero configura una interrupcion para reconocer una trtansferencia completa
//Se deben haber habilitado todos los modulos antes (GPIO, UART, uDMA)
void DMA_Start(volatile uint8_t *source,uint8_t *destination, uint32_t count){
  SourcePt = source;                    // Apuntador a la direccion origen
  DestinationPt = destination+count-1;  // Apuntador a la direccion destino
  Count = count;                        // Numero de bytes


  setRegular();             //0) Configura los parametros de la transmision
  setAlternate();           //1) Configura los parametros de la transmision en la estructua alterna
  NVIC_EN0_R |= (1<<5);     //2) Habilita interrupcion 5 para UART0
  UART0_DMACTL_R |= UART_DMACTL_RXDMAE;   //3) Activa dma_req signal para evento nivel de FIFO
  UDMA_ENASET_R = 0x00000100;     //4) Habilita el canal 8 a hacer solicitudes de transferencia
}

//*************DMA_Stop********************
//Inhabilita al DMA a recibir mas solicitudes de transferencia
void DMA_Stop(){
    UDMA_ENACLR_R = BIT8;       //0) Deshabilitar canal 8 a realizar solicitudes
    NVIC_DIS0_R |= (1<<5);      //1) Desactiva la interrupcion directo del NVIC
    UART0_IM_R &= ~0X00010000;  //2) Enmascara la interrupcion en el modulo UART
}

// ************DMA_Status*****************
// Checa cuantas transferencias se han realizado
// Entradas: nada
// Salidas: contador de transferencias completadas
uint32_t DMA_Status(void){
  return Buff_Rx_Count;     //contador de transferencias completadas
  // µDMA Channel 8 enable bit is high if active
}

