// DMATimer.c
// Runs on LM4F120
// Periodic timer triggered DMA transfer
// Uses Timer5A to trigger the DMA, read from an 8-bit PORT, and then write to a memory Buffer
// There is a Timer5A interrupt after the buffer is full
// Jonathan Valvano
// January 1, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Operating Systems for ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2012
   Section 6.4.5, Program 6.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "tm4c1294ncpdt.h"

// The control table used by the uDMA controller.  This table must be aligned to a 1024 byte boundary.
uint32_t ucControlTable[256] __attribute__ ((aligned(1024)));
// Timer5A uses uDMA channel 8 encoding 3
#define CH8 (8*4)
#define CH8ALT (8*4+128)
#define CH2 (2*4)
#define CH2ALT (2*4+128)
#define BIT8 0x00000100
#define BIT2 0x00000004

unsigned long CounTimer=0;
// ***************** Timer5A_Init ****************
// Activate Timer5A trigger DMA periodically
// Inputs:  period in usec
// Outputs: none
//void Timer5A_Init(unsigned short period){ volatile unsigned long Delay;
void Timer5A_Init(void){ volatile unsigned long Delay;
  SYSCTL_RCGCTIMER_R |= 0x20;      // 0) activate timer5
  while((SYSCTL_PRTIMER_R & 0x20)==0);
  Delay = 0;                       // wait for completion
  TIMER5_CTL_R &= ~0x00000001;     // 1) disable timer5A during setup
  TIMER5_CFG_R = 0x00000004;       // 2) configure for 16-bit timer mode
  TIMER5_TAMR_R = 0x00000002;      // 3) configure for periodic mode, default down-count settings
  TIMER5_TAILR_R = 0x00FFFF;       // 4) reload value
  //TIMER5_TAILR_R = period-1;       // 4) reload value
  TIMER5_TAPR_R = 0xA0;              // 5) 1us timer5A
  //TIMER5_TAPR_R = 49;              // 5) 1us timer5A
  TIMER5_ICR_R = 0x00000001;       // 6) clear timer5A timeout flag
  TIMER5_IMR_R |= 0x00000001;      // 7) arm timeout interrupt
  NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|0x00000040; // 8) priority 2


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

    UART0_IFLS_R &= 0xF8;   //FIFO Configurado la transmision a 1/8 de capacidad 16 bits para rafagas

    UART0_CTL_R |= 0x00000001;      // El UART está habilitado.

}


// ************DMA_Init*****************
// Initialize the PortF to memory transfer, triggered by timer 5A
// This needs to be called once before requesting a transfer
// The source address is fixed, destination address incremented each byte
// Inputs:  period in usec
// Outputs: none
void DMA_Init(void){  //int i;
  volatile unsigned long delay;
//  for(i=0; i<256; i++){
//    ucControlTable[i] = 0;
//  }
  SYSCTL_RCGCDMA_R |= 0x01;    // µDMA Module Run Mode Clock Gating Control
  delay = SYSCTL_RCGCDMA_R;   // allow time to finish
  UDMA_CFG_R = 0x01;          // MASTEN Controller Master Enable
  UDMA_CTLBASE_R = (uint32_t)ucControlTable;
  UDMA_CHMAP0_R = (UDMA_CHMAP0_R&0xFFFFF0FF)|0x00000100;  // timer5A
  UDMA_PRIOCLR_R = 0xFF;     // default, not high priority
  UDMA_ALTCLR_R = BIT2;      // use primary control
  UDMA_USEBURSTCLR_R = BIT2; // responds to both burst and single requests
  UDMA_REQMASKCLR_R = BIT2;  // allow the µDMA controller to recognize requests for this channel

}

uint16_t *SourcePt;         //ultima direccion del buffer de memoria / incremento de 2
volatile uint32_t *DestinationPt;   //Direccion buffer SPI
uint32_t Count_Word;        //Cantidad de transferencias de 16bit

//Funcion privada para reprogramar la parte principal de un canal de la estructura de control

void Set_Regular(void){
  ucControlTable[CH2]   = (uint32_t)SourcePt;                 // first and last address
  ucControlTable[CH2+1] = (uint32_t)DestinationPt;    // last address
  ucControlTable[CH2+2] = 0xDD00C001+((Count_Word-1)<<4);             // DMA Channel Control Word (DMACHCTL)
/* DMACHCTL          Bits    Value Description
   DSTINC            31:30   11    no destination address increment
   DSTSIZE           29:28   01    16-bit destination data size
   SRCINC            27:26   11     no source address increment
   SRCSIZE           25:24   01     16-bit source data size
   reserved          23:18   0     Reserved
   ARBSIZE           17:14   0     Arbitrates after 8 transfer
   XFERSIZE          13:4  countWord-1 Transfer count items
   NXTUSEBURST       3       0      N/A for this transfer type
   XFERMODE          2:0     01      Use basic transfer mode
  */

}

// ************DMA_Star*****************
// Called to transfer halfwords from source to destination
// The source address is incremented by 16-bit, destination fxed
// Inputs:  source is a pointer to a RAM buffer containing waveform to output
//          destination is fix SSI0 buffer
//          count is the number of halfwords to transfer (max is 1024 words)
// Outputs: none
// This routine does not wait for completion
void DMA_Star(uint16_t *source, volatile uint32_t *destination, uint32_t count){
    SourcePt = source+count-1;
    DestinationPt = destination;
    Count_Word = count;
    Set_Regular();
    Set_Alternate();
    TIMER3_DMAEV_R |= 0x01;         // Activa dma_req signal para evento time-out pp1019

    NVIC_EN1_R = 1<<(35-32);     // HABILITA LA INTERRUPCION DE  TIMER3
    TIMER3_CTL_R |= 0x00000001;  // HABILITA TIMER A
    //NVIC_EN2_R |= 0x10000000;        // 9) enable interrupt 19 in NVIC
      // vector number 108, interrupt number 92

    //TIMER5_CTL_R |= 0x00000001;      // 10) enable timer5A
   // interrupts enabled in the main program after all devices initialized

    UDMA_ENASET_R |= BIT2;  // µDMA Channel 2 is enabled.
    // bit 8 in UDMA_ENASET_R become clear when done
    // bits 2:0 ucControlTable[CH8+2] become clear when done
}
// ************DMA_Status*****************
// Can be used to check the status of a previous request
// Inputs:  none
// Outputs: true if still active, false if complete
uint32_t DMA_Status(void){
  return CounTimer;  // µDMA Channel 8 enable bit is high if active
}

//*************DMA_Stop********************
void DMA_Stop(){
    UDMA_ENACLR_R = BIT8;
    NVIC_DIS2_R = 0x10000000;
    TIMER5_CTL_R &=~0x01;
}

void Timer5A_Handler(void){ // interrupts after each block is transferred
  TIMER5_ICR_R = TIMER_ICR_TATOCINT; // acknowledge timer5A timeout
  CounTimer++;



}

void Timer3A_Init(void){
    uint32_t ui32Loop;

        SYSCTL_RCGCTIMER_R |= 0x08;           // HABILITA TIMER 3
        ui32Loop = SYSCTL_RCGCGPIO_R;         // retardo para establecimiento
                                              // reloj del PORTN Y TIMER 3
        SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R12;     //habilita PORTN


        TIMER3_CTL_R = 0X00000000;       // DESHABILITA TIMER EN LA CONFIGURACION
                                         // - afecta a Timer A y B
        TIMER3_CFG_R = 0X00000004;       // CONFIGURAR PARA 16 BITS
                                         // - afecta a Timer A y B

                                         // CONFIGURAR PARA MODO PERIODICO,
                                         // CUENTA HACIA ABAJO
        TIMER3_TAMR_R = 0x00000002;

        TIMER3_TAILR_R =   0x00FFFF;     //  Cuenta básica de 16 bits
        TIMER3_TAPR_R  =       0xA0;     // 16 MHZ /16 =1MHz -> 16-1=15
                                         // (se escribe valor de prescalador -1)
        TIMER3_ICR_R   = 0x00000001;     // LIMPIA BANDERA PENDIENTE DE TIMER3A

                                     // si se usa interrupción:
        TIMER3_IMR_R |= 0x00000001;  // ACTIVA INTRRUPCION DE TIMEOUT - TIMER A

                                    // es posible programar un valor de prioridad.



        GPIO_PORTN_DIR_R = 0x01;    // configura como salida
        GPIO_PORTN_DEN_R = 0x01;    // habilita el bit 0 como digital
        GPIO_PORTN_DATA_R = 0x00;
}

void Timer3A_Handler(void){
        TIMER3_ICR_R = 0x00000001; //
        CounTimer++;
        //GPIO_PORTN_DATA_R ^= 0x01;

//        if((ucControlTable[CH8+2]&0x0007)==0){
//            Set_Regular();}
//
//        if((ucControlTable[CH8ALT+2]&0x0007)==0){
//              Set_Alternate();}
        if((UDMA_ENASET_R & BIT2)==0){
            Set_Regular();
            }
}
