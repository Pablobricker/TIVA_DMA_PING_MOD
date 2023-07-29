#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "DMA_UART_INT_RXBURST.h"


//Porting
//https://www.ti.com/lit/an/spma065/spma065.pdf
//https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/446049/tm4c129-vs-tm4c123-dma
//https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/341482/beware---uart-udma-changed-on-tm4c129x-from-lm4f120-tm4c123x
//https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf?ts=1689447500642&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTM4C123GH6PM
//https://www.ti.com/lit/ds/symlink/tm4c1294ncpdt.pdf?ts=1688441837744
//ppags 1171 y 1218 y 683
//UART issues main
//https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/414362/uart-tx-dma-done-interrupt-always-1
//ADC
//https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1112153/ek-tm4c1294xl-adc-dma-interrupt-issue
//SPI
//https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1175216/tm4c129xnczad-dma-enabled-ssi
//https://www.analog.com/media/en/technical-documentation/data-sheets/max5352-max5353.pdf
//RomFunctions ve al proyecto dma_struct y ve los archivos rom.h y rom_mapped
//https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/871972/compiler-ek-tm4c123gxl-can-t-access-rom_-functions
//https://www.ti.com/lit/er/spmz850g/spmz850g.pdf?ts=1689348082296&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTM4C1294NCPDT
//https://www.ti.com/lit/ug/spmu298e/spmu298e.pdf?ts=1689315655896&ref_url=https%253A%252F%252Fwww.bing.com%252F




uint8_t MiTabla[256];


#define TOTALBYTES 16

int main(void){
    Puertos();          //0) Configura puertos
    ConfigurarUART();   //1) Configura UART
    uDMA();             //3) Configura uDMA
    DMA_Start(0x4000C000, MiTabla,TOTALBYTES);  //Configura estructura de control de canale
    while(1){           // Programa principal

    }
}

//la interrupcion se levanta hasta que se llena Mi_Tabla
//con la cantidad #TOTALBYTES no en cada rafaga
//:( asi es la maquina y se amuelan :)
void UART0_Handler(void){
    Buff_Rx_Count++;
    int i;
    for(i=0;i<16;i++){
        MiTabla[i] += 1;    //Rutina de pos procesamiento una vez recibidos los datos
    }
    DMA_Stop();

    UART0_ICR_R = 0X00010000;

}


