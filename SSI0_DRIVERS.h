//DEFINICION DE FUNCIONES TRABAJANDO CON SPI
 void SSI0_init (void);
 void SSI0_trigger(void);
 void SSI0_sendData (uint16_t dat);
 void pot_setVal (uint8_t slider);

//DECLARACION DE VARIABLES AUXILIARES PARA LA TRANSMISION SPI
 uint8_t j=0x00;
 uint8_t k=0x00;
 uint8_t seno[]={0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,
                 0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
                 0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,
                 0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
                 0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,
                 0xea,0xeb,0xed,0xef,0xf0,0xf2,0xf3,0xf4,
                 0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfb,0xfc,
                 0xfd,0xfd,0xfe,0xfe,0xfe,0xff,0xff,0xff,
                 0xff,0xff,0xff,0xff,0xfe,0xfe,0xfd,0xfd,
                 0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,0xf6,
                 0xf5,0xf4,0xf2,0xf1,0xef,0xee,0xec,0xeb,
                 0xe9,0xe7,0xe5,0xe3,0xe1,0xdf,0xdd,0xdb,
                 0xd9,0xd7,0xd4,0xd2,0xcf,0xcd,0xca,0xc8,
                 0xc5,0xc3,0xc0,0xbd,0xba,0xb8,0xb5,0xb2,
                 0xaf,0xac,0xa9,0xa6,0xa3,0xa0,0x9d,0x9a,
                 0x97,0x94,0x91,0x8e,0x8a,0x87,0x84,0x81,
                 0x7e,0x7b,0x78,0x75,0x71,0x6e,0x6b,0x68,
                 0x65,0x62,0x5f,0x5c,0x59,0x56,0x53,0x50,
                 0x4d,0x4a,0x47,0x45,0x42,0x3f,0x3c,0x3a,
                 0x37,0x35,0x32,0x30,0x2d,0x2b,0x28,0x26,
                 0x24,0x22,0x20,0x1e,0x1c,0x1a,0x18,0x16,
                 0x14,0x13,0x11,0x10,0xe,0xd,0xb,0xa,
                 0x9,0x8,0x7,0x6,0x5,0x4,0x3,0x3,
                 0x2,0x2,0x1,0x1,0x0,0x0,0x0,0x0,
                 0x0,0x0,0x0,0x1,0x1,0x1,0x2,0x2,
                 0x3,0x4,0x4,0x5,0x6,0x7,0x8,0x9,
                 0xb,0xc,0xd,0xf,0x10,0x12,0x14,0x15,
                 0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,0x25,
                 0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,0x38,
                 0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,0x4f,
                 0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,0x67,
                 0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c,0x80};

 volatile uint32_t Delay;

 volatile uint8_t Coef;

 volatile uint8_t Shap;

 uint8_t x = 0x00;
//------------------------------------------------------------------------
//%%%%%%%%%%%$ Habilita la interrupcion numero 7 %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%---------Con la menor prioridad (7)----------------------------------
 void SSI0_trigger(void){

     SSI0_ICR_R = 0x40;                                             // Se limpian posibles interrupciones. p.1259.
     SSI0_IM_R = 0x40;                                              // Se desenmascara la interrupcion EOTIM. p.1253.
     NVIC_EN0_R|= (1<<(7));                                         //Habilita la interrupcion 7 (SSI0). p116.

     NVIC_PRI1_R |= 0xFF000000;                                     // Prioridad 7 la menor (p.152)
     pot_setVal(0x00);                                              //Activa la interrupcion
 }

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

//------------------------------------------------------------------
//%%%%%%%% Envia dato de palabra de 16 bits por SPI %%%%%%%%%%%%%%%%
// dat: contenido del buffer de transmision (MOSI)------------------
//------------------------------------------------------------------
 void SSI0_sendData (uint16_t dat) {
     SSI0_DR_R = dat;                                                   // envia dato.
 }


//-----------------------------------------------------------------
//%%%%%%%% Caracteriza al mensaje como un comando de escritura %%%%
// slider: valor que representa la amplitud de salida (8 bits)-----
//-----------------------------------------------------------------

 void pot_setVal(uint8_t slider) {
     SSI0_sendData(0x1100 | slider);                                    //los primeros 8 bits significan una escritura en el primer potenciometro
 }

//---------------------------------------------------------------------
//%%% Envio de la señal segun las variables de control recibidas-------
//---------------------------------------------------------------------

void SSI0_Handler(){

    SSI0_ICR_R = 0x40;                                                  // Se limpian posibles interrupciones. p.1259.
    int i;                                                              // Variable para manejo de la frecuencia

    switch(Shap & 0x0F){
        case 1:             //Triangular
            for (i = 0; i < (Delay); i++);
            //pot_setVal(x++);
            pot_setVal((uint8_t)((Coef*x++)/250));                      //Generacion de una señal diente de sierra por medio de un contador
        break;

        case 2:             //Cuadrada
            j++;
            if(j<0x80){

                for (i = 0; i < (Delay); i++);
                pot_setVal(0x00);
            }
            else{
                for (i = 0; i < Delay; i++);
                pot_setVal((uint8_t)(Coef));                            //Generacion de una señal cuadrada con una conmutacion de valores
                //pot_setVal(0xFF);
            }
        break;

        case 3:         //Senoidal
            k++;

            for (i = 0; i < Delay; i++);
            //pot_setVal(seno[k]);
            pot_setVal((uint8_t)((Coef*seno[k]/250)));                  //Generacion de una señal senoidal con un arreglo de puntos
        break;

        default:                                                        //Señal por default Diente de sierra
            Delay = 500;
            int iz;
            for (iz = 0; iz < Delay; iz++);
            pot_setVal(x++);
       break;
    }
}
