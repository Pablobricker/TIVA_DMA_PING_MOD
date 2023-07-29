
#define I2C_MCS_ACK 0x00000008 //Transmmitter Acknowledge Enable
#define I2C_MCS_DATACK 0x00000008 // Data Acknowledge Enable
#define I2C_MCS_ADRACK 0x00000004 // Acknowledge Address
#define I2C_MCS_STOP 0x00000004 // Generate STOP
#define I2C_MCS_START 0x00000002 // Generate START
#define I2C_MCS_ERROR 0x00000002 // Error
#define I2C_MCS_RUN 0x00000001 // I2C Master Enable
//#define I2C_MSA_RS 0x01 //Sentido del esclavo
#define MAXRETRIES 5 // number of receive attempts before giving up
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MDR_DATA_M          0x000000FF  // This byte contains the data
#define I2C_MSA_SA_M            0x000000FE  // I2C Slave Address

/*El clculo del Time Period Register
 (TPR) se especifica en la pgina 1284
 Asumiendo un reloj de 16 MHz y un modo de operacin estndar (100 kbps):
*/
int TPR = 1;

//**Direcciones del VL53L0X
int VL53L0xAdd =0x29;///Direccn del VL53L0x
int SubRegIdx= 0x0014;
int AdreMin=0x01;




// Variables para manejar los valores del sensor
uint8_t error;
uint32_t i;
uint8_t data[10];
int Interrupcion=0;
uint8_t range[2];
uint8_t ranges[2];
uint16_t tmpuint16;

#define VL53L0X_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
        (uint16_t)lsb)

//Contadores de fallas para las funciones de poleo
int failr=0;
int fail=0;

uint8_t lidar_id;

uint8_t stop_variable=0;

typedef enum
{
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

uint8_t cntr = 0x00;

//-------------------------------------------------------------
//%%% Funcion que inicializa los relojes, el GPIO y el I2C0 %%%
//-------------------------------------------------------------
void I2C_Init(void){
    //RELOJ
    SYSCTL_RCGCI2C_R |= 0x0001;
    SYSCTL_RCGCGPIO_R |= 0x0002;
    while((SYSCTL_PRGPIO_R&0x0002) == 0){};
    //GPIO
    GPIO_PORTB_AFSEL_R |= 0x0C;
    GPIO_PORTB_ODR_R |= 0x08;
    GPIO_PORTB_DIR_R |= 0x0C;                               //Activo al PB2 y al PB3 como OUTPUT
    GPIO_PORTB_DEN_R |= 0x0C;                               //Activo la funcide PB3 y PB2
    GPIO_PORTB_PCTL_R|=0x00002200;

    //CONFIGURAC DEL MODULO I2C0
    I2C0_MCR_R = 0x00000010;                                    // Habilitar funcion para el I2C0
    I2C0_MTPR_R = TPR;                                          // Se establece una velocidad estndar de 100kbps

}

//-------------------------------------------------------------
//%%% Inicializa los registros del slave para lectura %%%%%%%%%
//-------------------------------------------------------------
void VL53_Init(){
            while(I2C0_MCS_R&0x00000001){};                     //esperar a que I2C este listo

            VL53_WHOAMI();
            data_init();                                        //Obtiene el ID del lidar para comprobar la comunicacon
            static_init();                                      //Configura los registros del sensor para empezar la medicion
            ref_calibration();                                  //Configura la escala de medicion en mm

            //HABILITAR INTERRUPCION
            I2C0_MICR_R = 0x00000041;
            I2C0_MIMR_R |= 0x00000041;
            NVIC_EN0_R = (1<<8);                                // Habilita NVIC para la bandera I2C0
            NVIC_PRI2_R |= 0x000000B0;                          // Prioridad 6 (p.152) Interrupcion 8

           //TRIGGER PARA ACTIVAR LA INTERRUPCION               //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;            // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                          // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF

            I2C0_MDR_R = (0x80 & 0xFF);
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);         //LECTURA DEL REGISTRO 0X80 "MEDICION"
}

//----------------------------------------------------------------
//%%% Transmite la direccion del esclavo con el sentido de lectura
// deviceAdress: Direccion del esclavo----------------------------
// targetRegister: Registro a donde apunta la lectura-------------
//----------------------------------------------------------------
void I2C_direct_read(uint8_t deviceAddress, uint8_t targetRegister) {

    for(i=0; i<175; i++);
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                           // wait for transmission done

    I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;               // MSA[7:1] is slave address
    I2C0_MSA_R &= ~I2C_MSA_RS;                                      // MSA[0] is 0 for send

    I2C0_MDR_R = targetRegister & I2C_MDR_DATA_M;                   // prepare targetRegister

    I2C0_MCS_R = (I2C_MCS_START  |                                  // generate start/restart
                  I2C_MCS_RUN |                                     // generate enable
                  I2C_MCS_STOP);                                    // master stop

    for(i=0; i<175; i++);
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                           // wait for transmission done
}

//------------------------------------------------------------------
//%%% Transmite la direccion del esclavo con el sentido de escritura
// deviceAdress: Direccion del esclavo------------------------------
// targetRegister: Registro donde apunta la escritura---------------
//------------------------------------------------------------------
void I2C_direct_write(uint8_t deviceAddress, uint8_t targetRegister) {

    for(i=0; i<175; i++);
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done

    I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address
    I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send

    I2C0_MDR_R = targetRegister & I2C_MDR_DATA_M;               // prepare targetRegister

    I2C0_MCS_R = (I2C_MCS_START  |                                   // generate start/restart
                  I2C_MCS_RUN  );                                    // generate enable


    for(i=0; i<175; i++);
    while (I2C0_MCS_R & I2C_MCS_BUSY) {};                            // wait for transmission done
}

//---------------------------------------------------------------------------
//%%% Realiza la lectura de un byte %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//ReaData: Apuntador donde se almacena el byte leido-------------------------
//---------------------------------------------------------------------------
void I2C_read(uint8_t deviceAddress, uint8_t targetRegister, uint8_t *ReaData){


    I2C_direct_read(deviceAddress, targetRegister);
        // check error bits
        if((I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
            failr=1;
        }
        do {
                        for(i=0; i<175; i++);
                        while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for I2C ready
                        I2C0_MSA_R = (deviceAddress << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
                        I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive

                        I2C0_MCS_R = (I2C_MCS_STOP  |                        // generate stop
                                      I2C_MCS_START |                        // generate start/restart
                                      I2C_MCS_RUN);                          // master enable
                        for(i=0; i<175; i++);
                        while (I2C0_MCS_R & I2C_MCS_BUSY) {};                // wait for transmission done

                    }                                                        // repeat if error
                    while (((I2C0_MCS_R & (I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0));
                    ReaData[0] = (I2C0_MDR_R & I2C_MDR_DATA_M);
}


//---------------------------------------------------------------------------
//%%% Realiza la escritura de un byte %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//data: Byte que se va a escribir al registro--------------------------------
//---------------------------------------------------------------------------
void I2C_write(uint8_t deviceAddress, uint8_t targetRegister, uint8_t data){


    I2C_direct_write(deviceAddress, targetRegister);
        // check error bits
        if((I2C0_MCS_R & (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
            I2C0_MCS_R = I2C_MCS_STOP;                                   // stop transmission
            // return error bits if nonzero
            fail=1;
        }

        I2C0_MDR_R = data & I2C_MDR_DATA_M;                       // prepare data byte
                I2C0_MCS_R = (I2C_MCS_STOP |                                 // generate stop
                              I2C_MCS_RUN);                                  // master enable
                for(i=0;i<175;i++);
                while (I2C0_MCS_R & I2C_MCS_BUSY) {};                        // wait for transmission done

}

//--------------------------------------------------------------
//%%% Obtiene el id del dispositivo leyendo el registro 0xC0 %%%
//--------------------------------------------------------------
void VL53_WHOAMI(){

    I2C_read(0x29, 0xC0,&lidar_id);
}

void data_init(){

    uint8_t vhv_config_scl_sda=0;

    //Set 2v8 mode of power supply
    I2C_read(0x29,0x89,&vhv_config_scl_sda);
    vhv_config_scl_sda |=0x01;
    I2C_write(0x29,0x89,vhv_config_scl_sda);


    //Set i2c estandar mode
    I2C_write(0x29,0x88,0x00);
    I2C_write(0x29,0x80,0x01);
    I2C_write(0x29,0xFF,0x01);
    I2C_write(0x29,0x00,0x00);

    I2C_read(0x29,0x91,&stop_variable);

    I2C_write(0x29,0x00,0x01);
    I2C_write(0x29,0xFF,0x00);
    I2C_write(0x29,0x80,0x00);

}


void static_init(){

    //Load default tuning settings
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x00, 0x00);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x09, 0x00);
    I2C_write(0x29,0x10, 0x00);
    I2C_write(0x29,0x11, 0x00);
    I2C_write(0x29,0x24, 0x01);
    I2C_write(0x29,0x25, 0xFF);
    I2C_write(0x29,0x75, 0x00);
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x4E, 0x2C);
    I2C_write(0x29,0x48, 0x00);
    I2C_write(0x29,0x30, 0x20);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x30, 0x09);
    I2C_write(0x29,0x54, 0x00);
    I2C_write(0x29,0x31, 0x04);
    I2C_write(0x29,0x32, 0x03);
    I2C_write(0x29,0x40, 0x83);
    I2C_write(0x29,0x46, 0x25);
    I2C_write(0x29,0x60, 0x00);
    I2C_write(0x29,0x27, 0x00);
    I2C_write(0x29,0x50, 0x06);
    I2C_write(0x29,0x51, 0x00);
    I2C_write(0x29,0x52, 0x96);
    I2C_write(0x29,0x56, 0x08);
    I2C_write(0x29,0x57, 0x30);
    I2C_write(0x29,0x61, 0x00);
    I2C_write(0x29,0x62, 0x00);
    I2C_write(0x29,0x64, 0x00);
    I2C_write(0x29,0x65, 0x00);
    I2C_write(0x29,0x66, 0xA0);
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x22, 0x32);
    I2C_write(0x29,0x47, 0x14);
    I2C_write(0x29,0x49, 0xFF);
    I2C_write(0x29,0x4A, 0x00);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x7A, 0x0A);
    I2C_write(0x29,0x7B, 0x00);
    I2C_write(0x29,0x78, 0x21);
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x23, 0x34);
    I2C_write(0x29,0x42, 0x00);
    I2C_write(0x29,0x44, 0xFF);
    I2C_write(0x29,0x45, 0x26);
    I2C_write(0x29,0x46, 0x05);
    I2C_write(0x29,0x40, 0x40);
    I2C_write(0x29,0x0E, 0x06);
    I2C_write(0x29,0x20, 0x1A);
    I2C_write(0x29,0x43, 0x40);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x34, 0x03);
    I2C_write(0x29,0x35, 0x44);
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x31, 0x04);
    I2C_write(0x29,0x4B, 0x09);
    I2C_write(0x29,0x4C, 0x05);
    I2C_write(0x29,0x4D, 0x04);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x44, 0x00);
    I2C_write(0x29,0x45, 0x20);
    I2C_write(0x29,0x47, 0x08);
    I2C_write(0x29,0x48, 0x28);
    I2C_write(0x29,0x67, 0x00);
    I2C_write(0x29,0x70, 0x04);
    I2C_write(0x29,0x71, 0x01);
    I2C_write(0x29,0x72, 0xFE);
    I2C_write(0x29,0x76, 0x00);
    I2C_write(0x29,0x77, 0x00);
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x0D, 0x01);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x80, 0x01);
    I2C_write(0x29,0x01, 0xF8);
    I2C_write(0x29,0xFF, 0x01);
    I2C_write(0x29,0x8E, 0x01);
    I2C_write(0x29,0x00, 0x01);
    I2C_write(0x29,0xFF, 0x00);
    I2C_write(0x29,0x80, 0x00);

    //Configure Interrupt
    I2C_write(0x29,0x0A,0x04);
    //Jalar el pin de gpio a un nivel alto porque no lo hac por default

    uint8_t gpio_hv_mux_active_high = 0;
    I2C_read(0x29,0x84,&gpio_hv_mux_active_high);
    gpio_hv_mux_active_high &= ~0x01;
    I2C_write(0x29, 0x84,gpio_hv_mux_active_high);

    //Clear interrupt
    I2C_write(0x29,0x0B,0x01);

    //secuencia especfica de pasos

    I2C_write(0x29, 0x01,0x28 + 0x40 + 0x80);

}


void single_ref_calibration(calibration_type_t calib_type){
    uint8_t sysrange_start = 0;
        uint8_t sequence_config = 0;
        switch (calib_type)
        {
        case CALIBRATION_TYPE_VHV:
            sequence_config = 0x01;
            sysrange_start = 0x01 | 0x40;
            break;
        case CALIBRATION_TYPE_PHASE:
            sequence_config = 0x02;
            sysrange_start = 0x01 | 0x00;
            break;
        }

        I2C_write(0x29,0x01,sequence_config);

        I2C_write(0x29,0x00,sysrange_start);

        uint8_t interrupt_status = 0;

        do{
            I2C_read(0x29,0x13,&interrupt_status);
        }while ((interrupt_status & 0x07) == 0);

        I2C_write(0x29, 0x0B, 0x01);

        I2C_write(0x29,0x00, 0x00);

}

void ref_calibration(){
    single_ref_calibration(CALIBRATION_TYPE_VHV);
    single_ref_calibration(CALIBRATION_TYPE_PHASE);

    I2C_write(0x29,0x01,0x28 + 0x40 + 0x80);
}


//-------------------------------------------------------------------
//%%% Ciclo de lectura repetitiva consta de 8 escrituras y 2 lecturas
//%%% Ciclos de comprobaciones para verificar el rango %%%%%%%%%%%%%%
//-------------------------------------------------------------------
void I2C0_Handler(){
    uint8_t interrupt_status = 0;
    cntr++;

        I2C0_MICR_R |= 0x00000041;  //Clear interrupt



        switch(cntr & 0XFF){

        case 0x01:
             I2C0_MDR_R = (0x01 & 0xFF);
             I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

                break;

        case 0x02:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0xFF;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x03:
            I2C0_MDR_R = (0x01 & 0xFF);
            I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

       break;

        case 0x04:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0x00;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x05:
            I2C0_MDR_R = (0x00 & 0xFF);
            I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);
            I2C0_MICR_R |= 0x00000040;  //Clear interrupt STOP
        break;

        case 0x06:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0x91;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x07:
            I2C0_MDR_R = (stop_variable & 0xFF);
            I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

        break;

        case 0x08:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0x00;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x09:
            I2C0_MDR_R = (0x01 & 0xFF);
            I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

        break;

        case 0x0A:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0xFF;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x0B:
            I2C0_MDR_R = (0x00 & 0xFF);
            I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

        break;

        case 0x0C:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0x80;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x0D:
             I2C0_MDR_R = 0x00;
             I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

        break;

        case 0x0E:
                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0x00;
            I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);
        break;

        case 0x0F:
            I2C0_MIMR_R = 0x00000040;
            I2C0_MDR_R = 0x01;
            I2C0_MCS_R = (I2C_MCS_STOP | I2C_MCS_RUN);

        break;

        case 0x10:

                                                                    //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
            I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
            I2C0_MDR_R = 0x13;
            I2C0_MCS_R = (I2C_MCS_START |I2C_MCS_STOP | I2C_MCS_RUN);

        break;
        case 0x11:
            I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
            I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
            I2C0_MCS_R = (I2C_MCS_START |I2C_MCS_STOP | I2C_MCS_RUN);

        break;

        case 0x12:


        interrupt_status = I2C0_MDR_R & 0xFF;

        if(interrupt_status & 0x07 == 0){
        cntr = 0x0F;
                                                                        //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
        I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
        I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
        I2C0_MDR_R = 0x13;
        I2C0_MCS_R = (I2C_MCS_START |I2C_MCS_STOP | I2C_MCS_RUN);

       }
       else{
                                                               //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
       I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
       I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
       I2C0_MDR_R = (0x14 + 10) & 0xFF;
       I2C0_MCS_R = (I2C_MCS_START |I2C_MCS_STOP | I2C_MCS_RUN);

                    }
      break;


      case 0x13:
      I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
      I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
      I2C0_MCS_R = (I2C_MCS_START |I2C_MCS_STOP | I2C_MCS_RUN);


       break;

       case 0x14:
       ranges[0] = I2C0_MDR_R & 0xFF;

       I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;    // MSA[7:1] is slave address
       I2C0_MSA_R |= I2C_MSA_RS;                            // MSA[0] is 1 for receive
       I2C0_MCS_R = (I2C_MCS_START |I2C_MCS_STOP | I2C_MCS_RUN);


       break;


       case 0x15:
       I2C0_MIMR_R = 0x00000041;
       ranges[1] = I2C0_MDR_R & 0xFF;

       cntr=0x00;
       tmpuint16 = VL53L0X_MAKEUINT16(ranges[1], ranges[0]);
                                                               //Pero por alguna razon el registro MDR se traba en el ultimo valor leido si uso interrupciones
       I2C0_MSA_R = (0x29 << 1) & I2C_MSA_SA_M;                // MSA[7:1] is slave address ES AQUI DONDE ESTA EL ERROR XDDDD
       I2C0_MSA_R &= ~I2C_MSA_RS;                                       // MSA[0] is 0 for send NO LE TIENES QUE PONER &0XFF
       I2C0_MDR_R = 0x80;
       I2C0_MCS_R = (I2C_MCS_START | I2C_MCS_RUN);

       break;

       default:
                    cntr=0x00;
       break;


        }



}


