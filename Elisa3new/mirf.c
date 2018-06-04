/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#include "mirf.h"
#include "nRF24L01.h"
#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <variables.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )


// Flag which denotes transmitting mode
volatile uint8_t PTX;

void mirf_init()
// Initializes pins as interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
    // Define CSN and CE as Output and set them to default
    //DDRB |= ((1<<CSN)|(1<<CE));
    mirf_CE_hi;
    mirf_CSN_hi;

	mirf_config();
}


void mirf_config()
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
{

	uint8_t temp[3];

	// power down
	mirf_config_register(CONFIG, 0x0D);

	// address width
	mirf_config_register(SETUP_AW, 0x01);

	// tx address
	temp[0] = (rfAddress>>8)&0xFF;
	temp[1] = rfAddress & 0xFF;
	temp[2] = 0x00;
	mirf_write_register(TX_ADDR, temp, 3);

	// rx address => same as tx address for auto ack
	mirf_write_register(RX_ADDR_P0, temp, 3);

	// enable auto ack for pipe0
	mirf_config_register(EN_AA, 0x01);

	// enable pipe0
	mirf_config_register(EN_RXADDR, 0x01);

	// 500µs (+ 86µs on-air), 2 re-transmissions
	mirf_config_register(SETUP_RETR, 0x12);

    // select RF channel
    mirf_config_register(RF_CH,40);

	// RX payload size; it isn't needed because the dynamic payload length is activated for ACK+PAYLOAD feature
    mirf_config_register(RX_PW_P0, PAYLOAD_SIZE);

	// enable extra features
    mirf_CSN_lo;
    SPI_Write_Byte(NRF_ACTIVATE);
    SPI_Write_Byte(0x73);
    mirf_CSN_hi;

	// enable dynamic payload for pipe0
	mirf_config_register(NRF_DYNPD, 0x01);

	// enable payload with ACK and dynamic payload length
	mirf_config_register(NRF_FEATURE, 0x06);

	// power up; enable crc (2 bytes); prx; max_rt, tx_ds enabled
	mirf_config_register(CONFIG, 0x0F);

    // Start receiver
    //PTX = 0;        // Start in receiving mode
    //RX_POWERUP;     // Power up in receiving mode
    //mirf_CE_hi;     // Listening for pakets
}

void mirf_set_RADDR(uint8_t * adr)
// Sets the receiving address
{
    mirf_CE_lo;
    mirf_write_register(RX_ADDR_P0,adr,5);
    mirf_CE_hi;
}

void mirf_set_TADDR(uint8_t * adr)
// Sets the transmitting address
{
	mirf_write_register(TX_ADDR, adr,5);
}

/*
#if defined(__AVR_ATmega8__)
SIGNAL(SIG_INTERRUPT0)
#endif // __AVR_ATmega8__
#if defined(__AVR_ATmega168__)
SIGNAL(SIG_PIN_CHANGE2)
#endif // __AVR_ATmega168__
// Interrupt handler
{
    uint8_t status;
    // If still in transmitting mode then finish transmission
    if (PTX) {

        // Read MiRF status
        mirf_CSN_lo;                                // Pull down chip select
        status = spi_fast_shift(NOP);               // Read status register
        mirf_CSN_hi;                                // Pull up chip select

        mirf_CE_lo;                             // Deactivate transreceiver
        RX_POWERUP;                             // Power up in receiving mode
        mirf_CE_hi;                             // Listening for pakets
        PTX = 0;                                // Set to receiving mode

        // Reset status register for further interaction
        mirf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)); // Reset status register
    }
}
*/

uint8_t mirf_data_ready()
// Checks if data is available for reading
{
    if (PTX) return 0;
    uint8_t status;
    // Read MiRF status
    mirf_CSN_lo;                                // Pull down chip select
    status = SPI_Write_Byte(NOP);               // Read status register
    mirf_CSN_hi;                                // Pull up chip select
    return status & (1<<RX_DR);

}

uint8_t rx_fifo_is_empty() {

	uint8_t fifo_status = 0;

	mirf_read_register(FIFO_STATUS, &fifo_status, 1);

	return (uint8_t)(fifo_status&0x01);
}

void flush_rx_fifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_RX);
    mirf_CSN_hi;

}

void mirf_get_data(uint8_t * data)
// Reads mirf_PAYLOAD bytes into data array
{
    mirf_CSN_lo;                               		// Pull down chip select
    SPI_Write_Byte( R_RX_PAYLOAD );            		// Send cmd to read rx payload
    SPI_ReadWrite_Block(data,data,PAYLOAD_SIZE); 	// Read payload
    mirf_CSN_hi;                               		// Pull up chip select
    mirf_config_register(STATUS,(1<<RX_DR));   		// Reset status register
}

void mirf_config_register(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Byte(value);
    mirf_CSN_hi;
}

void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(R_REGISTER | (REGISTER_MASK & reg));
    SPI_ReadWrite_Block(value,value,len);
    mirf_CSN_hi;
}

void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len)
// Writes an array of bytes into inte the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Block(value,len);
    mirf_CSN_hi;
}


void mirf_send(uint8_t * value, uint8_t len)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    while (PTX) {}                  // Wait until last paket is send

    mirf_CE_lo;

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up

    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_CSN_hi;                    // Pull up chip select

    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( W_TX_PAYLOAD ); // Write cmd to write payload
    SPI_Write_Block(value,len);   // Write payload
    mirf_CSN_hi;                    // Pull up chip select

    mirf_CE_hi;                     // Start transmission
}

void writeAckPayload(unsigned char *data, unsigned char size) {

	unsigned char k = 0;

	flushTxFifo();

    mirf_CSN_lo;

	SPI_Write_Byte(NRF_W_ACK_PAYLOAD_P0);

	for(k=0; k<size; k++) {
		SPI_Write_Byte(data[k]);
	}

    mirf_CSN_hi;


}


void flushTxFifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_TX);
    mirf_CSN_hi;

}



void handleRFCommands(void) { // parametros para a fç de comunicação do robo

	int i = 0;
	int j = 0;
	int Teste = 0;

	//Recebimento da msg no robô
	if(mirf_data_ready()) {

		rfFlags |= 0x02;

		// clear irq status
		mirf_config_register(STATUS, 0x70);

		mirf_get_data(rfData);
		flush_rx_fifo();


        //xxx = 5;//rfData[12]&0xFF;
        if(Halt==-1){

		H_Send = rfData[0]&0xFF;

		int Msg_Rec[1][6] = {-1};

		for(i=1;i<=5;i++){
		    Msg_Rec[0][i-1] = rfData[i]&0xFF;
		}
		Msg_Rec[0][5] = rfData[6]<<8 | rfData[7]&0xFF;

		if(H_Send == 3){
            Fila[F_Ini][0] = 3;
		}else{
		    if(H_Send == 4){ // Mudado rfData[0]&0xFF; para H_Send
		        //Verificar duplicidade de msg
		        for(i=0;i<=2;i++){
		            if(Msg_Rec[0][i] != Fila[F_Ini][i+1]){
		                Teste = 1;
		            }
		        }

		        if(Teste == 1){

                if(F_Ini == F_Fim){// Teste de Fila vazia
                    for(j=0;j<=5;j++){
                        Fila[F_Ini][j] = rfData[j]&0xFF; // Tp_Msg(1)/L(2)/F(3)/S(4)/Bw(5)
                    }
                    Fila[F_Ini][6] = rfData[6]<<8 | rfData[7]&0xFF; // Orig
                    Fila[F_Ini][7] = -1;
                    F_Fim = (F_Fim+1)%N_Fila;
                }else{
                    if(((F_Fim+1)%N_Fila) == F_Ini){// Teste de Fila cheia
                        turnOnGreenLeds();
                    }else{
                        for(j=0;j<=5;j++){
                            Fila[F_Fim][j] = rfData[j]&0xFF; // Tp_Msg(1)/L(2)/F(3)/S(4)/BwR(5)
                        }
                        Fila[F_Fim][6] = rfData[6]<<8 | rfData[7]&0xFF; // Orig
                        Fila[F_Fim][7] = -1;
                        F_Fim = (F_Fim+1)%N_Fila;
                    }
                }

            }
		    }
		}



        //Robô montando para enviar para o PC

		// write back the ack payload
		if(Fila[F_Ini][0] == 3){//
		ackPayload[0] = packetId&0xFF; // Verifica se é lixo
    	ackPayload[1] = Tp_Msg&0xFF; // Parâmetro "Tp_Msg"
	    ackPayload[2] = L&0xFF; // Parâmetro "L" passado
		ackPayload[3] = F&0xFF; // Parâmetro "F" passado
		ackPayload[4] = S&0xFF; //Parâmetro "S" passado
		ackPayload[5] = Bw&0xFF; //Parâmetro "Bw" passado
		ackPayload[6] = Dest&0xFF; //Endereço - atualmente do destino
		ackPayload[7] = Dest>>8; //Endereço - atualmente do destino
		ackPayload[8] = 0; //T[0]&0xFF;
		ackPayload[9] = 0; //T[1]&0xFF;
		ackPayload[10] = 0; //T[2]&0xFF;
		ackPayload[11] = 0; //T[3]&0xFF;
		ackPayload[12] = 0; //T[4]&0xFF;
		ackPayload[13] = 0; //T[5]&0xFF;
		ackPayload[14] = rfAddress&0xFF; //Edereço do emissor - T[6]&0xFF;
		ackPayload[15] = rfAddress>>8; //Edereço do emissor - T[7]&0xFF;


		writeAckPayload(ackPayload, 16);



		}

    }
	}

}




