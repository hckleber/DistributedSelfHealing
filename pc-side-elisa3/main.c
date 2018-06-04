#include <stdio.h>
#include <sys/types.h>
#include <math.h>
#include <time.h>
#include "windows.h"
#include "libusb/libusb.h"

// macro for handling flags byte
#define FRONT_IR_ON(x) ((x) |= (1 << 1))
#define BACK_IR_ON(x) ((x) |= (1 << 0))
#define ALL_IR_ON(x) ((x) |= (1<<0) | (1 << 1))
#define TV_REMOTE_ON(x) ((x) |= (1<<2))
#define SLEEP_ON(x) ((x) = 0x08)
#define CALIBRATION_ON(x) ((x) |= (1<<4))
#define OBSTACLE_AVOID_ON(x) ((x) |= (1<<6))
#define CLIFF_AVOID_ON(x) ((x) |= (1<<7))
#define FRONT_IR_OFF(x) ((x) &= ~(1 << 1))
#define BACK_IR_OFF(x) ((x) &= ~(1 << 0))
#define ALL_IR_OFF(x) ((x) &= ~(1 << 0) & ~(1 << 1))
#define TV_REMOTE_OFF(x) ((x) &= ~(1 << 2))
#define SLEEP_OFF(x) ((x) &= ~(1 << 3))
#define CALIBRATION_OFF(x) ((x) &= ~(1 << 4))
#define OBSTACLE_AVOID_OFF(x) ((x) &= ~(1 << 6))
#define CLIFF_AVOID_OFF(x) ((x) &= ~(1 << 7))

#define RAD_2_DEG 57.2957796

#define NUM_ROBOTS 1
#define PAYLOAD_SIZE 13
#define ADDR_SIZE 2
#define ROBOT_PACKET_SIZE (PAYLOAD_SIZE+ADDR_SIZE)
#define PACKETS_SIZE 64
#define OVERHEAD_SIZE (2*NUM_ROBOTS+1)
#define UNUSED_BYTES 3

// The usb buffer between the pc and the base-station is 64 bytes.
// Each packet exchanged with the bast-station must contain as the
// first byte the "command id" that at the moment can be either
// "change robot state" (0x27) or "goto base-station bootloader" (0x28).
// In order to optimize the throughput the packet exchanged with the radio
// base-station contains informations to send to four different robots
// simultaneously.
// Each robot must be identified by a 2 byte address, thus we have:
// 64 - 1 - 2*4 = 55 / 4 = 13 bytes usable for the payload of each robot.
//
// Payload content for each robot:
// --------------------------------------------------------------------------
// R | B | G | IR/flags | Right | Left | Leds | ...remaining 6 bytes not used
// --------------------------------------------------------------------------
//
// * R, B, G: values from 0 (OFF) to 100 (ON max power)
// * IR/flags:
//   - first two bits are dedicated to the IRs:
//     0x00 => all IRs off
//     0x01 => back IR on
//     0x02 => front IRs on
//     0x03 => all IRs on
//   - third bit is used for enabling/disablng IR remote control (0=>diabled, 1=>enabled)
//   - fourth bit is used for sleep (1 => go to sleep for 1 minute)
//   - fifth bit is used to calibrate all sensors (proximity, ground, accelerometer)
//   - sixth bits is reserved (used by radio station)
//   - seventh bit is used for enabling/disabling onboard obstacle avoidance
//   - eight bit is used for enabling/disabling onboard cliff avoidance
// * Right, Left: speed (in percentage); MSBit indicate direction: 1=forward, 0=backward; values from 0 to 100
// * Leds: each bit define whether the corresponding led is turned on (1) or off(0); e.g. if bit0=1 then led0=on
// * remaining bytes free to be used
//
// Overhead content :
// - command: 1 byte, indicates which command the packet refer to
// - address: 2 bytes per robot


//int current_address = 0;

static struct libusb_device_handle *devh = NULL;

static int find_nrf_device(void)
{
	devh = (libusb_device_handle*)libusb_open_device_with_vid_pid(NULL, 0x1915, 0x0101);
	return devh ? 0 : -1;
}

int usb_send(char* data, int nbytes) {

	int transferred = 0;
	int r = 0;

	r = libusb_bulk_transfer(devh, 0x01, data, 64, &transferred, 5000); // address 0x01
	if (r < 0) {
		fprintf(stderr, "bulk write error %d\n", r);
		return r;
	}
	if (transferred < nbytes) {
		fprintf(stderr, "short write (%d)\n", r);
		return -1;
	}

	return 0;

}

int usb_receive(char* data, int nbytes) {

	int received = 0;
	int r = 0;

	r = libusb_bulk_transfer(devh, 0x81, data, nbytes, &received, 5000);
	if (r < 0) {
		fprintf(stderr, "bulk read error %d\n", r);
		return r;
	}
	if (received < nbytes) {
		fprintf(stderr, "short read (%d)\n", r);
		return -1;
	}

    return 0;

}

char speed(char value) {
    if(value >= 0) {
        return (value|0x80);
    } else {
        return ((-value)&0x7F);
    }
}

void curPos(int x, int y) {
  HANDLE hStdout;
  CONSOLE_SCREEN_BUFFER_INFO csbiInfo;
  hStdout=GetStdHandle(STD_OUTPUT_HANDLE);
  GetConsoleScreenBufferInfo(hStdout, &csbiInfo);
  csbiInfo.dwCursorPosition.X=x;
  csbiInfo.dwCursorPosition.Y=y;
  SetConsoleCursorPosition(hStdout, csbiInfo.dwCursorPosition);
}

int main(int argc, char *argv[]) {
    #define NB_ROBOTS 4 //20
	int r;
	unsigned int i, delayCounter=0;
    char RX_buffer[64]={0};         // Last packet received from base station
    char TX_buffer[64]={0};         // Next packet to send to base station
    unsigned int proxValue[4][7] = {0};
    unsigned char flags=0;
    unsigned long int counter = 0;
	unsigned char flagsTx = 0x00;
    unsigned char exitProg=0;
    int current_address[NB_ROBOTS] = {3215,3219}; //,3314,3321,3326,3327,3328,3329,3338,3340,3348,3354,3355,3359}; /////{3215,3218,3219,3224,3255,3282,3321,3326,3327,3328,3338,3340,3348,3354,3355,3359};   //são os endereços dos robôs que você está usando
    int enviado[NB_ROBOTS] = {0};
    clock_t TIME = 0; //0.75*NB_grupos;
    clock_t start;
    clock_t finish;


    int x = 0;
    int y = 0;
    int a = 0;
    int b = 0;
    int End_Dest = 0;
    int proxybuffer[4][7] = {0};


    for (x=0;x<4;x++){
        for (y=0;y<7;y++){
            proxValue[x][y] = -1;
        }
    }



    start = clock(); // Marcação de tempo
    //start_TX = clock();

	r = libusb_init(NULL);
	if (r < 0)
		return r;
	//printf("libusb initialized\n");

	r = find_nrf_device();
	if (r < 0) {
		fprintf(stderr, "Could not find/open device\n");
	}
	//printf("device found\n");

	r = libusb_claim_interface(devh, 0);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", r);
	}
	printf("claimed interface\n");

    TX_buffer[0]=0x27;  // Set command: change robots state

    i=0;

    while(!exitProg) {

        delayCounter++;
        if(delayCounter>10) {   // wait a little from one command to the other
            delayCounter = 0;
            if(GetKeyState (0x51) < 0) {     // 'q'
                exitProg = 1;
            }
        }




            // first robot

        if(counter == 0){
            b = 3;
        }else{
            if(counter%2 == 0){

            TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = 4;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = proxValue[0][0]&0xFF;   // Tp_Msg
            TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = proxValue[0][1]&0xFF;   // L
            TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = proxValue[0][2]&0xFF;   // F
            TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = proxValue[0][3]&0xFF;   // S
            TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = proxValue[0][4]&0xFF;   // Bw
            TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = proxValue[0][6]>>8;     // Orig
            TX_buffer[(0*ROBOT_PACKET_SIZE)+8] = proxValue[0][6]&0xFF;   // Orig(complemento)
            TX_buffer[(0*ROBOT_PACKET_SIZE)+9] = 0;//proxValue[1][5]>>8;     // Dest
            TX_buffer[(0*ROBOT_PACKET_SIZE)+10] = 0;//proxValue[1][5]&0xFF;   // Dest (complemento);
            TX_buffer[(0*ROBOT_PACKET_SIZE)+11] = 0;//proxValue[1][0]>>8;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+12] = 0;//proxValue[1][1]&0xFF;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+13] = 0;//habilita_send_robo&0xFF;          //Livre
            TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = proxValue[0][5]>>8;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = proxValue[0][5]&0xFF;
                b = 0;
                End_Dest = proxValue[0][5];

//                for(i=0;i<7;i++){ // Subistituir proxValue por proxybuffer acima
//                    proxybuffer[0][i] = 0;
//                }

            // Inicialização de troca de mensagens
            }else{

            TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = 3;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+8] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+9] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+10] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+11] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+12] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+13] = 0;
            TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = current_address[a]>>8;//current_address[a]>>8; //3219>>8&0xFF; //proxValue[0][1]&0xFF; //(current_address[4*grupo_TX]>>8)&0xFF;     // address of the robot
            TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = current_address[a]&0xFF;//current_address[a]&0xFF; //3219&0xFF; //proxValue[0][1]>>8; //current_address[4*grupo_TX]&0xFF;
                b = 1;
                End_Dest = current_address[a];
//            }
            }
        }



//        if(proxValue[0][0] != 1){
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = 0;              //Robô0
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = 0;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = 0;//proxValue[0][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = 0;//proxValue[1][0]&0xFF;              //Robô1
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = 0;//proxValue[1][0]>>8;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = 0;//proxValue[1][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = 0;//proxValue[2][0]&0xFF;              //Robô2
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+8] = 0;//proxValue[2][0]>>8;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+9] = 0;//proxValue[2][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+10] = 0;//proxValue[3][0]&0xFF;             //Robô3
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+11] = 0;//proxValue[3][0]>>8;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+12] = 0;//proxValue[3][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+13] = 0;//habilita_send_robo&0xFF;          //Livre
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = 0; //3219>>8&0xFF; //proxValue[0][1]&0xFF; //(current_address[4*grupo_TX]>>8)&0xFF;     // address of the robot
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = 0; //3219&0xFF; //proxValue[0][1]>>8; //current_address[4*grupo_TX]&0xFF;
//        }else{
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = 0;              //Robô0
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = 0;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = 0;//proxValue[0][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = 0;//proxValue[1][0]&0xFF;              //Robô1
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = 0;//proxValue[1][0]>>8;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = 0;//proxValue[1][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = 0;//proxValue[2][0]&0xFF;              //Robô2
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+8] = 0;//proxValue[2][0]>>8;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+9] = 0;//proxValue[2][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+10] = 0;//proxValue[3][0]&0xFF;             //Robô3
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+11] = 0;//proxValue[3][0]>>8;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+12] = 0;//proxValue[3][1]&0xFF;
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+13] = 5;//habilita_send_robo&0xFF;          //Livre
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = proxValue[0][6]>>8;; //3219>>8&0xFF; //proxValue[0][1]&0xFF; //(current_address[4*grupo_TX]>>8)&0xFF;     // address of the robot
//            TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = proxValue[0][6]&0xFF; //3219&0xFF; //proxValue[0][1]>>8; //current_address[4*grupo_TX]&0xFF;
//        }

//            TX_buffer[(3*ROBOT_PACKET_SIZE)+1] = proxValue[3][0]&0xFF;              //Robô0
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+2] = proxValue[3][1]&0xFF;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+3] = proxValue[3][2]&0xFF;//proxValue[0][1]&0xFF;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+4] = proxValue[3][3]&0xFF;//proxValue[1][0]&0xFF;              //Robô1
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+5] = proxValue[3][4]&0xFF;//proxValue[1][0]>>8;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+6] = proxValue[3][5]>>8;//proxValue[1][1]&0xFF;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+7] = proxValue[3][5]&0xFF;//proxValue[2][0]&0xFF;              //Robô2
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+8] = proxValue[3][6]>>8;//proxValue[2][0]>>8;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+9] = proxValue[3][6]&0xFF;//proxValue[2][1]&0xFF;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+10] = 0;//proxValue[3][0]&0xFF;             //Robô3
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+11] = 0;//proxValue[3][0]>>8;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+12] = 0;//proxValue[3][1]&0xFF;
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+13] = 0;//habilita_send_robo&0xFF;          //Livre
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+14] = proxValue[3][5]>>8;; // 3219>>8&0xFF; //proxValue[0][1]&0xFF; //(current_address[4*grupo_TX]>>8)&0xFF;     // address of the robot
//            TX_buffer[(3*ROBOT_PACKET_SIZE)+15] = proxValue[3][5]&0xFF; // 3219&0xFF; //proxValue[0][1]>>8; //current_address[4*grupo_TX]&0xFF;


            // transfer the data to the base-station


            r = usb_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);


            if(r < 0) {
              printf("send error!\n");
            }


            /* Leitura dos enviados */
            for(i=0;i<NB_ROBOTS;i++){
                if ((((signed int)TX_buffer[2]<<8)|(unsigned char)TX_buffer[1]) == current_address[i]) {
                    enviado[i] = (signed int)TX_buffer[3];
                } else if ((((signed int)TX_buffer[5]<<8)|(unsigned char)TX_buffer[4]) == current_address[i]) {
                    enviado[i] = (signed int)TX_buffer[6];
                } else if ((((signed int)TX_buffer[8]<<8)|(unsigned char)TX_buffer[7]) == current_address[i]) {
                    enviado[i] = (signed int)TX_buffer[9];
                } else if ((((signed int)TX_buffer[11]<<8)|(unsigned char)TX_buffer[10]) == current_address[i]) {
                    enviado[i] = (signed int)TX_buffer[12];
                }
            }


        // Zerando o Rx buffer
        for (x=0;x<4;x++){
            for (y=0;y<16;y++){
                RX_buffer[16*x+y] = -1;
            }
        }
        r = usb_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
        if(r < 0) {
            printf("receive error!\n");
        }


//        for(x=0;x<4;x++){
//            for(y=0;y<5;y++){
//                proxValue[x][y] = 0;
//            }
//        }



        if (RX_buffer[0]>2){// || RX_buffer[16]>2 || RX_buffer[32]>2 || RX_buffer[48]>2){ // && (RX_buffer[16]>2) && (RX_buffer[32]>2) && (RX_buffer[48]>2)){
            for (x=0;x<4;x++){
            //x=0;
                proxValue[x][0] = (signed int)RX_buffer[16*x+1]; // Tp_Msg
                proxValue[x][1] = (signed int)RX_buffer[16*x+2]; // L
                proxValue[x][2] = (signed int)RX_buffer[16*x+3]; // F
                proxValue[x][3] = (signed int)RX_buffer[16*x+4]; // S
                proxValue[x][4] = (signed int)RX_buffer[16*x+5]; // Bw
                proxValue[x][5] = ((signed int)RX_buffer[16*x+7]<<8)|(unsigned char)RX_buffer[16*x+6]; // Dest
                proxValue[x][6] = ((signed int)RX_buffer[16*x+15]<<8)|(unsigned char)RX_buffer[16*x+14]; // Origem
               }
        }

//        if(TX_buffer[(0*ROBOT_PACKET_SIZE)+1] == 3){
//            for(x=0;x<4;x++){
//                for(y=0;y<7;y++){
//                    proxybuffer[x][y] = proxValue[x][y];
//                }
//            }
//        }



        //system( "cls" );
        //printf("\033[2J");
        curPos(0,0);

        printf("Mensagem enviada:\n");
        printf("Hb_Send\t Tp_Msg\t L\t F\t S\t Bw\t Dest\t \r\n");
        printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\t \n\r\n", TX_buffer[(0*ROBOT_PACKET_SIZE)+1], TX_buffer[(0*ROBOT_PACKET_SIZE)+2], TX_buffer[(0*ROBOT_PACKET_SIZE)+3], TX_buffer[(0*ROBOT_PACKET_SIZE)+4],TX_buffer[(0*ROBOT_PACKET_SIZE)+5], TX_buffer[(0*ROBOT_PACKET_SIZE)+6], End_Dest);
//        printf("%d\t %d\t %d\t %d\t %d\t %d\t \n\r\n", TX_buffer[(1*ROBOT_PACKET_SIZE)+13], TX_buffer[(1*ROBOT_PACKET_SIZE)+2], TX_buffer[(1*ROBOT_PACKET_SIZE)+3], TX_buffer[(1*ROBOT_PACKET_SIZE)+4],TX_buffer[(1*ROBOT_PACKET_SIZE)+5], TX_buffer[(1*ROBOT_PACKET_SIZE)+6]);


        printf("\n\n Proxima mensagem:\n");
        printf("\tTp_Msg\t L\t F\t S\t Bw\t Dest\t Orig\r\n");
        printf("\t%d\t %d\t %d\t %d\t %d\t %d\t %d\t \n\r\n", proxValue[0][0], proxValue[0][1], proxValue[0][2], proxValue[0][3], proxValue[0][4], proxValue[0][5], proxValue[0][6]);
        printf("\t%d\t %d\t %d\t %d\t %d\t %d\t %d\t \n\r\n", proxValue[1][0], proxValue[1][1], proxValue[1][2], proxValue[1][3], proxValue[1][4], proxValue[1][5], proxValue[1][6]);
//        printf("\t%d\t %d\t %d\t %d\t %d\t %d\t %d\t \n\r\n", proxValue[2][0], proxValue[2][1], proxValue[2][2], proxValue[2][3], proxValue[2][4], proxValue[2][5], proxValue[2][6]);
//        printf("\t%d\t %d\t %d\t %d\t %d\t %d\t %d\t \n\r\n", proxValue[3][0], proxValue[3][1], proxValue[3][2], proxValue[3][3], proxValue[3][4], proxValue[3][5], proxValue[3][6]);
        if(b ==3){
            printf("Nenhuma mensagem enviada!");
        }else{
            printf("Robo selecionado p enviar: %d (%d)  \n", current_address[a], a);
            printf("Tipo de mensagem = ");
            if(b==0){
                printf("Habilita envio da Origem");
            }else{
                printf("Origem ========> Destino");
            }
        }


        printf("\nCOUNTER %ld\r\n", counter);

        getch(); // Pausa o andamento do programa

        CALIBRATION_OFF(flagsTx);   // calibration done once

        counter++;

        if(counter%2 == 0){
        }else{
            if (a < (NB_ROBOTS-1)){
                a++;
            }else {
                a = 0;
            }
        }
    }

	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(NULL);




	return 0;

}

