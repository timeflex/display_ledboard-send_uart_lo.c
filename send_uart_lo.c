#include <stdio.h>
#include <unistd.h>			/*  Used for UART*/
#include <fcntl.h>			/*  Used for UART*/
#include <termios.h>		/*  Used for UART*/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "parson.h"


#define STR(x)			STRINGIFY(x)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/*
#define STRINGIFY(x)	#x
#define STR(x)			STRINGIFY(x)

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
*/
#define DISPLAYLOG(args...)    fprintf(stderr,"flextec_log-displaycontroller: " args) /* message that is destined to the user */


/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_DISPLAY_UART_PATH  "/dev/ttyS1"
#define DEFAULT_MODEM_UART_PATH    "/dev/ttyS2"



/* --------  PRIVATE VARIABLES    ---*/
int err;
int i;
char *rx_ch = "RX:";
char *rx_rssi="RSSI";

char *rx_cmg_start="+CMSG: Start";
char *rx_cmg_tx="+CMSG: TX \"2A\"";
char *rx_cmg_wait_ack="+CMSG: Wait ACK";
char *rx_cmg_ack_rcvd="+CMSG: ACK Received";
unsigned char *rx_p;
unsigned char dis_buf[134];
unsigned char pre_dis_buf[134];

static int keep_alive_interval=600;/* keep alive check time in seconds */
static int counter_keep_alive=0;
static int max_try_keep_alive=30;


/* --------  PRIVATE VARIABLES GLOBAL   ---*/
static bool network_joined;
static bool incompl_dta;
static bool incompl_dta_kalive;
static bool dis_buffer_ready;
static bool kepp_alive_ack;
static bool send_keep_alive;
static bool keep_alive_trnfd;


static char display_path[64] = DEFAULT_DISPLAY_UART_PATH; /* path of the TTY port display board (led) is connected on */
static char modem_path[64]   = DEFAULT_MODEM_UART_PATH;   /* path of the TTY port modem board (lora modem) is connected on */

unsigned char rx_buffer[256];
unsigned char rx_buffer_keepalive[256];
unsigned char tmp_rx_buffer[256];
unsigned char tmp_pr_rx_buffer[256];
unsigned char tmp_kealive_rx_buffer[256];

struct discont_conf_ {
	bool isSetup;
	bool isGPS;
	bool isDisplay;
	bool isLan;
	bool isWan;
	bool isWifi;
	bool isLoraBridge;
	bool isDebug;
	bool gw_2g_3g;

};
struct discont_dets {

	struct discont_conf_ gwconf;
	char flextech_gw_ver[4];
	char leduart;
	char reuart;


};



/*----------PUBLIC FUNCTIONS----------*/

void *display_uart(void *arg);
void *keep_alive(void *arg);
int flextec_discon_configuration(const char * conf_file);
int hex_to_ascii(char c, char d);
int hex_to_int(char c);
void clearScreen(int line);


/* --- MUTEX for updating send buffer ----*/
pthread_mutex_t tx_buf_data_lock;
pthread_mutex_t tx_display_buf_data_lock= PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tx_start_buf_data_lock;
pthread_mutex_t tx_startkeep_alive_data_lock;

pthread_cond_t  tx_data_recvd  = PTHREAD_COND_INITIALIZER;


/* -------- PTHRED DISPLAY ----------*/

pthread_t uart_disp_t;
pthread_t keep_alive_t;

/*  -----MODEM CONFIG TX BYTES -----*/

 char *at_cmds[] = {"AT+ID\n",
						"AT+MODE=LWOTAA\n",
						"AT+CLASS=C\n",
						"AT+KEY=APPKEY,"" \"2B7E151628AED2A6ABF7158809CF4F3C\"\n",
						"AT+ID=AppEui, \"BE7A0000FFEE09CF\"\n",
						"AT+RXWIN2=866.55,SF12,125\n",/*"AT+RXWIN2=869.525,SF12,125\n",*/
						"AT+CH=0,865.1,DR0,DR5\n",
						"AT+CH=1,865.3,DR0,DR6\n",
						"AT+CH=2,865.5,DR0,DR5\n",
						"AT+JOIN\n",
						"AT+RXWIN2?"
						};


/*  -----MODEM TX BYTES -----*/

char *start_tx_data[]={"AT+MSG=LED_START\n"};
char *tx_data[]={0};
char tx_keep_alive[]={"AT+CMSG=2A\n"};
char init_rx_data[134]=           {"L S T F l e x i   S i g n    i n i t i a l i z i n g                                                                                  "};
char join_network[134]=           {"L S T A T  +  J O I N                                                                                                                 "};
char join_network_fail[134]=      {"L S T + J O I N : J O I N      F A I L E D                                                                                            "};
char join_network_success[134]=   {"L S T + J O I N : J O I N      S U C C E S S                                                                                          "};
char join_network_modem_busy[134]={"L S T + J O I N : L o R a W A N  m o d e m   i s  b u s y                                                                             "};
/*
  -------------------------
  ----- SETUP USART 0 -----
  -------------------------
  At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
*/
int uart0_filestream = -1;
int uart1_filestream = -1;


int main(void){


	/* parse json file for setup configurations*/
	const char dispcont_conf_fname[] = "/etc/config/dispcont_flextec_conf.json"; /* contain display configuration */

	if (access(dispcont_conf_fname, R_OK) == 0) {
		/* if there is a debug conf, parse only the debug conf */
		DISPLAYLOG("INFO: found  configuration file %s, \n",
				dispcont_conf_fname);
		flextec_discon_configuration(dispcont_conf_fname);
	} else {
		DISPLAYLOG(
				"INFO:  configuration file %s not found ,default configuration will be applied, \n",
				dispcont_conf_fname);
		/*	        return EXIT_FAILURE;*/
	}




	if (pthread_mutex_init(&tx_buf_data_lock, NULL) != 0){
		printf("\n mutex init failed\n");
				return 1;
	}

	err = pthread_create(&(uart_disp_t), NULL, &display_uart, NULL);
	if (err != 0){
		 printf("\ncan't create thread :[%s]", strerror(err));
	}




	/*
	  OPEN THE UART
	  The flags (defined in fcntl.h):
	  	Access modes (use 1 of these):
	  		O_RDONLY - Open for reading only.
	  		O_RDWR - Open for reading and writing.
	  		O_WRONLY - Open for writing only.

	  	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	  											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	  											immediately with a failure status if the output can't be written immediately.

	  	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	*/
	uart0_filestream = open(modem_path, O_RDWR | O_NOCTTY | O_NDELAY);/*  Open in non blocking read/write mode*/
	if (uart0_filestream == -1)
	{
			/*  ERROR - CAN'T OPEN SERIAL PORT*/
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
/* display uart*/
	uart1_filestream = open(display_path, O_RDWR | O_NOCTTY | O_NDELAY);/*  Open in non blocking read/write mode*/
	if (uart1_filestream == -1)
	{
			/*  ERROR - CAN'T OPEN SERIAL PORT*/
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
		/*
		  CONFIGURE THE UART
		  The flags (defined in /usr/include/termios.h - see http:  pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
		  	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
		  	CSIZE:- CS5, CS6, CS7, CS8
		  	CLOCAL - Ignore modem status lines
		  	CREAD - Enable receiver
		  	IGNPAR = Ignore characters with parity errors
		  	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
		  	PARENB - Parity enable
		  	PARODD - Odd parity (else even)
		*/
	struct termios options;
	struct termios options1;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		/*  <Set baud rate*/
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	tcgetattr(uart1_filestream, &options1);
	options1.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		/*  <Set baud rate*/
	options1.c_iflag = IGNPAR;
	options1.c_oflag = 0;
	options1.c_lflag = 0;
	tcflush(uart1_filestream, TCIFLUSH);
	tcsetattr(uart1_filestream, TCSANOW, &options1);

	sleep(5);
	/* init display */
	if (uart1_filestream != -1) {
		clearScreen(3);
		sleep(10);
		if ((init_rx_data != NULL) && (init_rx_data[0] != '\0')) {
			/* write uart tx buffer */
			/*memset(tempdisp,' ',128);*/
			if (uart1_filestream != -1) {
				int count = write(uart1_filestream, init_rx_data,134);
				if (count < 0) {
					printf("UART TX error\n");
				}
					printf(" bytes written : %s\n", init_rx_data);


			}
		}
	}

	sleep(2);

	if ((join_network != NULL) && (join_network[0] != '\0')) {
		/* write uart tx buffer */
		/*memset(tempdisp,' ',128);*/
		clearScreen(3);
		sleep(2);
		if (uart1_filestream != -1) {
			int count = write(uart1_filestream, join_network,134);
			if (count < 0) {
				printf("UART TX error\n");
			}
				printf(" bytes written : %s\n", join_network);


		}
	}


	if (uart0_filestream != -1) {

		int j ;
		/*	  ----- MODEM CONFIGURATION -----*//* read commands from  json file or default*/
		Join_Network: for (j = 0; j <= (sizeof(at_cmds)/sizeof(at_cmds[0]))-1; j++) {

			int count = write(uart0_filestream, at_cmds[j], strlen(at_cmds[j]));
			if (count < 0) {
				printf("UART 2 TX error\n");
			}
			if(strcmp(at_cmds[j],"AT+JOIN\n") == 0){
				printf("	/*  (modem config) = AT+JOIN  */\n");
				sleep(10);
			}else
				sleep(1);

			/*	   Read up to 255 characters from the port if they are there*/

			unsigned char rx_buffer[256];
			int rx_length = read(uart0_filestream, (void*) rx_buffer, 255);/*  Filestream, buffer to store in, number of bytes to read (max)*/
			if (rx_length < 0) {

			}else if (rx_length == 0) {
				/*  No data waiting*/
				printf("	/*  No RX data (modem config)*/\n");
			}else {
				/*  Bytes received*/
				rx_buffer[rx_length] = '\0';
				printf("%i bytes read : %s\n", rx_length, rx_buffer);

				/* Join network success check */
				if(strcmp(at_cmds[j],"AT+JOIN\n") == 0){
					printf("	/*  (modem config) = AT+JOIN  */\n");
					if(strstr(rx_buffer,"+JOIN: Join failed")!=NULL){

						clearScreen(3);
						sleep(2);
						if ((join_network_fail != NULL) && (join_network_fail[0] != '\0')) {
							/* write uart tx buffer */
							/*memset(tempdisp,' ',128);*/
							if (uart1_filestream != -1) {
								int count = write(uart1_filestream, join_network_fail,134);
								if (count < 0) {
									printf("UART TX error\n");
								}
									printf(" bytes written : %s\n", join_network_fail);


							}
						}

						network_joined=false;
						goto Join_Network;
					}else if(strstr(rx_buffer,"++JOIN: LoRaWAN modem is busy")!=NULL){

						clearScreen(3);
						sleep(2);
						if ((join_network_modem_busy != NULL) && (join_network_modem_busy[0] != '\0')) {
							/* write uart tx buffer */
							/*memset(tempdisp,' ',128);*/
							if (uart1_filestream != -1) {
								int count = write(uart1_filestream, join_network_modem_busy,134);
								if (count < 0) {
									printf("UART TX error\n");
								}
									printf(" bytes written : %s\n", join_network_modem_busy);


							}
						}


						network_joined=false;
						goto Join_Network;
					}else {

						clearScreen(3);
						sleep(2);
						if ((join_network_success != NULL) && (join_network_success[0] != '\0')) {
							/* write uart tx buffer */
							/*memset(tempdisp,' ',128);*/
							if (uart1_filestream != -1) {
								int count = write(uart1_filestream, join_network_success,134);
								if (count < 0) {
									printf("UART TX error\n");
								}
									printf(" bytes written : %s\n", join_network_success);


							}
						}


						network_joined=true;
					}
				}
			}
		}

		/* start command */

		if ((start_tx_data != NULL) && (start_tx_data[0] != '\0')) {
			int wr;
			for(wr=0;wr<=(sizeof(start_tx_data)/sizeof(start_tx_data[0]))-1;wr++){
				int count = write(uart0_filestream, start_tx_data[wr], strlen(start_tx_data[wr]));
									if (count < 0) {
										printf("UART 2 TX error\n");
									}
			}

			memset(start_tx_data, '\0', sizeof(start_tx_data));
		}



	}


	/* keep alive */

	/*		  ----- create thread for tx receive data ----- */
		err = pthread_create(&(keep_alive_t), NULL, &keep_alive, NULL);
		if (err != 0){
			printf("\ncan't create thread :[%s]", strerror(err));
		}


	for(;;){

			/* check any tx data for lora with mutex lock*/
/*
			pthread_mutex_lock(&tx_buf_data_lock);
			if ((tx_data != NULL) && (tx_data[0] != '\0')) {
				int wr;
				for(wr=0;wr<=(sizeof(tx_data)/sizeof(tx_data[0]))-1;wr++){
					int count = write(uart0_filestream, tx_data[wr], strlen(tx_data[wr]));
										if (count < 0) {
											printf("UART 2 TX error\n");
										}
				}

			memset(tx_data, '\0', sizeof(tx_data));
			}


			pthread_mutex_unlock(&tx_buf_data_lock);
*/



		pthread_mutex_lock(&tx_startkeep_alive_data_lock);

		if(send_keep_alive){

			if (counter_keep_alive>max_try_keep_alive){

				pthread_mutex_lock(&tx_display_buf_data_lock);
			    memset(dis_buf,0,134);
			    dis_buf[134]='3F';
			    strncpy(dis_buf, pre_dis_buf, 134);
			    pthread_mutex_unlock(&tx_display_buf_data_lock);

			}


			int count = write(uart0_filestream, tx_keep_alive,	strlen(tx_keep_alive));
			if (count < 0) {
				printf("UART 2 TX error\n");
			}
			sleep(2);
			send_keep_alive=false;
			keep_alive_trnfd=true;
		}

		pthread_mutex_unlock(&tx_startkeep_alive_data_lock);

		if (uart0_filestream != -1) {
		      int rx_length = read(uart0_filestream, (void*) rx_buffer, 255);/*  Filestream, buffer to store in, number of bytes to read (max)*/
				if (rx_length < 0) {

					printf(" * \n");

				} else if (rx_length == 0) {
					/*  No data waiting*/
					printf("	/*  No data received*/\n");
				} else {
					/* process received buffer*/
/*					rx_buffer[rx_length] = '\0'; */
					printf("%i bytes read : %s\n", rx_length, rx_buffer);


					if(incompl_dta){

						strcat(tmp_pr_rx_buffer, rx_buffer);
						memset(rx_buffer,0,sizeof(rx_buffer));
						strncpy(rx_buffer, tmp_pr_rx_buffer,sizeof(tmp_pr_rx_buffer));
						if (strstr(rx_buffer, rx_ch) != NULL && strstr(rx_buffer,rx_rssi) !=NULL) {

						}else{

							if(keep_alive_trnfd){

								if (strstr(rx_buffer, rx_cmg_start) != NULL && strstr(rx_buffer,rx_cmg_tx) !=NULL) {
										/* content has data part*/
									if (strstr(rx_buffer,rx_cmg_ack_rcvd) != NULL) {
										/* ack received*/
										kepp_alive_ack=true;
										counter_keep_alive=0;

									}else if(strstr(rx_buffer, rx_cmg_wait_ack) != NULL){
										/* ack not received*/
										kepp_alive_ack=false;
										counter_keep_alive++;

										}
									keep_alive_trnfd=false;
									}
							}

							memset(tmp_pr_rx_buffer,0,sizeof(rx_buffer));
							memset(rx_buffer,0,sizeof(rx_buffer));
						}
						incompl_dta=false;
					}



					/* check the RX String for read the first char */
					if (strstr(rx_buffer, rx_ch) != NULL && strstr(rx_buffer,rx_rssi) !=NULL) {

						int substringLength;

						rx_p=strstr(rx_buffer, rx_ch);
						int position =  rx_p ? rx_p - rx_buffer : -1;
						if(position>=0){
							substringLength = strlen(rx_buffer) - position;
							memset(tmp_rx_buffer,' ',substringLength+1);
							strncpy(tmp_rx_buffer, &rx_buffer[position], substringLength+1);
							tmp_rx_buffer[substringLength+1] = '\0';
							strncpy(rx_buffer, tmp_rx_buffer,sizeof(tmp_rx_buffer));
							memset(tmp_rx_buffer,' ',134);
						}

						int indx = rx_p ? rx_p - rx_buffer : -1;

						rx_p = strchr (rx_buffer,'"');
						int ch_pos=rx_p-rx_buffer+1;

						printf ("Character \" is found at position %d\n",rx_p-rx_buffer+1);

						int k=0;
						for (i = ch_pos; rx_buffer[i] != '\0'; ++i) {

						/*	printf("  %c ", rx_buffer[i]);*/

							tmp_rx_buffer[k]=rx_buffer[i];
							printf("  %c ", tmp_rx_buffer[k]);
							k++;
						}
						printf("%s \n",tmp_rx_buffer);
						rx_p = strchr (tmp_rx_buffer,'"');
						ch_pos=rx_p-tmp_rx_buffer+1;
						printf ("Character \" is found at position %d\n",rx_p-tmp_rx_buffer+1);

						if(ch_pos>=0){
						pthread_mutex_lock(&tx_display_buf_data_lock);

						memset(dis_buf,0, sizeof(dis_buf));
						printf("\n");
						for (i = 0; i <ch_pos-1; ++i) {
								printf("  %c ", rx_buffer[i]);
							dis_buf[i]=tmp_rx_buffer[i];

							printf("  %c ", dis_buf[i]);

						}
						printf("\n");
						memset(pre_dis_buf,0,134);
						strncpy(pre_dis_buf, dis_buf, 134);

					/*	pthread_cond_signal(&tx_data_recvd);*/
						dis_buffer_ready=true;
						pthread_mutex_unlock(&tx_display_buf_data_lock);
						incompl_dta=false;
						}
						memset(tmp_rx_buffer,0,sizeof(tmp_rx_buffer));
						memset(rx_buffer,0,sizeof(rx_buffer));
						counter_keep_alive=0;
					}else{

						if(keep_alive_trnfd){

							if (strstr(rx_buffer, rx_cmg_start) != NULL && strstr(rx_buffer,rx_cmg_tx) !=NULL) {
									/* content has data part*/
								if (strstr(rx_buffer,rx_cmg_ack_rcvd) != NULL) {
									/* ack received*/
									kepp_alive_ack=true;
									counter_keep_alive=0;
								}else if(strstr(rx_buffer, rx_cmg_wait_ack) != NULL){
									/* ack not received*/
									kepp_alive_ack=false;
									counter_keep_alive++;
									}
								keep_alive_trnfd=false;
								}
						}
						incompl_dta=true;
						strncpy(tmp_pr_rx_buffer, rx_buffer,sizeof(rx_buffer));

						printf(" Data partially recvd :\n");
					}

				}

			}
			sleep(1);

	}

	close(uart0_filestream);
return 0;
}

void clearScreen(int line){
	/* line : 1 top, 2 : bottom, 3 : both */
	char temp_clr1[134]= {"L S T                                                                                                                                 "};
	char temp_clr2[134]= {"L S B                                                                                                                                 "};
	char temp_clr3[134]= {"L B T                                                                                                                                 "};
/* 	char tempdisp[128]=	 {"L S T F l e x i   S i g n    intializing                                                                                              "};*/

	char * str_clr;
	switch(line) {

	case 1:
		str_clr=temp_clr1;
		break;
	case 2:
		 str_clr=temp_clr2;
		 break;
	case 3:
		 str_clr=temp_clr3;
		 break;
	}


	if (uart1_filestream != -1) {

		int count = write(uart1_filestream, str_clr,134);

		if (count < 0) {
			printf("UART TX error\n");
		}

		printf(" bytes written : %s\n", str_clr);

	}

}

void *display_uart(void *arg){
    int i;
    char buf = 0;
/*	char tempdisp[134]=	{"L B T H E L O O   W O R L D A A A 4 5 6 7 8 9 10 11       AAAAAAAAAAAABAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAABAAA"};*/
	char tempdisp[134]=	{"4C53424142                                                                                                                            "};

	 char disp_buf[134]={0};

	sleep(15);
	for(;;){

/*	 	clearScreen(3);*/

		sleep(1);
		pthread_mutex_lock(&tx_display_buf_data_lock);
/*		pthread_cond_wait(&tx_data_recvd, &tx_display_buf_data_lock);*/
		if ((dis_buf != NULL) && (dis_buf[0] != '\0')) {

			memset(disp_buf,'\0', sizeof(dis_buf));
		/*	 for(i = 0; i < sizeof(dis_buf)-1; i++){

				 disp_buf[i]=dis_buf[i];

			 }*/

			 int i,j;
			    char *output=dis_buf;
			    for (i = 0, j = 0; i<strlen(dis_buf); i++,j++)
			    {
			        if (dis_buf[i]!=' ')
			            output[j]=dis_buf[i];
			        else
			            j--;
			    }
			    output[j]=0;



				int length = strlen(output);
				j=0;
				/*convert HEX to ASCII char*/
	            for(i = 0; i < length; i++){
	            	if(i % 2 != 0){
	                   printf("%c", hex_to_ascii(buf, output[i]));

	                   disp_buf[j]=hex_to_ascii(buf, output[i]);
	                   j++;

	            	}else{
	                    buf = output[i];
	            	}
	            }


	            if(strcmp(disp_buf,"CLR1")==0){

	            	clearScreen(1);

	            }else if(strcmp(disp_buf,"CLR2")==0){

	            	clearScreen(2);

	            }else if(strcmp(disp_buf,"CLR3")==0){

	            	clearScreen(3);

	            }else{


	        		memset(&dis_buf[0], 0, sizeof(dis_buf));


	        		char witsp_output[134]={0};
	        		memset(witsp_output,' ',134);
					for (i = 0, j = 0; i<strlen(disp_buf); i++)
					{
						if (disp_buf[i]==' '){
							witsp_output[j]=disp_buf[i];
							witsp_output[j+1]=disp_buf[i];
							witsp_output[j+2]=disp_buf[i];
							witsp_output[j+3]=disp_buf[i];
							j=j+4;
						}else{
							printf(" %c\n",disp_buf[i]);
							witsp_output[j]=disp_buf[i];
							witsp_output[j+1]=' ';
							j=j+2;
						}

/*						if (disp_buf[i]!=' ')
							witsp_output[j]=disp_buf[i];
						else
							j--;*/
					}

					witsp_output[j]=0;



	        		if ((witsp_output != NULL) && (witsp_output[0] != '\0')) {
	            		/* write uart tx buffer */
	            		/*memset(tempdisp,' ',128);*/
	            		if (uart1_filestream != -1) {
	            			int count = write(uart1_filestream, witsp_output,134);
	            			if (count < 0) {
	            				printf("UART TX error\n");
	            			}
	            				printf(" bytes written : %s\n", witsp_output);


	            		}
	        		}


	            }
	        	memset(&dis_buf[0], 0, sizeof(dis_buf));

	        	memset(disp_buf,0, 134);

		}
		dis_buffer_ready=false;
		pthread_mutex_unlock(&tx_display_buf_data_lock);
	    printf("	/*  data waiting*/\n");

	/*//----- CLOSE THE UART -----*/

	}
	close(uart1_filestream);
return 0;
}

void *keep_alive(void *arg){

for(;;){

	sleep(keep_alive_interval);

	pthread_mutex_lock(&tx_startkeep_alive_data_lock);
	send_keep_alive=true;
	pthread_mutex_unlock(&tx_startkeep_alive_data_lock);

}
	/*//----- CLOSE THE UART -----*/

	return 0;

}

int flextec_discon_configuration(const char * conf_file) {

	const char conf_obj_name[] = "discont_conf";
	JSON_Value *root_val;
	JSON_Object *conf_obj = NULL;
	JSON_Array *array_val;

	/* try to parse JSON */
	root_val = json_parse_file_with_comments(conf_file);
	if (root_val == NULL) {
		DISPLAYLOG("ERROR: %s is not a valid JSON file\n", conf_file);
		exit(EXIT_FAILURE);
	}

	/* point to the gateway configuration object */
	conf_obj = json_object_get_object(json_value_get_object(root_val),
			conf_obj_name);
	if (conf_obj == NULL) {
		DISPLAYLOG("INFO: %s does not contain a JSON object named %s\n", conf_file,
				conf_obj_name);
		return -1;
	} else {
		DISPLAYLOG(
				"INFO: %s does contain a JSON object named %s, parsing gateway parameters\n",
				conf_file, conf_obj_name);
	}


	/* validate configuration setup*/

	/* at commands */
	array_val = json_object_get_array(conf_obj, "lora_at");
	 if (array_val != NULL) {
		 	 memset(at_cmds, '\0', sizeof(at_cmds));
	        for (i = 0; i < json_array_get_count(array_val); i++) {
/*	            printf("%s \n",json_array_get_string(array_val, i));*/
	          /*  json_array_get_string(array_val, i);*/
	            at_cmds[i]= malloc(strlen(json_array_get_string(array_val, i))+1);
	            strcpy(at_cmds[i],json_array_get_string(array_val, i));
	        }
	 }

	 /* initial dispaly */
/*		array_val = json_object_get_array(conf_obj, "display_init");
		 if (array_val != NULL) {
			 	 memset(tx_data, '\0', sizeof(tx_data));
		        for (i = 0; i < json_array_get_count(array_val); i++) {
		            printf("%s \n",json_array_get_string(array_val, i));
		            json_array_get_string(array_val, i);
		            tx_data= malloc(strlen(json_array_get_string(array_val, i))+1);
		            strcpy(tx_data,json_array_get_string(array_val, i));
		        }
		 }*/


/*	    if (array_val != NULL && json_array_get_count(array_val) > 1) {

	    	printf("%s \n",json_array_get_string(array_val, 0));
	    	printf("%s \n",json_array_get_string(array_val, 1));


	    }*/

/*	val = json_object_get_value(conf_obj, "isSetup");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isSetup = (bool) json_value_get_boolean(val);
				gw_setup = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway configure value  %s \n",
				gw_setup ? "true\n" : "false\n");
	}

		val = json_object_get_value(conf_obj, "flextech_gw_ver");
	 if (json_value_get_type(val) == JSONBoolean) {
	 gateway.flextech_gw_ver==
	 gw_setup = (bool) json_value_get_boolean(val);
	 DISPLAYLOG("INFO: Flextec gateway configure value  %s \n", gw_setup ? "true\n" : "false\n");
	 }

	val = json_object_get_value(conf_obj, "isDisplay");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isDisplay = (bool) json_value_get_boolean(val);
				gw_display = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_display value  %s \n",
				gw_display ? "true\n" : "false\n");
	}

	val = json_object_get_value(conf_obj, "isLan");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isLan = (bool) json_value_get_boolean(val);
					gw_lan = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_lan value  %s \n",
				gw_lan ? "true\n" : "false\n");
	}

	val = json_object_get_value(conf_obj, "isWan");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isWan = (bool) json_value_get_boolean(val);
				gw_wan = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_wan value  %s \n",
				gw_wan ? "true\n" : "false\n");

	}

	val = json_object_get_value(conf_obj, "isWifi");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isWifi = (bool) json_value_get_boolean(val);
				gw_wifi = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_wifi value  %s \n",
				gw_wifi ? "true\n" : "false\n");
	}

	val = json_object_get_value(conf_obj, "isLoraBridge");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isLoraBridge = (bool) json_value_get_boolean(val);
				gw_lorabridge = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_wifi value  %s \n",
				gw_lorabridge ? "true\n" : "false\n");
	}

	val = json_object_get_value(conf_obj, "isDebug");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.isDebug = (bool) json_value_get_boolean(val);
				gw_debug = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_wifi value  %s \n",
				gw_debug ? "true\n" : "false\n");
	}

	val = json_object_get_value(conf_obj, "isGSM_CDMA");
	if (json_value_get_type(val) == JSONBoolean) {
		gateway.gwconf.gw_2g_3g = (bool) json_value_get_boolean(val);
				gw_2g_3g = (bool) json_value_get_boolean(val);
		DISPLAYLOG("INFO: Flextec gateway gw_wifi value  %s \n",
				gw_2g_3g ? "true\n" : "false\n");
	}
*/

/*
	str = json_object_get_string(conf_obj, "display");
		if (str != NULL) {
			strncpy(modem_path, str, sizeof modem_path);
			DISPLAYLOG("INFO: GPS serial port path is configured to \"%s\"\n",
					modem_path);
		}
*/

	json_value_free(root_val);
	return 0;
}

int hex_to_int(char c){
        int first = c / 16 - 3;
        int second = c % 16;
        int result = first*10 + second;
        if(result > 9) result--;
        return result;
}

int hex_to_ascii(char c, char d){
        int high = hex_to_int(c) * 16;
        int low = hex_to_int(d);
        return high+low;
}


