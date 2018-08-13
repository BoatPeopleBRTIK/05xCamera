#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "MQTTClient.h"
//#include <memory.h>
//#include <conio.h>
//#include <graphics.h>

//#define ADDRESS     "10.0.1.163"
#define CLIENTID    "ExampleClientSub"
#define TOPIC       "camera"
#define PAYLOAD     "Hello World!"
#define QOS         1
#define TIMEOUT     10000L

#define DB		printf("Line: %d\n",li++)

int li;

////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t line_arr[38400];
uint8_t *line_ptr;

static void line_array(char *payload_ptr, uint32_t payload_len,uint8_t *u8_arr){
	
	char temp;
	uint8_t temp_2;
	uint32_t i;
	uint8_t zero;
	zero=0;
	FILE* wrImg;
	wrImg = fopen("mqtt_recd_frame.yuv","wb");

	if (wrImg ){
		fwrite(payload_ptr,1,payload_len,wrImg);
		printf("\nmqtt_recd_frame.yuv written\n"); }
    	else{printf("Can't open file");}

	fclose(wrImg);
}

void print_lines(void){
		
		uint16_t i = 0;
		uint16_t j = 0;
		printf("\n\na = [");
		for(i=0;i<119;i++){
			printf("[");
			for(j = 0;j<311;j++){
				printf("0x%x,",line_arr[312*i + j]);	
			}
			printf("0x%x],",line_arr[312*i + j]);
		}
		printf("[");
		for(j = 0;j<311;j++){
			printf("0x%x,",line_arr[312*(i) + j]);	
		}
		printf("0x%x]]",line_arr[312*(i) + j]);

	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile MQTTClient_deliveryToken deliveredtoken;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message){

    int i;
    char* payloadptr;
    char temp;
    uint8_t temp_2;

    payloadptr = message->payload;

///////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////

    // printf("\nPayload length: %d\n\n\n",message->payloadlen);
    //printf("\n\npayload pointer start: %x\tVAL: %x\n\n",payloadptr, (*payloadptr));

    line_ptr = line_arr;
    line_array(message->payload,message->payloadlen,line_ptr);

    //print_lines();
    //printf("\n\npayload pointer end: %x\tVAL: %x\n\n",payloadptr, *payloadptr);	
   
    putchar('\n');
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;

}

void connlost(void *context, char *cause){

    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
}
int main(int argc, char* argv[])
{
    li = 0;
	
    //printf("\nargc - %d\n",argc);
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;
    int ch;

    MQTTClient_create(&client, argv[1], CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }   

    MQTTClient_subscribe(client, TOPIC, QOS);

    do
    {
        ch = getchar();
    } while(ch!='Q' && ch != 'q');
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);

    return rc;
}
