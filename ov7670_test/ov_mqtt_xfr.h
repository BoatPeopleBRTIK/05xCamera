#ifndef OV_MQTT_XFR_H_
#define OV_MQTT_XFR_H_
////////////////////////////////////// MQTT PUBLISH ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

#include "wifi.h"
#include <apps/netutils/mqtt_api.h>
#include <apps/netutils/dhcpc.h>

#include <stdbool.h>

#include <tinyara/config.h>
#include <tinyara/gpio.h>
#include <sys/types.h>
#include <tinyara/clock.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <tinyara/i2c.h>

#include "util_ov7670.h"

/*
 * main.c
 *
 *  Created on: Jun 19, 2018
 *      Author: shivam.k
 */



#define DEFAULT_CLIENT_ID "123456789"
#define SERVER_ADDR "10.0.1.163"

#define RED_ON_BOARD_LED 45
#define NET_DEVNAME "wl1"

// mqtt client handle
mqtt_client_t* pClientHandle = NULL;

// mqtt client parameters
mqtt_client_config_t clientConfig;

//int blinkerValue = 0;
int currentLED = 0;

//uint8_t y_arr[20000];

// mqtt client on connect callback
void onConnect(void* client, int res) {
    printf("mqtt client connected to the server\n");
}


// mqtt client on disconnect callback
void onDisconnect(void* client, int res) {
    printf("mqtt client disconnected from the server\n");
}

// mqtt client on publish callback
void onPublish(void* client, int res) {
   printf("mqtt client Published message\n");
}

// Utility function to configure mqtt client
void initializeConfigUtil(void) {
    uint8_t macId[IFHWADDRLEN];
    clientConfig.protocol_version = MQTT_PROTOCOL_VERSION_31;
    int res = netlib_getmacaddr("wl1", macId);
    if (res < 0) {
        printf("Get MAC Address failed. Assigning \
                Client ID as 123456789");
        clientConfig.client_id =
                DEFAULT_CLIENT_ID; // MAC id Artik 053
    } else {
    printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
            ((uint8_t *) macId)[0],
            ((uint8_t *) macId)[1], ((uint8_t *) macId)[2],
            ((uint8_t *) macId)[3], ((uint8_t *) macId)[4],
            ((uint8_t *) macId)[5]);
    char buf[12];
    sprintf(buf, "%02x%02x%02x%02x%02x%02x",
            ((uint8_t *) macId)[0],
            ((uint8_t *) macId)[1], ((uint8_t *) macId)[2],
            ((uint8_t *) macId)[3], ((uint8_t *) macId)[4],
            ((uint8_t *) macId)[5]);
    clientConfig.client_id = buf; // MAC id Artik 053
    printf("Registering mqtt client with id = %s\n", buf);
    }
    clientConfig.on_connect = (void*) onConnect;
    clientConfig.on_disconnect = (void*) onDisconnect;
    clientConfig.on_publish = (void*) onPublish;
}

static void mqtt_in(char* server_addr) {

    bool wifiConnected = false;
    //gpio_write(RED_ON_BOARD_LED, 1); // Turn on on board Red LED to indicate no WiFi connection is established

#ifdef CONFIG_CTRL_IFACE_FIFO
    int ret;

    while(!wifiConnected) {
        ret = mkfifo(CONFIG_WPA_CTRL_FIFO_DEV_REQ,
                CONFIG_WPA_CTRL_FIFO_MK_MODE);
        if (ret != 0 && ret != -EEXIST) {
            printf("mkfifo error for %s: %s",
                    CONFIG_WPA_CTRL_FIFO_DEV_REQ,
                    strerror(errno));
        }
        ret = mkfifo(CONFIG_WPA_CTRL_FIFO_DEV_CFM,
                CONFIG_WPA_CTRL_FIFO_MK_MODE);
        if (ret != 0 && ret != -EEXIST) {
            printf("mkfifo error for %s: %s",
                    CONFIG_WPA_CTRL_FIFO_DEV_CFM,
                    strerror(errno));
        }

        ret = mkfifo(CONFIG_WPA_MONITOR_FIFO_DEV,
                CONFIG_WPA_CTRL_FIFO_MK_MODE);
        if (ret != 0 && ret != -EEXIST) {
            printf("mkfifo error for %s: %s",
                    CONFIG_WPA_MONITOR_FIFO_DEV,
                    strerror(errno));
        }
    #endif

        if (start_wifi_interface() == SLSI_STATUS_ERROR) {
            printf("Connect Wi-Fi failed. Try Again.\n");
        }
        else {
            wifiConnected = true;
            //gpio_write(RED_ON_BOARD_LED, 0); // Turn off Red LED to indicate WiFi connection is established
        }
    }

    printf("Connect to Wi-Fi success\n");

    bool mqttConnected = false;
    bool ipObtained = false;
    printf("Get IP address\n");

    struct dhcpc_state state;
    void *dhcp_handle;

    while(!ipObtained) {
        dhcp_handle = dhcpc_open(NET_DEVNAME);
        ret = dhcpc_request(dhcp_handle, &state);
        dhcpc_close(dhcp_handle);

        if (ret != OK) {
            printf("Failed to get IP address\n");
            printf("Try again\n");
            sleep(1);
        }
        else {
            ipObtained = true;
        }
    }
    netlib_set_ipv4addr(NET_DEVNAME, &state.ipaddr);
    netlib_set_ipv4netmask(NET_DEVNAME, &state.netmask);
    netlib_set_dripv4addr(NET_DEVNAME, &state.default_router);

    printf("IP address  %s\n", inet_ntoa(state.ipaddr));

    sleep(5);

    // Connect to the WiFi network for Internet connectivity
    printf("mqtt client tutorial\n");

    // Initialize mqtt client
    initializeConfigUtil();

    pClientHandle = mqtt_init_client(&clientConfig);
    if (pClientHandle == NULL) {
        printf("mqtt client handle initialization fail\n");
        //return 0;
    }

    while (mqttConnected == false ) {
        sleep(2);
        // Connect mqtt client to server
        int re = mqtt_connect(pClientHandle,
                server_addr, 1883, 60);
        if (re < 0) {
            printf("mqtt client connect to server fail\n");
            continue;
        }
        mqttConnected = true;
    }

	   // bool mqttSubscribe = false;
	/*
	    // Subscribe to topic of interest
	    while (mqttSubscribe == false ) {
		sleep(2);
		int resu = mqtt_subscribe(pClientHandle,
		        "color", 0); //topic - color, QOS - 0
		if (resu < 0) {
		    printf("mqtt client subscribe to topic \
		            failed\n");
		    continue;
		}
		mqttSubscribe = true;
		printf("mqtt client Subscribed to the topic successfully\n");
	    }
	*/
}

static void mqtt_pub(uint8_t* buff_ptr, uint32_t buff_len){
	int ret;
    	int ret_check;
    	ret_check = 0;
    	while(!ret_check) {
       
	printf("\nSize of temp_buff: %d\n",buff_len);    

                    char* msg = (char*)(buff_ptr);
                    // construct the mqtt message to be published
                    mqtt_msg_t message;
                    message.payload = msg;
                    message.payload_len = buff_len;
                    message.topic = "camera";
                    message.qos = 0;
                    message.retain = 0;
                    ret = mqtt_publish(pClientHandle, message.topic, (char*)message.payload, message.payload_len, message.qos, message.retain);
                    if (ret < 0) {
                        printf("Error publishing \n");
                    }
                    else {
                        ret_check = 1;
                    }
         }
}


#endif


//////////////////////////////////////////////////////////////////////////////////////
