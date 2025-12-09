/*
 * wifi_app.c
 *
 *  Created on: Oct 17, 2021
 *      Author: kjagu
 */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/netdb.h"
#include "nvs.h"

#include "http_server.h"
#include "rgb_led.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "esp_sntp.h"
#include "esp_netif.h"

// Tag used for ESP serial console messages
static const char TAG [] = "wifi_app";

SemaphoreHandle_t mySemaphore;

register_saved_e register_readings_from_flash [NUM_REGISTERS_AV];// registers

// Used for returning the WiFi configuration
wifi_config_t *wifi_config = NULL;

// Used to track the number for retries when a connection attempt fails
static int g_retry_number;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t wifi_app_queue_handle;

// netif objects for the station and access point
esp_netif_t* esp_netif_sta = NULL;
esp_netif_t* esp_netif_ap  = NULL;

bool time_was_synchronized;

extern uint8_t s_led_state;








void init_obtain_time( void ){
	time_was_synchronized = false;
}

bool get_state_time_was_synchronized( void ){
	return time_was_synchronized;
}


static void obtain_time(void)
{	
	setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    ESP_LOGI(TAG, "Initializing SNTP");
    // Configurar servidores SNTP usando la API moderna; incluir IPs evita problemas de DNS en hotspots
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "216.239.35.0");   // time.google.com (IP)
    esp_sntp_setservername(1, "162.159.200.1");  // cloudflare ntp (IP)
    esp_sntp_setservername(2, "pool.ntp.org");   // dominio como respaldo
    esp_sntp_init();

    // Esperar a que se sincronice el tiempo con el servidor SNTP
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry < retry_count)
    {
        ESP_LOGI(TAG, "System time is set!");
		time_was_synchronized = true;
    }
    else
    {
        ESP_LOGE(TAG, "Unable to set system time. Check your SNTP configuration.");
    }
}

void save_wifi_credentials(const char *ssid, const char *password) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "wifi_ssid", ssid));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "wifi_password", password));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

void save_reg_data(uint8_t register_num, char *str) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
	
	if ( register_num == 1 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg01", str));
	}
	else if ( register_num == 2 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg02", str));
	}
	else if ( register_num == 3 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg03", str));
	}
	else if ( register_num == 4 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg04", str));
	}
	else if ( register_num == 5 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg05", str));
	}
	else if ( register_num == 6 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg06", str));
	}
	else if ( register_num == 7 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg07", str));
	}
	else if ( register_num == 8 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg08", str));
	}
	else if ( register_num == 9 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg09", str));
	}
	else if ( register_num == 10 ){
		ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "reg10", str));
	}

	esp_err_t err;
	err = nvs_commit(nvs_handle);
	if (err == ESP_OK) {
		printf("information saved\n");
	}
    else if (err != ESP_OK) {
        printf("Error al confirmar cambios\n");
    }


    nvs_close(nvs_handle);
}


esp_err_t read_reg_data(char *str_to_save ,uint8_t register_num){

	nvs_handle_t nvs_handle;
    esp_err_t err;

    // Abrir el NVS
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        // Manejar el error
        return ESP_FAIL;
    }

    // Tamaño de la cadena que se espera leer
    size_t str_len = 0;
	char reg_to_send[6];
	const char *ptr_const_char = reg_to_send;
	memset(&reg_to_send[0], 0x00, 6);
	
	if ( register_num == 1 ){
		strcpy(reg_to_send, "reg01");
			
	}
	else if ( register_num == 2 ){
		strcpy(reg_to_send, "reg02");
				
	}
	else if ( register_num == 3 ){
		strcpy(reg_to_send, "reg03");
			
	}
	else if ( register_num == 4 ){
		strcpy(reg_to_send, "reg04");	
		
	}
	else if ( register_num == 5 ){
		strcpy(reg_to_send, "reg05");
			
	}
	else if ( register_num == 6 ){
		strcpy(reg_to_send, "reg06");
			
	}
	else if ( register_num == 7 ){
		strcpy(reg_to_send, "reg07");	
		
	}
	else if ( register_num == 8 ){
		strcpy(reg_to_send, "reg08");
			
	}
	else if ( register_num == 9 ){
		strcpy(reg_to_send, "reg09");	
		
	}
	else if ( register_num == 10 ){
		strcpy(reg_to_send, "reg10");	
		
	}


	

	
	size_t required_size;
		// Get the size of the stored string
		esp_err_t erras;
    	erras = nvs_get_str(nvs_handle, ptr_const_char, NULL, &required_size);
		if (erras != ESP_OK) {
			// Silenciar ausencia de registro
			nvs_close(nvs_handle);
			return ESP_FAIL;
    	}
		// Allocate memory for wifi_ssid

    	char *reg_buffer = malloc(required_size);
    	if (reg_buffer == NULL) {
    	    // Handle memory allocation error
    	    ESP_LOGE(TAG, "Failed to allocate memory for wifi_ssid");
    	    nvs_close(nvs_handle);
    	    return ESP_FAIL;
    	}
    	// Get reg
    	erras = nvs_get_str(nvs_handle, ptr_const_char, reg_buffer, &required_size);
    	strncpy(str_to_save, reg_buffer, required_size);
		ESP_LOGI(TAG, "Valor leído para la clave '%s': %s", reg_to_send, str_to_save);
    	free(reg_buffer);

		
		
            
/*

		err = nvs_get_str(nvs_handle, ptr_const_char, str_to_save, 11);
		if (err == ESP_OK) {
			// Imprimir la cadena leída
			printf("Valor leído: %s\n", str_to_save);
		} else {
			// Manejar el error al leer
			fprintf(stderr, "Error al leer el valor desde el NVS.\n");
		}		
*/

    // Cerrar el NVS
    nvs_close(nvs_handle);
	return ESP_OK;


}
void update_register(int reg_to_update){
	char register_information_read[12];
	register_information_read[11] = 0x00;
	char hora_min_str[3];
	hora_min_str[2] = 0x00;
	char day[2];
	day[1] = 0x00;
	register_information_read[11] = 0x00;
	if ( read_reg_data( &register_information_read[0], reg_to_update ) == ESP_OK ){
		
		strncpy(&hora_min_str[0], &register_information_read[0], 2);
		register_readings_from_flash[reg_to_update-1].hour = atoi(hora_min_str);
		
		strncpy(&hora_min_str[0], &register_information_read[2], 2);
		register_readings_from_flash[reg_to_update-1].min = atoi(hora_min_str);

		strncpy(&day[0], &register_information_read[4], 1);
		register_readings_from_flash[reg_to_update-1].monday = atoi(day);

		strncpy(&day[0], &register_information_read[5], 1);
		register_readings_from_flash[reg_to_update-1].tuesday = atoi(day);

		strncpy(&day[0], &register_information_read[6], 1);
		register_readings_from_flash[reg_to_update-1].wednesday = atoi(day);

		strncpy(&day[0], &register_information_read[7], 1);
		register_readings_from_flash[reg_to_update-1].thursday = atoi(day);

		strncpy(&day[0], &register_information_read[8], 1);
		register_readings_from_flash[reg_to_update-1].friday = atoi(day);

		strncpy(&day[0], &register_information_read[9], 1);
		register_readings_from_flash[reg_to_update-1].saturday = atoi(day);

		strncpy(&day[0], &register_information_read[10], 1);
		register_readings_from_flash[reg_to_update-1].sunday = atoi(day);

		ESP_LOGI(TAG, "hora: %d, min: %d, day0: %d, day1: %d, day2: %d, day3: %d, day4: %d, day5: %d, day6: %d", register_readings_from_flash[reg_to_update-1].hour, register_readings_from_flash[reg_to_update-1].min, register_readings_from_flash[reg_to_update-1].monday ,
		register_readings_from_flash[reg_to_update-1].tuesday , register_readings_from_flash[reg_to_update-1].wednesday , register_readings_from_flash[reg_to_update-1].thursday  , register_readings_from_flash[reg_to_update-1].friday 
		, register_readings_from_flash[reg_to_update-1].saturday, register_readings_from_flash[reg_to_update-1].sunday);
	}

	
}
void initialize_registers( void ){
	
	char register_information_read[12];
	register_information_read[11] = 0x00;
	char hora_min_str[3];
	hora_min_str[2] = 0x00;
	char day[2];
	day[1] = 0x00;
	
	
	for (int i = 0; i < NUM_REGISTERS_AV; i++){
		if ( read_reg_data( &register_information_read[0], i+1 ) == ESP_OK ){
			
			strncpy(&hora_min_str[0], &register_information_read[0], 2);
			register_readings_from_flash[i].hour = atoi(hora_min_str);
			
			strncpy(&hora_min_str[0], &register_information_read[2], 2);
			register_readings_from_flash[i].min = atoi(hora_min_str);

			strncpy(&day[0], &register_information_read[4], 1);
			register_readings_from_flash[i].monday = atoi(day);

			strncpy(&day[0], &register_information_read[5], 1);
			register_readings_from_flash[i].tuesday = atoi(day);

			strncpy(&day[0], &register_information_read[6], 1);
			register_readings_from_flash[i].wednesday = atoi(day);

			strncpy(&day[0], &register_information_read[7], 1);
			register_readings_from_flash[i].thursday = atoi(day);

			strncpy(&day[0], &register_information_read[8], 1);
			register_readings_from_flash[i].friday = atoi(day);

			strncpy(&day[0], &register_information_read[9], 1);
			register_readings_from_flash[i].saturday = atoi(day);

			strncpy(&day[0], &register_information_read[10], 1);
			register_readings_from_flash[i].sunday = atoi(day);

			//ESP_LOGI(TAG, "hora: %d, min: %d, day0: %d, day1: %d, day2: %d, day3: %d, day4: %d, day5: %d, day6: %d", register_readings_from_flash[i].hour, register_readings_from_flash[i].min, register_readings_from_flash[i].monday ,
			//register_readings_from_flash[i].tuesday , register_readings_from_flash[i].wednesday , register_readings_from_flash[i].thursday  , register_readings_from_flash[i].friday 
			//, register_readings_from_flash[i].saturday, register_readings_from_flash[i].sunday);
		}
		else{
			register_readings_from_flash[i].hour = 99;
			register_readings_from_flash[i].min = 99;
			register_readings_from_flash[i].monday = 0;
			register_readings_from_flash[i].tuesday = 0;
			register_readings_from_flash[i].wednesday = 0;
			register_readings_from_flash[i].thursday = 0;
			register_readings_from_flash[i].friday = 0;
			register_readings_from_flash[i].sunday = 0;
			register_readings_from_flash[i].saturday = 0;

		}

	}
}

void load_wifi_credentials(char *ssid, char *password) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &nvs_handle));

    size_t required_size;

    // Get the size of wifi_ssid
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle, "wifi_ssid", NULL, &required_size));
    // Allocate memory for wifi_ssid
    char *ssid_buffer = malloc(required_size);
    if (ssid_buffer == NULL) {
        // Handle memory allocation error
        ESP_LOGE(TAG, "Failed to allocate memory for wifi_ssid");
        nvs_close(nvs_handle);
        return;
    }
    // Get wifi_ssid
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle, "wifi_ssid", ssid_buffer, &required_size));
    // Copy wifi_ssid to the output parameter
    strncpy(ssid, ssid_buffer, required_size);

    // Repeat the process for wifi_password
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle, "wifi_password", NULL, &required_size));
    char *password_buffer = malloc(required_size);
    if (password_buffer == NULL) {
        // Handle memory allocation error
        ESP_LOGE(TAG, "Failed to allocate memory for wifi_password");
        free(ssid_buffer);
        nvs_close(nvs_handle);
        return;
    }
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle, "wifi_password", password_buffer, &required_size));
    strncpy(password, password_buffer, required_size);

    // Free the allocated memory
    free(ssid_buffer);
    free(password_buffer);

    nvs_close(nvs_handle);
}
bool nvs_credentials_exist() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }

    size_t ssid_size, password_size;
    err = nvs_get_str(nvs_handle, "wifi_ssid", NULL, &ssid_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_get_str(nvs_handle, "wifi_password", NULL, &password_size);
    nvs_close(nvs_handle);

    return err == ESP_OK;
}


void connect_to_wifi(void) {

	if (xSemaphoreTake(mySemaphore, portMAX_DELAY)) { // helpus to not allow multiple calls
 
    	char ssid[32];
    	char password[64];
    	load_wifi_credentials(&ssid[0], &password[0]);
		wifi_config_t* wifi_config = wifi_app_get_wifi_config();
		ESP_LOGI(TAG, "saved SSID: %s", ssid);
    	ESP_LOGI(TAG, "saved password: %s", password);

		//memset(wifi_config->sta.ssid, 0x00, sizeof(wifi_config->sta.ssid));
		//memset(wifi_config->sta.password, 0x00, sizeof(wifi_config->sta.password));
		memset(wifi_config, 0x00, sizeof(wifi_config_t));
   		strncpy((char*)wifi_config->sta.ssid, ssid, sizeof(wifi_config->sta.ssid));
    	strncpy((char*)wifi_config->sta.password, password, sizeof(wifi_config->sta.password));
   		esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config);
   		wifi_app_connect_sta();

	       // Do some work
        // Release the semaphore (give it back)
        xSemaphoreGive(mySemaphore);		

    }
}

void check_sta_connection_state( void *pvParameters ) {
	wifi_ap_record_t ap_info;
	esp_err_t ret;
	while(true){
		ret = esp_wifi_sta_get_ap_info(&ap_info);
		 ESP_LOGI(TAG, "Checking sta info");
		    if (ret == ESP_OK) {
				
        		if (ap_info.authmode != WIFI_AUTH_MAX) {
        		    ESP_LOGI(TAG, "Connected to SSID: %s", ap_info.ssid);
					if (get_state_time_was_synchronized() == false)
						obtain_time();

					
        		} else {
        		    ESP_LOGI(TAG, "Not connected to any WiFi network");
					if (nvs_credentials_exist()) {
						// Credentials exist, try to connect
						connect_to_wifi();
						ESP_LOGI(TAG, "CHECKING CONNECTION TO STA_BEFORE_SAVED");
					}

					//return false;
        		}
    		} else {
					if (nvs_credentials_exist()) {
							// Credentials exist, try to connect
							connect_to_wifi();
							ESP_LOGI(TAG, "CHECKING CONNECTION TO STA_BEFORE_SAVED");
						}
       			 ESP_LOGI(TAG, "Failed to get connection info");
					//return false;
        		}
		vTaskDelay(20000 / portTICK_PERIOD_MS);

	}
    
    
    // Get the connection info

}
/**
 * WiFi application event handler
 * @param arg data, aside from event data, that is passed to the handler when it is called
 * @param event_base the base id of the event to register the handler for
 * @param event_id the id fo the event to register the handler for
 * @param event_data event data
 */
static void wifi_app_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT)
	{
		switch (event_id)
		{
			case WIFI_EVENT_AP_START:
				ESP_LOGI(TAG, "WIFI_EVENT_AP_START");
				break;

			case WIFI_EVENT_AP_STOP:
				ESP_LOGI(TAG, "WIFI_EVENT_AP_STOP");
				break;

			case WIFI_EVENT_AP_STACONNECTED:
				ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED");
				break;

			case WIFI_EVENT_AP_STADISCONNECTED:
				ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
				break;

			case WIFI_EVENT_STA_START:
				ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
				break;

			case WIFI_EVENT_STA_CONNECTED:
				ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
				break;

			case WIFI_EVENT_STA_DISCONNECTED:
				ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");

				// wifi_event_sta_disconnected_t *wifi_event_sta_disconnected = (wifi_event_sta_disconnected_t*)malloc(sizeof(wifi_event_sta_disconnected_t));
				// *wifi_event_sta_disconnected = *((wifi_event_sta_disconnected_t*)event_data);
				// printf("WIFI_EVENT_STA_DISCONNECTED, reason code %d\n", wifi_event_sta_disconnected->reason);

				// if (g_retry_number < MAX_CONNECTION_RETRIES)
				// {
				// 	esp_wifi_connect();
				// 	g_retry_number ++;
				// }
				// else
				// {
				// 	wifi_app_send_message(WIFI_APP_MSG_STA_DISCONNECTED);
				// }

				break;
		}
	}
	else if (event_base == IP_EVENT)
	{
		switch (event_id)
		{
			case IP_EVENT_STA_GOT_IP:
				ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");

				wifi_app_send_message(WIFI_APP_MSG_STA_CONNECTED_GOT_IP);
				if (esp_netif_ap) {
					esp_err_t err = esp_netif_napt_enable(esp_netif_ap);
					ESP_LOGI(TAG, "NAPT enable on AP: %s", esp_err_to_name(err));
				}

				break;
		}
	}
}

/**
 * Initializes the WiFi application event handler for WiFi and IP events.
 */
static void wifi_app_event_handler_init(void)
{
	// Event loop for the WiFi driver
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// Event handler for the connection
	esp_event_handler_instance_t instance_wifi_event;
	esp_event_handler_instance_t instance_ip_event;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_wifi_event));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_ip_event));
}

/**
 * Initializes the TCP stack and default WiFi configuration.
 */
static void wifi_app_default_wifi_init(void)
{
	// Initialize the TCP stack
	ESP_ERROR_CHECK(esp_netif_init());

	// Default WiFi config - operations must be in this order!
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	esp_netif_sta = esp_netif_create_default_wifi_sta();
	esp_netif_ap = esp_netif_create_default_wifi_ap();
}

/**
 * Configures the WiFi access point settings and assigns the static IP to the SoftAP.
 */
static void wifi_app_soft_ap_config(void)
{
	// SoftAP - WiFi access point configuration
	wifi_config_t ap_config =
	{
		.ap = {
				.ssid = WIFI_AP_SSID,
				.ssid_len = strlen(WIFI_AP_SSID),
				.password = WIFI_AP_PASSWORD,
				.channel = WIFI_AP_CHANNEL,
				.ssid_hidden = WIFI_AP_SSID_HIDDEN,
				.authmode = WIFI_AUTH_WPA2_PSK,
				.max_connection = WIFI_AP_MAX_CONNECTIONS,
				.beacon_interval = WIFI_AP_BEACON_INTERVAL,
		},
	};

	// Configure DHCP for the AP
	esp_netif_ip_info_t ap_ip_info;
	memset(&ap_ip_info, 0x00, sizeof(ap_ip_info));

	esp_netif_dhcps_stop(esp_netif_ap);					///> must call this first
	inet_pton(AF_INET, WIFI_AP_IP, &ap_ip_info.ip);		///> Assign access point's static IP, GW, and netmask
	inet_pton(AF_INET, WIFI_AP_GATEWAY, &ap_ip_info.gw);
	inet_pton(AF_INET, WIFI_AP_NETMASK, &ap_ip_info.netmask);
	ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_netif_ap, &ap_ip_info));			///> Statically configure the network interface
	ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netif_ap));						///> Start the AP DHCP server (for connecting stations e.g. your mobile device)

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));						///> Setting the mode as Access Point / Station Mode
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));			///> Set our configuration
	ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH));		///> Our default bandwidth 20 MHz
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE));						///> Power save set to "NONE"

}

/**
 * Connects the ESP32 to an external AP using the updated station configuration
 */
static void wifi_app_connect_sta(void)
{
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_app_get_wifi_config()));
	ESP_ERROR_CHECK(esp_wifi_connect());
}

/**
 * Main task for the WiFi application
 * @param pvParameters parameter which can be passed to the task
 */
static void wifi_app_task(void *pvParameters)
{
	wifi_app_queue_message_t msg;

	// Initialize the event handler
	wifi_app_event_handler_init();

	// Initialize the TCP/IP stack and WiFi config
	wifi_app_default_wifi_init();

	// SoftAP config
	wifi_app_soft_ap_config();

	// Start WiFi
	ESP_ERROR_CHECK(esp_wifi_start());

	// Send first event message
	wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
	wifi_app_send_message(WIFI_APP_CONNECT_TO_STA);
	initialize_registers();  //Init registers of hours

	for (;;)
	{
		if (xQueueReceive(wifi_app_queue_handle, &msg, portMAX_DELAY))
		{
			switch (msg.msgID)
			{
				case WIFI_APP_CONNECT_TO_STA:
					//ESP_LOGI(TAG, "CHECKING CONNECTION TO STA");
					xTaskCreatePinnedToCore(&check_sta_connection_state, "check_sta_connection_state", WIFI_APP_TASK_STACK_SIZE, NULL, WIFI_APP_TASK_PRIORITY, NULL, WIFI_APP_TASK_CORE_ID);
	/*
					if (nvs_credentials_exist()) {
						// Credentials exist, try to connect
						connect_to_wifi();
						ESP_LOGI(TAG, "CHECKING CONNECTION TO STA_BEFORE_SAVED");
					}
					else{
						ESP_LOGI(TAG, "NO INFORMATION WAS FOUND ABOUT CREDENTIALS");
					}*/
					break;
				
				case WIFI_APP_MSG_START_HTTP_SERVER:
					ESP_LOGI(TAG, "WIFI_APP_MSG_START_HTTP_SERVER");

					http_server_start();
					rgb_led_http_server_started();

					break;

				case WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER:
					ESP_LOGI(TAG, "WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER");

					// Attempt a connection
					wifi_app_connect_sta();

					// Set current number of retries to zero
					g_retry_number = 0;

					// Let the HTTP server know about the connection attempt
					http_server_monitor_send_message(HTTP_MSG_WIFI_CONNECT_INIT);

					break;

				case WIFI_APP_MSG_STA_CONNECTED_GOT_IP:
					ESP_LOGI(TAG, "WIFI_APP_MSG_STA_CONNECTED_GOT_IP");

					rgb_led_wifi_connected();
					http_server_monitor_send_message(HTTP_MSG_WIFI_CONNECT_SUCCESS);

					break;

				case WIFI_APP_MSG_STA_DISCONNECTED:
					ESP_LOGI(TAG, "WIFI_APP_MSG_STA_DISCONNECTED");

					http_server_monitor_send_message(HTTP_MSG_WIFI_CONNECT_FAIL);

					break;

				default:
					break;

			}
		}
	}
}

BaseType_t wifi_app_send_message(wifi_app_message_e msgID)
{
	wifi_app_queue_message_t msg;
	msg.msgID = msgID;
	return xQueueSend(wifi_app_queue_handle, &msg, portMAX_DELAY);
}

wifi_config_t* wifi_app_get_wifi_config(void)
{
	return wifi_config;
}

bool compare_hour_day_structs (struct tm timeinfo, register_saved_e aux_reg ){
	static const char TAG2 [] = "comparing_app";

// chekcing ll the day
	if(timeinfo.tm_wday == 0){
		if( aux_reg.sunday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if(timeinfo.tm_wday == 1){
		if( aux_reg.monday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if(timeinfo.tm_wday == 2){
		if( aux_reg.tuesday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if(timeinfo.tm_wday == 3){
		if( aux_reg.wednesday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if(timeinfo.tm_wday == 4){
		if( aux_reg.thursday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if(timeinfo.tm_wday == 5){
		if( aux_reg.friday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if(timeinfo.tm_wday == 6){
		if( aux_reg.saturday != 1 ){	
			ESP_LOGI(TAG2, "WRONG DAY");
			return false;		
		}
	}

	if( timeinfo.tm_hour == aux_reg.hour ){
		if( timeinfo.tm_min == aux_reg.min ){
			// we should activate the motor
			toogle_led();
			vTaskDelay(40000 / portTICK_PERIOD_MS);
			return true;
		}
		else{
			ESP_LOGI(TAG2, "CORRECT DAY CORRECT HOUR WRONG MINUTE");
			return false;
		}
	}
	else{
		ESP_LOGI(TAG2, "CORRECT DAY WRONG HOUR");
		return false;
		
	}

}

void task_compare_hour_to_execute_action( void *pvParameters ) {
	time_t now;
    struct tm timeinfo;
	while(get_state_time_was_synchronized() == false){
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
       // Asegurar que localtime_r inicialice adecuadamente timeinfo
    

	while (1){

		ESP_LOGI(TAG, "COMPARING HOURS");


		if (time(&now) != -1 && localtime_r(&now, &timeinfo) != NULL)
   		{
   		    // Imprimir la hora actual
   		   // ESP_LOGI(TAG, "Día de la semana: %d, Hora: %02d:%02d:%02d", timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
			ESP_LOGI(TAG, "Día de la semana: %d, Hora: %d:%d:%d", timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
			
   		}
   		else
		{
		    ESP_LOGE(TAG, "Error al obtener la hora actual.");
		}

		for(int i = 0; i< NUM_REGISTERS_AV; i++){
			ESP_LOGI(TAG, "Revisando registro: %d", i);
			compare_hour_day_structs (timeinfo,  register_readings_from_flash[i] );

		}

		vTaskDelay(30000 / portTICK_PERIOD_MS);
	}


}





void wifi_app_start(void)
{
	ESP_LOGI(TAG, "STARTING WIFI APPLICATION");

	// Start WiFi started LED
	rgb_led_wifi_app_started();

	// Disable default WiFi logging messages
	esp_log_level_set("wifi", ESP_LOG_NONE);

	// Allocate memory for the wifi configuration
	wifi_config = (wifi_config_t*)malloc(sizeof(wifi_config_t));
	memset(wifi_config, 0x00, sizeof(wifi_config_t));

	// Create message queue
	wifi_app_queue_handle = xQueueCreate(3, sizeof(wifi_app_queue_message_t));
	// create semaphore for the wifi connection
	mySemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(mySemaphore);
	// Start the WiFi application task
	xTaskCreatePinnedToCore(&wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL, WIFI_APP_TASK_PRIORITY, NULL, WIFI_APP_TASK_CORE_ID);
	// Nota: el control programado del ventilador ahora lo gestiona fan_control.c.
	// Se desactiva la tarea legacy de comparación de registros antiguos para evitar conflictos.
	// xTaskCreatePinnedToCore(&task_compare_hour_to_execute_action, "checking_app_task", 4096, NULL, 5, NULL, 1);
	
	
}










