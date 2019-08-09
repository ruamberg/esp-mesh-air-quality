#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "mesh_light.h"
#include "nvs_flash.h"
#include "esp_http_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define MAX_HTTP_RECV_BUFFER 512
static const char *TAG = "HTTP_CLIENT";

/*******************************************************
 *                Macros
 *******************************************************/

/*******************************************************
 *                Constants
 *******************************************************/
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)
#define CONFIG_NODE_ID 1

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static const char *MESH_TAG = "mesh_main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;

mesh_light_ctl_t light_on = {
    .cmd = MESH_CONTROL_CMD,
    .on = 1,
    .token_id = MESH_TOKEN_ID,
    .token_value = MESH_TOKEN_VALUE,
};

mesh_light_ctl_t light_off = {
    .cmd = MESH_CONTROL_CMD,
    .on = 0,
    .token_id = MESH_TOKEN_ID,
    .token_value = MESH_TOKEN_VALUE,
};

enum dht11_status {
    DHT11_CRC_ERROR = -2,
    DHT11_TIMEOUT_ERROR,
    DHT11_OK
};

struct dht11_reading {
    int status;
    int temperature;
    int humidity;
};

/*******************************************************
 *                Function Declarations
 *******************************************************/

/*******************************************************
 *                Function Definitions
 *******************************************************/

 static gpio_num_t dht_gpio;
 static int64_t last_read_time = -2000000;
 static struct dht11_reading last_read;

 static int _waitOrTimeout(uint16_t microSeconds, int level) {
     int micros_ticks = 0;
     while(gpio_get_level(dht_gpio) == level) {
         if(micros_ticks++ > microSeconds)
             return DHT11_TIMEOUT_ERROR;
         ets_delay_us(1);
     }
     return micros_ticks;
 }

 static int _checkCRC(uint8_t data[]) {
     if(data[4] == (data[0] + data[1] + data[2] + data[3]))
         return DHT11_OK;
     else
         return DHT11_CRC_ERROR;
 }

 static void _sendStartSignal() {
     gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
     gpio_set_level(dht_gpio, 0);
     ets_delay_us(20 * 1000);
     gpio_set_level(dht_gpio, 1);
     ets_delay_us(40);
     gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
 }

 static int _checkResponse() {
     /* Wait for next step ~80us*/
     if(_waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
         return DHT11_TIMEOUT_ERROR;

     /* Wait for next step ~80us*/
     if(_waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR)
         return DHT11_TIMEOUT_ERROR;

     return DHT11_OK;
 }

 static struct dht11_reading _timeoutError() {
     struct dht11_reading timeoutError = {DHT11_TIMEOUT_ERROR, -1, -1};
     return timeoutError;
 }

 static struct dht11_reading _crcError() {
     struct dht11_reading crcError = {DHT11_CRC_ERROR, -1, -1};
     return crcError;
 }

 void DHT11_init(gpio_num_t gpio_num) {
     /* Wait 1 seconds to make the device pass its initial unstable status */
     vTaskDelay(1000 / portTICK_PERIOD_MS);
     dht_gpio = gpio_num;
 }

 struct dht11_reading DHT11_read() {
     /* Tried to sense too son since last read (dht11 needs ~2 seconds to make a new read) */
     if(esp_timer_get_time() - 2000000 < last_read_time) {
         return last_read;
     }

     last_read_time = esp_timer_get_time();

     uint8_t data[5] = {0,0,0,0,0};

     _sendStartSignal();

     if(_checkResponse() == DHT11_TIMEOUT_ERROR)
         return last_read = _timeoutError();

     /* Read response */
     for(int i = 0; i < 40; i++) {
         /* Initial data */
         if(_waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR)
             return last_read = _timeoutError();

         if(_waitOrTimeout(70, 1) > 28) {
             /* Bit received was a 1 */
             data[i/8] |= (1 << (7-(i%8)));
         }
     }

     if(_checkCRC(data) != DHT11_CRC_ERROR) {
         last_read.status = DHT11_OK;
         last_read.temperature = data[2];
         last_read.humidity = data[0];
         return last_read;
     } else {
         return last_read = _crcError();
     }
 }

 esp_err_t _http_event_handler(esp_http_client_event_t *evt)
 {
     switch(evt->event_id) {
         case HTTP_EVENT_ERROR:
             ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
             break;
         case HTTP_EVENT_ON_CONNECTED:
             ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
             break;
         case HTTP_EVENT_HEADER_SENT:
             ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
             break;
         case HTTP_EVENT_ON_HEADER:
             ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
             break;
         case HTTP_EVENT_ON_DATA:
             ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
             if (!esp_http_client_is_chunked_response(evt->client)) {
                 // Write out data
                 // printf("%.*s", evt->data_len, (char*)evt->data);
             }

             break;
         case HTTP_EVENT_ON_FINISH:
             ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
             break;
         case HTTP_EVENT_DISCONNECTED:
             ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
             break;
     }
     return ESP_OK;
 }

void node_data(char *data) {
    esp_http_client_config_t config = {
         .url = "http://192.168.43.49:3000",
         .event_handler = _http_event_handler,
    };
     esp_http_client_handle_t client = esp_http_client_init(&config);
     esp_err_t err = esp_http_client_perform(client);

     char post_data[300] = "data=";
     strcat(post_data, data);

     esp_http_client_set_url(client, "http://192.168.43.49:3000");
     esp_http_client_set_method(client, HTTP_METHOD_POST);
     esp_http_client_set_post_field(client, post_data, strlen(post_data));
     err = esp_http_client_perform(client);
     if (err == ESP_OK) {
          ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                      esp_http_client_get_status_code(client),
                      esp_http_client_get_content_length(client));
     } else {
          ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
     }

     esp_http_client_cleanup(client);
}


 void esp_mesh_p2p_tx_projeto(void *arg)
 {
     esp_err_t err;
     int send_count = 0;

     int temperature = 0;
     int humidity = 0;

     mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
     int route_table_size = 0;
     mesh_data_t data;
     data.data = tx_buf;
     data.size = sizeof(tx_buf);
     data.proto = MESH_PROTO_BIN;
     data.tos = MESH_TOS_P2P;
     is_running = true;

     while (is_running) {
         esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                    CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
         if (send_count && !(send_count % 10)) {
             ESP_LOGI(MESH_TAG, "size:%d/%d,send_value:%d ,send_count:%d", route_table_size,
                      esp_mesh_get_routing_table_size(), temperature, send_count);
         }
         send_count++;

         if(DHT11_read().status == 0) {
             ESP_LOGI(MESH_TAG, "Temperature is %d, Humidity is %d, Status %d",
             DHT11_read().temperature, DHT11_read().humidity, DHT11_read().status);
             temperature = DHT11_read().temperature;
             humidity = DHT11_read().humidity;
         } else {
             ESP_LOGI(MESH_TAG, "HDT11 ERROR Status %d", DHT11_read().status);
         }

         tx_buf[22] = CONFIG_NODE_ID;
         tx_buf[23] = temperature;
         tx_buf[24] = humidity;
         tx_buf[25] = mesh_layer;

         err = esp_mesh_send(NULL, &data, 0, NULL, 0);

          if (err) {
                          ESP_LOGE(MESH_TAG,
                                   "[ROOT-2-UNICAST:%d][L:%d]parent:"MACSTR", heap:%d[err:0x%x, proto:%d, tos:%d]",
                                   send_count, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                                   esp_get_free_heap_size(),
                                   err, data.proto, data.tos);
                      } else if (!(send_count % 10)) {
                          ESP_LOGW(MESH_TAG,
                                   "[ROOT-2-UNICAST:%d (count %d)][L:%d][rtableSize:%d]parent:"MACSTR", heap:%d[err:0x%x, proto:%d, tos:%d]",
                                   temperature, send_count, mesh_layer,
                                   esp_mesh_get_routing_table_size(),
                                   MAC2STR(mesh_parent_addr.addr),
                                   esp_get_free_heap_size(),
                                   err, data.proto, data.tos);
                      }

//         for (i = 0; i < route_table_size; i++) {
//
//
//         }
         /* if route_table_size is less than 10, add delay to avoid watchdog in this task. */
         if (route_table_size < 10) {
             vTaskDelay(1 * 1000 / portTICK_RATE_MS);
         }
     }
     vTaskDelete(NULL);
 }


void esp_mesh_p2p_rx_projeto(void *arg)
{
    esp_err_t err;
    mesh_addr_t from;
    int node_id = 0;
    int temperature = 0;
    int humidity = 0;
    int mesh_layer_rec = 0;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;
    is_running = true;
//    node_connected(MAC2STR(from.addr));

    while (is_running) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(MESH_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }

        node_id = data.data[22];
        temperature = data.data[23];
        humidity = data.data[24];
        mesh_layer_rec = data.data[25];

        ESP_LOGW(MESH_TAG,
                          "[#RX:id %d Temperature %d Humidity %d][L:%d] parent:"MACSTR", receive from "MACSTR", size:%d, heap:%d, flag:%d[err:0x%x, proto:%d, tos:%d]",
                             node_id, temperature, humidity, mesh_layer_rec,
                             MAC2STR(mesh_parent_addr.addr), MAC2STR(from.addr),
                             data.size, esp_get_free_heap_size(), flag, err, data.proto,
                             data.tos);

        if (esp_mesh_is_root()) {
            char *date;
            asprintf(&date, "{\"id\":%d, \"temperature\":%d, \"humidity\":%d, \"layer\": %d, \"parent\":\""MACSTR"\", \"address\":\""MACSTR"\", \"size\":%d, \"heap\":%d, \"flag\":%d, \"err\":\"0x%x\", \"proto\":%d, \"tos\":%d}",
                                                         node_id, temperature, humidity, mesh_layer_rec,
                                                         MAC2STR(mesh_parent_addr.addr), MAC2STR(from.addr),
                                                         data.size, esp_get_free_heap_size(), flag, err, data.proto,
                                                         data.tos);
            node_data(date);
        }

    }
    vTaskDelete(NULL);
}


esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_projeto, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_projeto, "MPRX", 3072, NULL, 5, NULL);
    }
    return ESP_OK;
}

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint8_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR"",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr));
        last_layer = mesh_layer;
        mesh_connected_indicator(mesh_layer);
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_disconnected_indicator();
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        mesh_connected_indicator(mesh_layer);
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%d", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:%s", ip4addr_ntoa(&event->ip_info.ip));
}


void app_main(void)
{
    ESP_ERROR_CHECK(mesh_light_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    tcpip_adapter_init();
    /* for mesh
     * stop DHCP server on softAP interface by default
     * stop DHCP client on station interface by default
     * */
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = CONFIG_MESH_CHANNEL;
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%d, %s\n",  esp_get_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");

    DHT11_init(GPIO_NUM_4);
}
