#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "mesh.h"
#include "root_cli.h"

#define TAG "APP"

void app_main(void)
{
    ESP_LOGI(TAG, "System start");

    mesh_init();

    uint16_t node_id = mesh_get_node_id();
    uint16_t root_id = mesh_get_root_id();

    ESP_LOGI(TAG, "Node ID = %u", node_id);

    if (node_id == root_id) {

        ESP_LOGI(TAG, "Node %u → Root", node_id);

        // Start CLI from root_cli component
        root_cli_start();
    }
    else {
        ESP_LOGI(TAG, "Node %u → Relay", node_id);
        /* Set test payload for TDMA slot TX (real mesh will use minimal sensor data). */
        char test_msg[32];
        int len = snprintf(test_msg, sizeof(test_msg), "hello from node %u", (unsigned)node_id);
        if (len < 0) {
            len = 0;
        }
        mesh_set_app_payload((const uint8_t *)test_msg, (uint16_t)len);
    }
}