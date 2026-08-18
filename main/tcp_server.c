#include "sdkconfig.h"

#if CONFIG_GW_WIFI_ENABLED && CONFIG_GW_TCP_SERVER_ENABLED

#include <string.h>
#include <errno.h>

#include "tcp_server.h"
#include "uart_gateway.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define TCP_CLIENT_RX_BUFFER_SIZE 256
#define TCP_OUT_QUEUE_DEPTH 64

typedef struct
{
    int sock;
    bool active;
    int magic_pos;
    bool header_received;
    size_t header_pos;
    size_t payload_pos;
    size_t payload_needed;
    uint8_t header_buffer[UART_PACKET_HEADER_SIZE];
    uint8_t *payload_buffer;
    bool extended_mode;
} tcp_client_ctx_t;

static TaskHandle_t tcp_task_handle = NULL;
static QueueHandle_t tcp_out_queue = NULL;
static int listen_sock = -1;
static volatile bool tcp_running = false;

static tcp_client_ctx_t clients[CONFIG_GW_TCP_MAX_CLIENTS];

static void close_client(tcp_client_ctx_t *client)
{
    if (!client->active)
    {
        return;
    }

    if (client->sock >= 0)
    {
        shutdown(client->sock, 0);
        close(client->sock);
    }

    if (client->payload_buffer)
    {
        free(client->payload_buffer);
        client->payload_buffer = NULL;
    }

    memset(client, 0, sizeof(*client));
    client->sock = -1;
}

static int send_all(int sock, const uint8_t *data, size_t length)
{
    size_t sent_total = 0;

    while (sent_total < length)
    {
        int sent = send(sock, data + sent_total, length - sent_total, 0);
        if (sent <= 0)
        {
            return -1;
        }
        sent_total += (size_t)sent;
    }

    return 0;
}

static void dispatch_full_packet(tcp_client_ctx_t *client)
{
    uart_packet_header_t *header = (uart_packet_header_t *)client->header_buffer;
    uart_gateway_handle_extended_packet(header->type, client->payload_buffer, client->payload_needed);

    if (client->payload_buffer)
    {
        free(client->payload_buffer);
        client->payload_buffer = NULL;
    }

    client->header_received = false;
    client->header_pos = 0;
    client->payload_pos = 0;
    client->payload_needed = 0;
}

static void process_extended_byte(tcp_client_ctx_t *client, uint8_t byte)
{
    if (!client->header_received)
    {
        if (client->header_pos < UART_PACKET_HEADER_SIZE)
        {
            client->header_buffer[client->header_pos++] = byte;
        }

        if (client->header_pos == UART_PACKET_HEADER_SIZE)
        {
            uart_packet_header_t *header = (uart_packet_header_t *)client->header_buffer;

            if (header->length < UART_PACKET_HEADER_SIZE)
            {
                client->header_pos = 0;
                return;
            }

            client->payload_needed = header->length - UART_PACKET_HEADER_SIZE;
            client->payload_pos = 0;
            client->header_received = true;

            if (client->payload_needed == 0)
            {
                dispatch_full_packet(client);
            }
            else
            {
                client->payload_buffer = (uint8_t *)malloc(client->payload_needed);
                if (!client->payload_buffer)
                {
                    client->header_received = false;
                    client->header_pos = 0;
                    client->payload_needed = 0;
                }
            }
        }

        return;
    }

    if (client->payload_pos < client->payload_needed)
    {
        client->payload_buffer[client->payload_pos++] = byte;
    }

    if (client->payload_pos == client->payload_needed)
    {
        dispatch_full_packet(client);
    }
}

static void process_client_bytes(tcp_client_ctx_t *client, const uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        uint8_t byte = buffer[i];

        if (!client->extended_mode)
        {
            if (byte == uart_extmode_magic[client->magic_pos])
            {
                client->magic_pos++;
                if (client->magic_pos == UART_EXTMODE_MAGIC_SIZE)
                {
                    client->extended_mode = true;
                    client->magic_pos = 0;
                }
                continue;
            }

            if (client->magic_pos > 0)
            {
                uart_gateway_queue_cdc_data(uart_extmode_magic, client->magic_pos);
                client->magic_pos = 0;
                if (byte == uart_extmode_magic[0])
                {
                    client->magic_pos = 1;
                    continue;
                }
            }

            uart_gateway_queue_cdc_data(&byte, 1);
            continue;
        }

        process_extended_byte(client, byte);
    }
}

static void broadcast_packet(uart_packet_header_t *packet)
{
    if (!packet)
    {
        return;
    }

    uint16_t packet_type = packet->type;
    uint16_t packet_len = packet->length;

    if (packet_len < UART_PACKET_HEADER_SIZE)
    {
        free(packet);
        return;
    }

    const uint8_t *payload = (const uint8_t *)PTR_BEHIND(packet);
    size_t payload_len = packet_len - UART_PACKET_HEADER_SIZE;

    for (size_t i = 0; i < CONFIG_GW_TCP_MAX_CLIENTS; i++)
    {
        tcp_client_ctx_t *client = &clients[i];
        if (!client->active)
        {
            continue;
        }

        int send_rc = 0;

        if (client->extended_mode)
        {
            send_rc = send_all(client->sock, (const uint8_t *)packet, packet_len);
        }
        else
        {
            if (packet_type == UART_PACKET_TYPE_DATA)
            {
                send_rc = send_all(client->sock, payload, payload_len);
            }
            else
            {
                continue;
            }
        }

        if (send_rc != 0)
        {
            close_client(client);
        }
    }

    free(packet);
}

static void tcp_server_task(void *pvParameters)
{
    (void)pvParameters;

    while (tcp_running)
    {
        fd_set readset;
        FD_ZERO(&readset);

        int max_fd = -1;

        if (listen_sock >= 0)
        {
            FD_SET(listen_sock, &readset);
            max_fd = listen_sock;
        }

        for (size_t i = 0; i < CONFIG_GW_TCP_MAX_CLIENTS; i++)
        {
            if (clients[i].active && clients[i].sock >= 0)
            {
                FD_SET(clients[i].sock, &readset);
                if (clients[i].sock > max_fd)
                {
                    max_fd = clients[i].sock;
                }
            }
        }

        struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = 20000,
        };

        int activity = 0;
        if (max_fd >= 0)
        {
            activity = select(max_fd + 1, &readset, NULL, NULL, &timeout);
        }

        if (activity > 0)
        {
            if (listen_sock >= 0 && FD_ISSET(listen_sock, &readset))
            {
                struct sockaddr_in source_addr;
                socklen_t addr_len = sizeof(source_addr);
                int new_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

                if (new_sock >= 0)
                {
                    bool attached = false;
                    for (size_t i = 0; i < CONFIG_GW_TCP_MAX_CLIENTS; i++)
                    {
                        if (!clients[i].active)
                        {
                            clients[i].sock = new_sock;
                            clients[i].active = true;
                            clients[i].extended_mode = false;
                            attached = true;
                            break;
                        }
                    }

                    if (!attached)
                    {
                        close(new_sock);
                    }
                }
            }

            for (size_t i = 0; i < CONFIG_GW_TCP_MAX_CLIENTS; i++)
            {
                tcp_client_ctx_t *client = &clients[i];
                if (!client->active || client->sock < 0)
                {
                    continue;
                }

                if (!FD_ISSET(client->sock, &readset))
                {
                    continue;
                }

                uint8_t rx_buffer[TCP_CLIENT_RX_BUFFER_SIZE];
                int len = recv(client->sock, rx_buffer, sizeof(rx_buffer), 0);
                if (len <= 0)
                {
                    close_client(client);
                    continue;
                }

                process_client_bytes(client, rx_buffer, (size_t)len);
            }
        }

        while (1)
        {
            uart_packet_header_t *packet = NULL;
            if (xQueueReceive(tcp_out_queue, &packet, 0) != pdTRUE)
            {
                break;
            }
            broadcast_packet(packet);
        }
    }

    for (size_t i = 0; i < CONFIG_GW_TCP_MAX_CLIENTS; i++)
    {
        close_client(&clients[i]);
    }

    if (listen_sock >= 0)
    {
        close(listen_sock);
        listen_sock = -1;
    }

    tcp_task_handle = NULL;
    vTaskDelete(NULL);
}

void tcp_server_fanout_packet(const uart_packet_header_t *packet)
{
    if (!tcp_running || !tcp_out_queue || !packet)
    {
        return;
    }

    uart_packet_header_t *copy = (uart_packet_header_t *)malloc(packet->length);
    if (!copy)
    {
        return;
    }

    memcpy(copy, packet, packet->length);

    if (xQueueSend(tcp_out_queue, &copy, 0) != pdTRUE)
    {
        free(copy);
    }
}

esp_err_t tcp_server_start(void)
{
    if (tcp_running)
    {
        return ESP_OK;
    }

    memset(clients, 0, sizeof(clients));
    for (size_t i = 0; i < CONFIG_GW_TCP_MAX_CLIENTS; i++)
    {
        clients[i].sock = -1;
    }

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0)
    {
        send_message("TCP socket failed: errno=%d", errno);
        return ESP_FAIL;
    }

    int yes = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_GW_TCP_SERVER_PORT),
        .sin_addr = {
            .s_addr = htonl(INADDR_ANY),
        },
    };

    if (bind(listen_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
    {
        send_message("TCP bind failed: errno=%d", errno);
        close(listen_sock);
        listen_sock = -1;
        return ESP_FAIL;
    }

    if (listen(listen_sock, CONFIG_GW_TCP_MAX_CLIENTS) < 0)
    {
        send_message("TCP listen failed: errno=%d", errno);
        close(listen_sock);
        listen_sock = -1;
        return ESP_FAIL;
    }

    tcp_out_queue = xQueueCreate(TCP_OUT_QUEUE_DEPTH, sizeof(uart_packet_header_t *));
    if (!tcp_out_queue)
    {
        close(listen_sock);
        listen_sock = -1;
        return ESP_ERR_NO_MEM;
    }

    tcp_running = true;

    if (xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 6144, NULL, 5, &tcp_task_handle, 0) != pdPASS)
    {
        tcp_running = false;
        vQueueDelete(tcp_out_queue);
        tcp_out_queue = NULL;
        close(listen_sock);
        listen_sock = -1;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void tcp_server_stop(void)
{
    if (!tcp_running)
    {
        return;
    }

    tcp_running = false;

    if (listen_sock >= 0)
    {
        shutdown(listen_sock, 0);
    }

    for (int i = 0; i < 50; i++)
    {
        if (tcp_task_handle == NULL)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (tcp_out_queue)
    {
        uart_packet_header_t *packet = NULL;
        while (xQueueReceive(tcp_out_queue, &packet, 0) == pdTRUE)
        {
            free(packet);
        }
        vQueueDelete(tcp_out_queue);
        tcp_out_queue = NULL;
    }
}

#else

#include "tcp_server.h"

esp_err_t tcp_server_start(void)
{
    return ESP_OK;
}

void tcp_server_stop(void)
{
}

void tcp_server_fanout_packet(const uart_packet_header_t *packet)
{
    (void)packet;
}

#endif
