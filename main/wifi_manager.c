#include "sdkconfig.h"

#if CONFIG_GW_WIFI_ENABLED

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "wifi_manager.h"
#include "config_manager.h"
#include "tcp_server.h"
#include "uart_gateway.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "mdns.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#ifdef CONFIG_LOGGER_ENABLED
#include "logger.h"
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static EventGroupHandle_t wifi_event_group = NULL;
static int retry_num = 0;
static httpd_handle_t http_server = NULL;
static TaskHandle_t dns_task_handle = NULL;
static volatile bool dns_running = false;
static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif = NULL;
static bool mdns_started = false;

extern const uint8_t web_config_html_start[] asm("_binary_config_html_start");
extern const uint8_t web_config_html_end[] asm("_binary_config_html_end");
extern const uint8_t web_files_html_start[] asm("_binary_files_html_start");
extern const uint8_t web_files_html_end[] asm("_binary_files_html_end");

static const char *find_token(const char *buffer, size_t len, const char *token)
{
    size_t token_len = strlen(token);
    if (token_len == 0 || len < token_len)
    {
        return NULL;
    }

    for (size_t i = 0; i <= (len - token_len); i++)
    {
        if (memcmp(buffer + i, token, token_len) == 0)
        {
            return buffer + i;
        }
    }

    return NULL;
}

static esp_err_t send_template_segment(httpd_req_t *req, const char *start, const char *end)
{
    if (start == NULL || end == NULL || end < start)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = (size_t)(end - start);
    if (len == 0)
    {
        return ESP_OK;
    }

    return httpd_resp_send_chunk(req, start, len);
}

static void sanitize_hostname(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    if (dst_len == 0)
    {
        return;
    }

    if (src == NULL)
    {
        src = "";
    }

    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; si++)
    {
        char c = src[si];
        if ((c >= 'A') && (c <= 'Z'))
        {
            dst[di++] = (char)(c + ('a' - 'A'));
        }
        else if (((c >= 'a') && (c <= 'z')) || ((c >= '0') && (c <= '9')) || c == '-')
        {
            dst[di++] = c;
        }
        else if (c == ' ' || c == '_' || c == '.')
        {
            dst[di++] = '-';
        }
    }

    while (di > 0 && dst[di - 1] == '-')
    {
        di--;
    }

    if (di == 0)
    {
        const char fallback[] = "esp32c3-gw";
        size_t fi = 0;
        while (fallback[fi] != '\0' && fi + 1 < dst_len)
        {
            dst[fi] = fallback[fi];
            fi++;
        }
        dst[fi] = '\0';
        return;
    }

    dst[di] = '\0';
}

static void apply_hostname_config(const char *host)
{
    if (sta_netif)
    {
        esp_netif_set_hostname(sta_netif, host);
    }
    if (ap_netif)
    {
        esp_netif_set_hostname(ap_netif, host);
    }

    if (!mdns_started)
    {
        if (mdns_init() == ESP_OK)
        {
            mdns_started = true;
        }
    }

    if (mdns_started)
    {
        mdns_hostname_set(host);
        mdns_instance_name_set(host);
    }
}

static void url_decode(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; si++)
    {
        if (src[si] == '+')
        {
            dst[di++] = ' ';
            continue;
        }

        if (src[si] == '%' && src[si + 1] && src[si + 2])
        {
            char hex[3] = { src[si + 1], src[si + 2], '\0' };
            dst[di++] = (char)strtol(hex, NULL, 16);
            si += 2;
            continue;
        }

        dst[di++] = src[si];
    }
    dst[di] = '\0';
}

static esp_err_t html_root_handler(httpd_req_t *req)
{
    static const char files_link_token[] = "{{FILES_LINK}}";
    static const char form_fields_token[] = "{{FORM_FIELDS}}";

    const config_field_desc_t *fields;
    size_t field_count = 0;
    fields = config_manager_get_fields(&field_count);

    const device_config_t *cfg = config_manager_get();

    const char *tmpl_start = (const char *)web_config_html_start;
    const char *tmpl_end = (const char *)web_config_html_end;
    size_t tmpl_len = (size_t)(tmpl_end - tmpl_start);

    const char *files_pos = find_token(tmpl_start, tmpl_len, files_link_token);
    const char *form_pos = find_token(tmpl_start, tmpl_len, form_fields_token);
    if (!files_pos || !form_pos || form_pos <= files_pos)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid config template");
    }

    httpd_resp_set_type(req, "text/html");

    if (send_template_segment(req, tmpl_start, files_pos) != ESP_OK)
    {
        return ESP_FAIL;
    }

#ifdef CONFIG_LOGGER_ENABLED
    if (httpd_resp_sendstr_chunk(req, "<a class='btn' href='/files'>&#128196; Log Files</a>") != ESP_OK)
    {
        return ESP_FAIL;
    }
#endif

    const char *after_files = files_pos + strlen(files_link_token);
    if (send_template_segment(req, after_files, form_pos) != ESP_OK)
    {
        return ESP_FAIL;
    }

    const char *current_group = NULL;
    char line[256];

    for (size_t i = 0; i < field_count; i++)
    {
        const config_field_desc_t *f = &fields[i];

        if (!current_group || strcmp(current_group, f->group) != 0)
        {
            if (current_group)
            {
                httpd_resp_sendstr_chunk(req, "</fieldset>");
            }
            snprintf(line, sizeof(line), "<fieldset><legend>%s</legend>", f->group);
            httpd_resp_sendstr_chunk(req, line);
            current_group = f->group;
        }

        char name[96];
        snprintf(name, sizeof(name), "%s__%s", f->nvs_namespace, f->nvs_key);

        if (f->type == CONFIG_FIELD_U8)
        {
            uint8_t value = *(const uint8_t *)(((const uint8_t *)cfg) + f->offset);
            snprintf(line, sizeof(line),
                     "<label>%s</label><input type='number' name='%s' min='%lu' max='%lu' value='%u'>",
                     f->label, name, (unsigned long)f->min_value, (unsigned long)f->max_value, value);
            httpd_resp_sendstr_chunk(req, line);
        }
        else if (f->type == CONFIG_FIELD_U32)
        {
            uint32_t value = *(const uint32_t *)(((const uint8_t *)cfg) + f->offset);
            snprintf(line, sizeof(line),
                     "<label>%s</label><input type='number' name='%s' min='%lu' max='%lu' value='%lu'>",
                     f->label, name, (unsigned long)f->min_value, (unsigned long)f->max_value, (unsigned long)value);
            httpd_resp_sendstr_chunk(req, line);
        }
        else
        {
            const char *value = (const char *)(((const uint8_t *)cfg) + f->offset);
            const char *type = (strstr(f->nvs_key, "password") != NULL) ? "password" : "text";
            snprintf(line, sizeof(line),
                     "<label>%s</label><input type='%s' name='%s' maxlength='%u' value='%s'>",
                     f->label, type, name, (unsigned)(f->str_max_len - 1), value);
            httpd_resp_sendstr_chunk(req, line);
        }
    }

    if (current_group)
    {
        httpd_resp_sendstr_chunk(req, "</fieldset>");
    }

    const char *after_form = form_pos + strlen(form_fields_token);
    if (send_template_segment(req, after_form, tmpl_end) != ESP_OK)
    {
        return ESP_FAIL;
    }

    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

static void apply_form_value(device_config_t *cfg, const config_field_desc_t *field, const char *value)
{
    uint8_t *base = (uint8_t *)cfg;

    if (field->type == CONFIG_FIELD_U8)
    {
        uint32_t parsed = (uint32_t)strtoul(value, NULL, 10);
        if (parsed >= field->min_value && parsed <= field->max_value)
        {
            *(uint8_t *)(base + field->offset) = (uint8_t)parsed;
        }
        return;
    }

    if (field->type == CONFIG_FIELD_U32)
    {
        uint32_t parsed = (uint32_t)strtoul(value, NULL, 10);
        if (parsed >= field->min_value && parsed <= field->max_value)
        {
            *(uint32_t *)(base + field->offset) = parsed;
        }
        return;
    }

    char *dst = (char *)(base + field->offset);
    strncpy(dst, value, field->str_max_len - 1);
    dst[field->str_max_len - 1] = '\0';
}

static esp_err_t html_save_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 4096)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
    }

    char *body = (char *)malloc(req->content_len + 1);
    if (!body)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
    }

    int received = 0;
    while (received < req->content_len)
    {
        int r = httpd_req_recv(req, body + received, req->content_len - received);
        if (r <= 0)
        {
            free(body);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Read failed");
        }
        received += r;
    }
    body[received] = '\0';

    device_config_t *cfg = config_manager_get_mutable();

    const config_field_desc_t *fields;
    size_t field_count = 0;
    fields = config_manager_get_fields(&field_count);

    char *saveptr = NULL;
    char *pair = strtok_r(body, "&", &saveptr);

    while (pair)
    {
        char *eq = strchr(pair, '=');
        if (eq)
        {
            *eq = '\0';
            const char *name = pair;
            const char *value_enc = eq + 1;

            char value[256];
            url_decode(value, sizeof(value), value_enc);

            for (size_t i = 0; i < field_count; i++)
            {
                char expected[96];
                snprintf(expected, sizeof(expected), "%s__%s", fields[i].nvs_namespace, fields[i].nvs_key);
                if (strcmp(name, expected) == 0)
                {
                    apply_form_value(cfg, &fields[i], value);
                    break;
                }
            }
        }

        pair = strtok_r(NULL, "&", &saveptr);
    }

    free(body);

    config_manager_save_all();

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<html><body><h3>Saved. Rebooting...</h3></body></html>");

    vTaskDelay(pdMS_TO_TICKS(400));
    esp_restart();

    return ESP_OK;
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

static void dns_task(void *pvParameters)
{
    (void)pvParameters;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr = { .s_addr = htonl(INADDR_ANY) },
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
    {
        close(sock);
        dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    uint8_t rx[256];
    uint8_t tx[512];

    while (dns_running)
    {
        struct sockaddr_in source_addr;
        socklen_t source_len = sizeof(source_addr);
        int len = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&source_addr, &source_len);
        if (len < 12)
        {
            continue;
        }

        memcpy(tx, rx, len);

        tx[2] = 0x81;
        tx[3] = 0x80;
        tx[6] = 0x00;
        tx[7] = 0x01;

        int pos = len;
        const uint8_t answer[] = {
            0xC0, 0x0C,
            0x00, 0x01,
            0x00, 0x01,
            0x00, 0x00, 0x00, 0x1E,
            0x00, 0x04,
            192, 168, 4, 1
        };

        if ((pos + (int)sizeof(answer)) <= (int)sizeof(tx))
        {
            memcpy(tx + pos, answer, sizeof(answer));
            pos += (int)sizeof(answer);
            sendto(sock, tx, pos, 0, (struct sockaddr *)&source_addr, source_len);
        }
    }

    close(sock);
    dns_task_handle = NULL;
    vTaskDelete(NULL);
}

#ifdef CONFIG_LOGGER_ENABLED

static esp_err_t send_escaped_text_chunk(httpd_req_t *req, const uint8_t *data, size_t len)
{
    char out[512];
    size_t oi = 0;

    for (size_t i = 0; i < len; i++)
    {
        uint8_t c = data[i];
        const char *replace = NULL;

        if (c == '&')
        {
            replace = "&amp;";
        }
        else if (c == '<')
        {
            replace = "&lt;";
        }
        else if (c == '>')
        {
            replace = "&gt;";
        }
        else if (c == '\r' || c == '\n' || c == '\t' || isprint((int)c))
        {
            if (oi + 1 >= sizeof(out))
            {
                if (httpd_resp_send_chunk(req, out, oi) != ESP_OK)
                {
                    return ESP_FAIL;
                }
                oi = 0;
            }
            out[oi++] = (char)c;
            continue;
        }
        else
        {
            if (oi + 1 >= sizeof(out))
            {
                if (httpd_resp_send_chunk(req, out, oi) != ESP_OK)
                {
                    return ESP_FAIL;
                }
                oi = 0;
            }
            out[oi++] = '.';
            continue;
        }

        size_t rlen = strlen(replace);
        if (oi + rlen >= sizeof(out))
        {
            if (httpd_resp_send_chunk(req, out, oi) != ESP_OK)
            {
                return ESP_FAIL;
            }
            oi = 0;
        }
        memcpy(out + oi, replace, rlen);
        oi += rlen;
    }

    if (oi > 0)
    {
        if (httpd_resp_send_chunk(req, out, oi) != ESP_OK)
        {
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

static esp_err_t html_files_handler(httpd_req_t *req)
{
    static const char file_rows_token[] = "{{FILE_ROWS}}";

    const char *tmpl_start = (const char *)web_files_html_start;
    const char *tmpl_end = (const char *)web_files_html_end;
    size_t tmpl_len = (size_t)(tmpl_end - tmpl_start);
    const char *rows_pos = find_token(tmpl_start, tmpl_len, file_rows_token);
    if (!rows_pos)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid files template");
    }

    httpd_resp_set_type(req, "text/html");

    if (send_template_segment(req, tmpl_start, rows_pos) != ESP_OK)
    {
        return ESP_FAIL;
    }

    DIR *dir = opendir(LOGGER_MOUNT_POINT);
    if (dir != NULL)
    {
        struct dirent *entry;
        char line[1400];
        while ((entry = readdir(dir)) != NULL)
        {
            if (entry->d_name[0] == '.')
            {
                continue;
            }
            char fpath[280];
            snprintf(fpath, sizeof(fpath), "%s/%s", LOGGER_MOUNT_POINT, entry->d_name);
            struct stat st;
            long fsize = -1;
            if (stat(fpath, &st) == 0)
            {
                fsize = (long)st.st_size;
            }
            snprintf(line, sizeof(line),
                "<tr><td>%s</td><td>%ld B</td>"
                "<td><a href='/files/preview/%s'>Preview</a>&nbsp;"
                "<a href='/files/download/%s'>Download</a>&nbsp;"
                "<a class='del' href='/files/delete/%s'"
                " onclick=\"return confirm('Delete?')\">Delete</a></td></tr>",
                entry->d_name, fsize, entry->d_name, entry->d_name, entry->d_name);
            httpd_resp_sendstr_chunk(req, line);
        }
        closedir(dir);
    }
    else
    {
        httpd_resp_sendstr_chunk(req, "<tr><td colspan='3'>Filesystem not mounted</td></tr>");
    }

    const char *after_rows = rows_pos + strlen(file_rows_token);
    if (send_template_segment(req, after_rows, tmpl_end) != ESP_OK)
    {
        return ESP_FAIL;
    }

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t file_download_handler(httpd_req_t *req)
{
    const char *prefix = "/files/download/";
    const char *fname = req->uri + strlen(prefix);

    if (strstr(fname, "..") != NULL || strchr(fname, '/') != NULL)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
    }

    char fpath[280];
    snprintf(fpath, sizeof(fpath), "%s/%s", LOGGER_MOUNT_POINT, fname);

    FILE *f = fopen(fpath, "rb");
    if (f == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    }

    httpd_resp_set_type(req, "application/octet-stream");
    char cd_hdr[128];
    snprintf(cd_hdr, sizeof(cd_hdr), "attachment; filename=\"%s\"", fname);
    httpd_resp_set_hdr(req, "Content-Disposition", cd_hdr);

    uint8_t *buf = (uint8_t *)malloc(1024);
    if (buf == NULL)
    {
        fclose(f);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
    }

    size_t n;
    while ((n = fread(buf, 1, 1024, f)) > 0)
    {
        if (httpd_resp_send_chunk(req, (const char *)buf, (ssize_t)n) != ESP_OK)
        {
            break;
        }
    }
    free(buf);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t file_preview_handler(httpd_req_t *req)
{
    const char *prefix = "/files/preview/";
    const char *fname = req->uri + strlen(prefix);

    if (strstr(fname, "..") != NULL || strchr(fname, '/') != NULL)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
    }

    char fpath[280];
    snprintf(fpath, sizeof(fpath), "%s/%s", LOGGER_MOUNT_POINT, fname);

    FILE *f = fopen(fpath, "rb");
    if (f == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Log Preview</title>"
        "<style>body{background:#0c1118;color:#d1d5db;font-family:'JetBrains Mono','Consolas',monospace;padding:16px;}"
        "a{color:#93c5fd;text-decoration:none;}"
        "pre{margin-top:12px;background:#0f1620;border:1px solid #2a3646;border-radius:6px;padding:12px;white-space:pre-wrap;word-break:break-word;}"
        "</style></head><body>");

    char header[512];
    snprintf(header, sizeof(header),
             "<h3>Preview: %s</h3><p><a href='/files'>&larr; Back to files</a></p><pre>",
             fname);
    httpd_resp_sendstr_chunk(req, header);

    uint8_t buf[256];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        if (send_escaped_text_chunk(req, buf, n) != ESP_OK)
        {
            fclose(f);
            return ESP_FAIL;
        }
    }

    fclose(f);

    httpd_resp_sendstr_chunk(req, "</pre></body></html>");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t file_delete_handler(httpd_req_t *req)
{
    const char *prefix = "/files/delete/";
    const char *fname = req->uri + strlen(prefix);

    if (strstr(fname, "..") != NULL || strchr(fname, '/') != NULL)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
    }

    char fpath[280];
    snprintf(fpath, sizeof(fpath), "%s/%s", LOGGER_MOUNT_POINT, fname);

    unlink(fpath);

    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/files");
    return httpd_resp_send(req, NULL, 0);
}

#endif /* CONFIG_LOGGER_ENABLED */

static void start_http_server(bool captive_mode)
{
#if CONFIG_GW_HTTP_SERVER_ENABLED
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = CONFIG_GW_HTTP_SERVER_PORT;
    cfg.max_uri_handlers = 8;
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&http_server, &cfg) != ESP_OK)
    {
        return;
    }

    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = html_root_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t uri_save = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = html_save_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(http_server, &uri_root);
    httpd_register_uri_handler(http_server, &uri_save);

#ifdef CONFIG_LOGGER_ENABLED
    httpd_uri_t uri_files = {
        .uri = "/files",
        .method = HTTP_GET,
        .handler = html_files_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t uri_download = {
        .uri = "/files/download/*",
        .method = HTTP_GET,
        .handler = file_download_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t uri_preview = {
        .uri = "/files/preview/*",
        .method = HTTP_GET,
        .handler = file_preview_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t uri_file_del = {
        .uri = "/files/delete/*",
        .method = HTTP_GET,
        .handler = file_delete_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(http_server, &uri_files);
    httpd_register_uri_handler(http_server, &uri_download);
    httpd_register_uri_handler(http_server, &uri_preview);
    httpd_register_uri_handler(http_server, &uri_file_del);
#endif /* CONFIG_LOGGER_ENABLED */

    if (captive_mode)
    {
        httpd_uri_t uri_all = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = captive_redirect_handler,
            .user_ctx = NULL,
        };
        httpd_register_uri_handler(http_server, &uri_all);
    }
#else
    (void)captive_mode;
#endif /* CONFIG_GW_HTTP_SERVER_ENABLED */
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        retry_num++;
        if (retry_num <= 2)
        {
            esp_wifi_connect();
        }
        else
        {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void start_fallback_ap(void)
{
    const device_config_t *cfg = config_manager_get();

    wifi_config_t ap_config = { 0 };

    strlcpy((char *)ap_config.ap.ssid, cfg->wifi_ap_ssid, sizeof(ap_config.ap.ssid));
    strlcpy((char *)ap_config.ap.password, cfg->wifi_ap_password, sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len = strlen((char *)ap_config.ap.ssid);
    ap_config.ap.channel = cfg->wifi_ap_channel;
    ap_config.ap.max_connection = cfg->wifi_ap_max_connections;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    if (strlen((char *)ap_config.ap.password) >= 8)
    {
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();

    dns_running = true;
    xTaskCreatePinnedToCore(dns_task, "dns_server", 3072, NULL, 3, &dns_task_handle, 0);

    start_http_server(true);
    tcp_server_start();
}

esp_err_t wifi_manager_start(void)
{
    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    sta_netif = esp_netif_create_default_wifi_sta();
    ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &event_handler,
                                        NULL,
                                        &instance_any_id);

    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &event_handler,
                                        NULL,
                                        &instance_got_ip);

    const device_config_t *config = config_manager_get();
    char host[DEVICE_NAME_MAX_LEN];
    sanitize_hostname(host, sizeof(host), config->device_name);
    apply_hostname_config(host);

    if (strlen(config->wifi_sta_ssid) > 0)
    {
        wifi_config_t wifi_config = { 0 };
        strlcpy((char *)wifi_config.sta.ssid, config->wifi_sta_ssid, sizeof(wifi_config.sta.ssid));
        strlcpy((char *)wifi_config.sta.password, config->wifi_sta_password, sizeof(wifi_config.sta.password));

        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_start();

        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdFALSE,
                                               pdFALSE,
                                               pdMS_TO_TICKS(config->wifi_sta_timeout_s * 1000));

        if (bits & WIFI_CONNECTED_BIT)
        {
            start_http_server(false);
            tcp_server_start();
            return ESP_OK;
        }

        send_message("WiFi STA connect failed, switching to AP captive portal");
        esp_wifi_stop();
    }

    start_fallback_ap();
    return ESP_OK;
}

void wifi_manager_stop(void)
{
    tcp_server_stop();

    if (http_server)
    {
        httpd_stop(http_server);
        http_server = NULL;
    }

    dns_running = false;
    if (dns_task_handle)
    {
        for (int i = 0; i < 40; i++)
        {
            if (!dns_task_handle)
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    esp_wifi_stop();

    if (mdns_started)
    {
        mdns_free();
        mdns_started = false;
    }
}

#else

#include "wifi_manager.h"

esp_err_t wifi_manager_start(void)
{
    return ESP_OK;
}

void wifi_manager_stop(void)
{
}

#endif
