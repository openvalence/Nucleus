// esp_http_server.h -- host stand-in for test_ws_port
// Constraints:
// - Declares only what flagship_p4/src/hub/ValenceWsPort.cpp uses; the
//   definitions are the suite's fakes in test_main.cpp.
#pragma once

#include <cstddef>
#include <cstdint>

using esp_err_t = int;
#define ESP_OK 0
#define ESP_FAIL -1

using httpd_handle_t = void*;
enum http_method { HTTP_GET = 1 };

struct httpd_req_t {
    httpd_handle_t handle;
    int method;
};

enum httpd_ws_type_t {
    HTTPD_WS_TYPE_CONTINUE = 0x0,
    HTTPD_WS_TYPE_TEXT = 0x1,
    HTTPD_WS_TYPE_BINARY = 0x2,
    HTTPD_WS_TYPE_CLOSE = 0x8,
    HTTPD_WS_TYPE_PING = 0x9,
    HTTPD_WS_TYPE_PONG = 0xA,
};

struct httpd_ws_frame_t {
    bool final;
    bool fragmented;
    httpd_ws_type_t type;
    uint8_t* payload;
    size_t len;
};

struct httpd_uri_t {
    const char* uri;
    http_method method;
    esp_err_t (*handler)(httpd_req_t*);
    void* user_ctx;
    bool is_websocket;
    bool handle_ws_control_frames;
    const char* supported_subprotocol;
};

struct httpd_config_t {
    uint16_t server_port;
    uint16_t ctrl_port;
    uint16_t max_open_sockets;
    uint16_t max_uri_handlers;
    bool lru_purge_enable;
    esp_err_t (*open_fn)(httpd_handle_t, int);
    void (*close_fn)(httpd_handle_t, int);
    size_t stack_size;
};
#define HTTPD_DEFAULT_CONFIG() httpd_config_t{}

esp_err_t httpd_ws_send_frame_async(httpd_handle_t hd, int fd, httpd_ws_frame_t* frame);
esp_err_t httpd_ws_send_frame(httpd_req_t* req, httpd_ws_frame_t* frame);
esp_err_t httpd_ws_recv_frame(httpd_req_t* req, httpd_ws_frame_t* frame, size_t max_len);
int httpd_req_to_sockfd(httpd_req_t* req);
esp_err_t httpd_sess_trigger_close(httpd_handle_t hd, int fd);
esp_err_t httpd_start(httpd_handle_t* hd, const httpd_config_t* cfg);
esp_err_t httpd_register_uri_handler(httpd_handle_t hd, const httpd_uri_t* uri);
esp_err_t httpd_get_client_list(httpd_handle_t hd, size_t* fds, int* client_fds);
