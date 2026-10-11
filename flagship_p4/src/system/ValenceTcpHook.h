#pragma once

// ValenceTcpHook -- the lwIP segment output hook, included into every lwIP
// source through ESP_IDF_LWIP_HOOK_FILENAME (src/CMakeLists.txt)
// Constraints:
// - C, not C++: lwIP compiles as C. Declarations only; the body is
//   ValenceTcpTally.cpp.
// - Runs on the tcpip thread for every segment lwIP outputs. Never calls a
//   tcp API, never blocks, never logs, never allocates.
// - Adds no TCP option: the hook returns `opts` unchanged, so
//   LWIP_HOOK_TCP_OUT_TCPOPT_LENGTH stays undefined.
// See: lwip/opt.h LWIP_HOOK_TCP_OUT_ADD_TCPOPTS, system/TcpTally.h

#ifdef __cplusplus
extern "C" {
#endif

struct pbuf;
struct tcp_hdr;
struct tcp_pcb;

u32_t* valence_tcp_out_hook(struct pbuf* p, struct tcp_hdr* hdr, const struct tcp_pcb* pcb, u32_t* opts);

#ifdef __cplusplus
}
#endif

#define LWIP_HOOK_TCP_OUT_ADD_TCPOPTS(p, hdr, pcb, opts) valence_tcp_out_hook(p, hdr, pcb, opts)
