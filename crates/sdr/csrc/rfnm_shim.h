// Copyright 2025-2026 CEMAXECUTER LLC
// Thin C shim for librfnm C++ API.

#ifndef RFNM_SHIM_H
#define RFNM_SHIM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* rfnm_shim_dev;

typedef struct {
    char serial[32];
    char board_name[64];
    uint64_t dcs_clk;
    uint8_t rx_ch_cnt;
    int8_t gain_min;
    int8_t gain_max;
    int path_preferred;
} rfnm_shim_info;

int rfnm_shim_find(rfnm_shim_info* info, int max_devices);

rfnm_shim_dev rfnm_shim_open(const char* serial);
void rfnm_shim_close(rfnm_shim_dev dev);

int rfnm_shim_get_info(rfnm_shim_dev dev, rfnm_shim_info* info);
int rfnm_shim_set_stream_format_cs16(rfnm_shim_dev dev, size_t* bufsize_out);

int rfnm_shim_configure_rx(rfnm_shim_dev dev, uint32_t channel,
                            int64_t freq, int8_t gain,
                            int16_t samp_div_m, int16_t samp_div_n,
                            int16_t lpf_bw, int path);
int rfnm_shim_apply(rfnm_shim_dev dev, uint16_t applies);

// Low-level streaming: uses device rx_work_start/stop + rx_qbuf/rx_dqbuf
// directly, bypassing rx_stream's reorder/zero-padding logic.
int rfnm_shim_rx_start(rfnm_shim_dev dev, uint32_t channel, size_t bufsize);
int rfnm_shim_rx_stop(rfnm_shim_dev dev, uint32_t channel);
int rfnm_shim_rx_read(rfnm_shim_dev dev, uint8_t ch_mask,
                       int16_t* buf, size_t max_elems,
                       size_t* elems_read, uint32_t timeout_us);

int rfnm_shim_set_rx_gain(rfnm_shim_dev dev, uint32_t channel, int8_t gain);

#ifdef __cplusplus
}
#endif

#endif
