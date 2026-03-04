// Copyright 2025-2026 CEMAXECUTER LLC
// Thin C++ shim for librfnm. Wraps rfnm::device behind extern "C" functions
// with opaque pointers for Rust FFI.
//
// Uses the low-level rx_qbuf/rx_dqbuf API directly, bypassing rx_stream's
// reorder queue and zero-padding logic which livelocks at high sample rates.

#include "rfnm_shim.h"
#include <librfnm/device.h>
#include <spdlog/spdlog.h>
#include <cstring>
#include <cstdio>

static void format_serial(const uint8_t serial[9], char* out, size_t out_len) {
    snprintf(out, out_len,
        "%02x%02x%02x%02x%02x%02x%02x%02x%02x",
        serial[0], serial[1], serial[2], serial[3], serial[4],
        serial[5], serial[6], serial[7], serial[8]);
}

static void fill_info(const struct rfnm_dev_hwinfo* hw, rfnm_shim_info* info) {
    memset(info, 0, sizeof(*info));
    format_serial(hw->motherboard.serial_number, info->serial, sizeof(info->serial));
    strncpy(info->board_name, hw->motherboard.user_readable_name, sizeof(info->board_name) - 1);
    info->dcs_clk = hw->clock.dcs_clk;
    info->rx_ch_cnt = 0;
    for (int i = 0; i < 2; i++) {
        info->rx_ch_cnt += hw->daughterboard[i].rx_ch_cnt;
    }
}

int rfnm_shim_find(rfnm_shim_info* info, int max_devices) {
    try {
        auto list = rfnm::device::find(rfnm::TRANSPORT_USB);
        int count = 0;
        for (size_t i = 0; i < list.size() && count < max_devices; i++) {
            fill_info(&list[i], &info[count]);
            count++;
        }
        return count;
    } catch (...) {
        return 0;
    }
}

rfnm_shim_dev rfnm_shim_open(const char* serial) {
    try {
        // Suppress librfnm's spdlog chatter (cc overwritten, stale cc, etc.)
        spdlog::set_level(spdlog::level::err);

        std::string addr = serial ? serial : "";
        auto* dev = new rfnm::device(rfnm::TRANSPORT_USB, addr);
        return static_cast<rfnm_shim_dev>(dev);
    } catch (...) {
        return nullptr;
    }
}

void rfnm_shim_close(rfnm_shim_dev dev) {
    if (dev) {
        delete static_cast<rfnm::device*>(dev);
    }
}

int rfnm_shim_get_info(rfnm_shim_dev dev, rfnm_shim_info* info) {
    if (!dev || !info) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        const auto* hw = d->get_hwinfo();
        if (!hw) return -1;
        fill_info(hw, info);

        // Get gain range and preferred antenna path from first RX channel
        const auto* ch = d->get_rx_channel(0);
        if (ch) {
            info->gain_min = ch->gain_range.min;
            info->gain_max = ch->gain_range.max;
            info->path_preferred = static_cast<int>(ch->path_preferred);
        }
        return 0;
    } catch (...) {
        return -1;
    }
}

int rfnm_shim_set_stream_format_cs16(rfnm_shim_dev dev, size_t* bufsize_out) {
    if (!dev) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        auto rc = d->set_stream_format(rfnm::STREAM_FORMAT_CS16, bufsize_out);
        return static_cast<int>(rc);
    } catch (...) {
        return -1;
    }
}

int rfnm_shim_configure_rx(rfnm_shim_dev dev, uint32_t channel,
                            int64_t freq, int8_t gain,
                            int16_t samp_div_m, int16_t samp_div_n,
                            int16_t lpf_bw, int path) {
    if (!dev) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        rfnm_api_failcode rc;

        // Ensure channel is OFF before configuring.
        d->set_rx_channel_active(channel, RFNM_CH_OFF, RFNM_CH_STREAM_AUTO, false);

        rc = d->set_rx_channel_freq(channel, freq, false);
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        rc = d->set_rx_channel_gain(channel, gain, false);
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        rc = d->set_rx_channel_samp_freq_div(channel, samp_div_m, samp_div_n, false);
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        rc = d->set_rx_channel_rfic_lpf_bw(channel, lpf_bw, false);
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        rc = d->set_rx_channel_path(channel, static_cast<rfnm_rf_path>(path), false);
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        return 0;
    } catch (...) {
        return -1;
    }
}

int rfnm_shim_apply(rfnm_shim_dev dev, uint16_t applies) {
    if (!dev) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        auto rc = d->set(applies);
        return static_cast<int>(rc);
    } catch (...) {
        return -1;
    }
}

// Low-level streaming: activate channel, start USB workers, flush stale data.
// The library internally allocates MIN_RX_BUFCNT (1000) buffers.
int rfnm_shim_rx_start(rfnm_shim_dev dev, uint32_t channel, size_t /*bufsize*/) {
    if (!dev) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);

        // Activate channel and apply
        d->set_rx_channel_active(channel, RFNM_CH_ON, RFNM_CH_STREAM_AUTO, false);
        auto rc = d->set(rfnm::rx_channel_apply_flags[channel]);
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        // Start USB worker threads (allocates 1000 buffers internally)
        rc = d->rx_work_start();
        if (rc != RFNM_API_OK) return static_cast<int>(rc);

        // Flush stale buffers from previous sessions
        d->rx_flush(100000, rfnm::channel_flags[channel]);

        // Discard a few initial buffers to let hardware settle
        for (int i = 0; i < 3; i++) {
            rfnm::rx_buf* buf = nullptr;
            rc = d->rx_dqbuf(&buf, rfnm::channel_flags[channel], 500000);
            if (rc == RFNM_API_OK && buf) {
                d->rx_qbuf(buf);
            }
        }

        return 0;
    } catch (...) {
        return -1;
    }
}

int rfnm_shim_rx_stop(rfnm_shim_dev dev, uint32_t channel) {
    if (!dev) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        d->rx_work_stop();
        d->set_rx_channel_active(channel, RFNM_CH_OFF, RFNM_CH_STREAM_AUTO, false);
        d->set(rfnm::rx_channel_apply_flags[channel]);
        d->rx_flush(0, rfnm::channel_flags[channel]);
        return 0;
    } catch (...) {
        return -1;
    }
}

// Read one buffer via rx_dqbuf, copy CS16 data to output, re-queue buffer.
// No reordering or zero-padding -- just raw sequential buffers.
int rfnm_shim_rx_read(rfnm_shim_dev dev, uint8_t ch_mask,
                       int16_t* buf, size_t max_elems,
                       size_t* elems_read, uint32_t timeout_us) {
    if (!dev || !buf || !elems_read) return -1;
    *elems_read = 0;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        rfnm::rx_buf* rx_buf = nullptr;

        auto rc = d->rx_dqbuf(&rx_buf, ch_mask, timeout_us);
        if (rc != RFNM_API_OK || !rx_buf) {
            return static_cast<int>(rc);
        }

        // Copy CS16 samples from library buffer to output.
        // Each buffer has RFNM_USB_RX_PACKET_ELEM_CNT complex samples.
        // In CS16: 4 bytes per sample, so elem_cnt = data_bytes / 4.
        size_t elem_cnt = RFNM_USB_RX_PACKET_ELEM_CNT;
        if (elem_cnt > max_elems) elem_cnt = max_elems;
        memcpy(buf, rx_buf->buf, elem_cnt * 4);
        *elems_read = elem_cnt;

        // Return buffer to library immediately
        d->rx_qbuf(rx_buf);

        return static_cast<int>(rc);
    } catch (...) {
        return -1;
    }
}

int rfnm_shim_set_rx_gain(rfnm_shim_dev dev, uint32_t channel, int8_t gain) {
    if (!dev) return -1;
    try {
        auto* d = static_cast<rfnm::device*>(dev);
        auto rc = d->set_rx_channel_gain(channel, gain, true);
        return static_cast<int>(rc);
    } catch (...) {
        return -1;
    }
}
