// Copyright 2025-2026 CEMAXECUTER LLC

//! RFNM SDR backend.
//!
//! Uses librfnm via a C++ shim (rfnm_shim.cpp) for USB streaming.
//! Low-level qbuf/dqbuf API: bypasses rx_stream's reorder/zero-padding
//! which livelocks at high sample rates.
//! CS16 format: 12-bit ADC samples left-shifted into i16 by the library.
//!
//! Target: full BLE band at 122.88 Msps (div=1) with GPU channelizer.
//! Also supports 61.44 Msps (div=2) for half-band coverage.

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

// RFNM API error codes (from rfnm_fw_api.h)
const RFNM_API_OK: c_int = 0;
const RFNM_API_DQBUF_OVERFLOW: c_int = 6;

// Channel constants (from constants.h)
const RFNM_CH0: u8 = 0x01;

#[repr(C)]
#[derive(Clone)]
struct RfnmShimInfo {
    serial: [c_char; 32],
    board_name: [c_char; 64],
    dcs_clk: u64,
    rx_ch_cnt: u8,
    gain_min: i8,
    gain_max: i8,
    path_preferred: c_int,
}

extern "C" {
    fn rfnm_shim_find(info: *mut RfnmShimInfo, max_devices: c_int) -> c_int;
    fn rfnm_shim_open(serial: *const c_char) -> *mut std::ffi::c_void;
    fn rfnm_shim_close(dev: *mut std::ffi::c_void);
    fn rfnm_shim_get_info(dev: *mut std::ffi::c_void, info: *mut RfnmShimInfo) -> c_int;
    fn rfnm_shim_set_stream_format_cs16(
        dev: *mut std::ffi::c_void,
        bufsize: *mut usize,
    ) -> c_int;
    fn rfnm_shim_configure_rx(
        dev: *mut std::ffi::c_void,
        channel: u32,
        freq: i64,
        gain: i8,
        samp_div_m: i16,
        samp_div_n: i16,
        lpf_bw: i16,
        path: c_int,
    ) -> c_int;
    fn rfnm_shim_rx_start(
        dev: *mut std::ffi::c_void,
        channel: u32,
        bufsize: usize,
    ) -> c_int;
    fn rfnm_shim_rx_stop(dev: *mut std::ffi::c_void, channel: u32) -> c_int;
    fn rfnm_shim_rx_read(
        dev: *mut std::ffi::c_void,
        ch_mask: u8,
        buf: *mut i16,
        max_elems: usize,
        elems_read: *mut usize,
        timeout_us: u32,
    ) -> c_int;
    fn rfnm_shim_set_rx_gain(
        dev: *mut std::ffi::c_void,
        channel: u32,
        gain: i8,
    ) -> c_int;
}

fn cchar_to_string(buf: &[c_char]) -> String {
    unsafe {
        CStr::from_ptr(buf.as_ptr())
            .to_string_lossy()
            .trim_end_matches('\0')
            .to_string()
    }
}

// ── Device enumeration ──────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct RfnmInfo {
    pub serial: String,
    pub board_name: String,
    pub dcs_clk: u64,
}

pub fn list_devices() -> Result<Vec<RfnmInfo>, String> {
    let mut raw = vec![
        RfnmShimInfo {
            serial: [0; 32],
            board_name: [0; 64],
            dcs_clk: 0,
            rx_ch_cnt: 0,
            gain_min: 0,
            gain_max: 0,
            path_preferred: 0,
        };
        8
    ];
    let count = unsafe { rfnm_shim_find(raw.as_mut_ptr(), 8) };
    if count < 0 {
        return Err("RFNM: device enumeration failed".to_string());
    }
    let mut devices = Vec::new();
    for i in 0..count as usize {
        devices.push(RfnmInfo {
            serial: cchar_to_string(&raw[i].serial),
            board_name: cchar_to_string(&raw[i].board_name),
            dcs_clk: raw[i].dcs_clk,
        });
    }
    Ok(devices)
}

/// Parse interface string: "rfnm" (first device) or "rfnm-SERIAL"
fn parse_serial(iface: &str) -> &str {
    if let Some(rest) = iface.strip_prefix("rfnm-") {
        rest
    } else {
        ""
    }
}

// ── Handle ──────────────────────────────────────────────────────────────────

pub struct RfnmHandle {
    dev: *mut std::ffi::c_void,
    max_samps: usize,
    ch_mask: u8,
    pub running: Arc<AtomicBool>,
    overflow_count: u64,
    recv_count: u64,
    /// IIR DC offset estimates for I and Q channels
    dc_i: f64,
    dc_q: f64,
}

unsafe impl Send for RfnmHandle {}

impl RfnmHandle {
    pub fn open(
        iface: &str,
        sample_rate: u32,
        center_freq: u64,
        gain: f64,
        _antenna: Option<&str>,
    ) -> Result<Self, String> {
        let serial = parse_serial(iface);
        let serial_c = CString::new(serial)
            .map_err(|e| format!("RFNM: invalid serial: {}", e))?;

        unsafe {
            let dev = rfnm_shim_open(serial_c.as_ptr());
            if dev.is_null() {
                return Err(format!(
                    "RFNM: failed to open device{}",
                    if serial.is_empty() {
                        String::new()
                    } else {
                        format!(" (serial={})", serial)
                    }
                ));
            }

            let mut info = RfnmShimInfo {
                serial: [0; 32],
                board_name: [0; 64],
                dcs_clk: 0,
                rx_ch_cnt: 0,
                gain_min: 0,
                gain_max: 0,
                path_preferred: 0,
            };
            if rfnm_shim_get_info(dev, &mut info) != 0 {
                rfnm_shim_close(dev);
                return Err("RFNM: failed to get device info".to_string());
            }

            let dcs_clk = info.dcs_clk;
            if dcs_clk == 0 {
                rfnm_shim_close(dev);
                return Err("RFNM: device reported 0 Hz base clock".to_string());
            }

            // Find best integer divisor for requested sample rate
            let target = sample_rate as u64;
            let mut best_div = 0u64;
            let mut best_err = f64::MAX;
            for &d in &[1u64, 2, 4, 8] {
                let actual = dcs_clk / d;
                let ratio = actual as f64 / target as f64;
                let err = (ratio - 1.0).abs();
                if err < best_err && err <= 0.03 {
                    best_err = err;
                    best_div = d;
                }
            }
            if best_div == 0 {
                rfnm_shim_close(dev);
                let rates: Vec<String> = [1u64, 2, 4, 8]
                    .iter()
                    .map(|d| format!("{}", dcs_clk / d / 1_000_000))
                    .collect();
                return Err(format!(
                    "RFNM: no compatible rate for -C {} (base clock {} MHz). Try: -C {}",
                    sample_rate / 1_000_000,
                    dcs_clk / 1_000_000,
                    rates.join(", -C "),
                ));
            }

            let actual_rate = dcs_clk / best_div;

            // Set CS16 stream format
            let mut bufsize: usize = 0;
            let r = rfnm_shim_set_stream_format_cs16(dev, &mut bufsize);
            if r != RFNM_API_OK {
                rfnm_shim_close(dev);
                return Err(format!("RFNM: set_stream_format_cs16 failed ({})", r));
            }
            let max_samps = bufsize / 4; // CS16 = 4 bytes per complex sample

            // Configure RX channel 0
            let gain_i8 = (gain as i8).clamp(info.gain_min, info.gain_max);
            let lpf_bw = (actual_rate / 1_000_000) as i16;
            let rf_path = info.path_preferred;
            let r = rfnm_shim_configure_rx(
                dev,
                0,
                center_freq as i64,
                gain_i8,
                best_div as i16,
                1,
                lpf_bw,
                rf_path,
            );
            if r != RFNM_API_OK {
                rfnm_shim_close(dev);
                return Err(format!("RFNM: configure_rx failed ({})", r));
            }

            // Start low-level streaming
            let r = rfnm_shim_rx_start(dev, 0, bufsize);
            if r != RFNM_API_OK {
                rfnm_shim_close(dev);
                return Err(format!("RFNM: rx_start failed ({})", r));
            }

            let dev_serial = cchar_to_string(&info.serial);
            let dev_name = cchar_to_string(&info.board_name);
            eprintln!(
                "RFNM: {} {} | {} MHz @ {} MS/s (div={}) | gain={} dB (range {}..{}) | path={} | CS16 buf={}",
                dev_name,
                dev_serial,
                center_freq / 1_000_000,
                actual_rate / 1_000_000,
                best_div,
                gain_i8,
                info.gain_min,
                info.gain_max,
                rf_path,
                max_samps,
            );

            Ok(Self {
                dev,
                max_samps,
                ch_mask: RFNM_CH0,
                running: Arc::new(AtomicBool::new(true)),
                overflow_count: 0,
                recv_count: 0,
                dc_i: 0.0,
                dc_q: 0.0,
            })
        }
    }

    /// Receive CS16 samples directly into caller's i16 buffer.
    /// Applies IIR DC offset removal (Lime RFIC has significant baseband DC).
    /// Returns number of complex samples received, or 0 on error/timeout.
    pub fn recv_into_i16(&mut self, buf: &mut [i16]) -> usize {
        let max = self.max_samps.min(buf.len() / 2);
        let mut elems_read: usize = 0;
        let r = unsafe {
            rfnm_shim_rx_read(
                self.dev,
                self.ch_mask,
                buf.as_mut_ptr(),
                max,
                &mut elems_read,
                20000,
            )
        };
        if r == RFNM_API_DQBUF_OVERFLOW {
            self.overflow_count += 1;
        }
        if r != RFNM_API_OK {
            return 0;
        }

        // DC offset removal: IIR low-pass tracks DC, subtract from each sample.
        // Alpha ~0.0005 gives ~2000 sample time constant (fast enough to converge
        // in first buffer, slow enough not to distort 1+ MHz BLE signals).
        // On first buffer, seed DC estimate from first sample for instant lock.
        const ALPHA: f64 = 0.0005;
        if self.recv_count == 0 && elems_read > 0 {
            self.dc_i = buf[0] as f64;
            self.dc_q = buf[1] as f64;
        }
        for i in 0..elems_read {
            let ri = i * 2;
            let qi = i * 2 + 1;
            self.dc_i += ALPHA * (buf[ri] as f64 - self.dc_i);
            self.dc_q += ALPHA * (buf[qi] as f64 - self.dc_q);
            buf[ri] = (buf[ri] as f64 - self.dc_i).round() as i16;
            buf[qi] = (buf[qi] as f64 - self.dc_q).round() as i16;
        }

        self.recv_count += 1;
        if self.recv_count <= 3 || self.recv_count % 5000 == 0 {
            let check = elems_read.min(1024);
            let mut sum_sq = 0u64;
            for i in 0..check {
                let re = buf[i * 2] as i64;
                let im = buf[i * 2 + 1] as i64;
                sum_sq += (re * re + im * im) as u64;
            }
            let rms = ((sum_sq as f64) / (check as f64)).sqrt();
            eprintln!(
                "[rfnm] recv #{}: {} samps, rms={:.1}, dc=({:.0},{:.0}), overflows={}",
                self.recv_count, elems_read, rms, self.dc_i, self.dc_q, self.overflow_count,
            );
        }
        elems_read
    }

    pub fn set_gain(&self, gain: f64) {
        unsafe {
            rfnm_shim_set_rx_gain(self.dev, 0, gain as i8);
        }
    }

    pub fn max_samps(&self) -> usize {
        self.max_samps
    }

    pub fn overflow_count(&self) -> u64 {
        self.overflow_count
    }
}

impl Drop for RfnmHandle {
    fn drop(&mut self) {
        unsafe {
            rfnm_shim_rx_stop(self.dev, 0);
            rfnm_shim_close(self.dev);
        }
        if self.overflow_count > 0 {
            eprintln!("RFNM: {} overflows during capture", self.overflow_count);
        }
    }
}
