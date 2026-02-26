// waveform_selection.hpp - Waveform and rate selection algorithm
//
// Centralized algorithm for selecting waveform mode and code rate
// based on SNR and fading index. Used by both protocol negotiation
// and ModemEngine.
//
// Based on testing with CFO=20Hz across AWGN/good/moderate channels (2026-01-29)

#pragma once

#include "protocol/frame_v2.hpp"  // WaveformMode
#include "ultra/types.hpp"        // CodeRate
#include "psk/multi_carrier_dpsk.hpp"  // SpreadingMode

namespace ultra {
namespace protocol {

// Waveform + rate + modulation recommendation
struct WaveformRecommendation {
    WaveformMode waveform;
    Modulation modulation;
    CodeRate rate;
    float estimated_throughput_bps;
    int num_carriers = 10;           // For MC-DPSK: 5=narrow, 10=standard
    SpreadingMode spreading = SpreadingMode::NONE;  // For MC-DPSK: time-domain repetition
};

// Shared helper: Select code rate for OFDM modes based on SNR and fading
// This is the SINGLE SOURCE OF TRUTH for rate selection thresholds.
// Both recommendWaveformAndRate() and recommendDataMode() use this.
//
// Fading index now combines freq_cv + temporal_cv (Doppler measurement).
// Thresholds (2026-02-10) - Full rate ladder:
//   AWGN only (< 0.15):             R3/4 @ SNR >= 20 (10/10 seeds, 0 retx)
//   Good fading or better (< 0.65): R2/3 @ SNR >= 20 (30/30 seeds, 0 retx)
//   Good fading or better (< 0.65): R1/2 @ SNR >= 15 (5/5 seeds, 0 retx)
//   Moderate fading (< 1.10):       R1/2 @ SNR >= 15 (6/6 seeds, 100% delivery)
//   Heavy+ (>= 1.10):              R1/4 only
//
// R3/4 verified (2026-02-10):
//   DQPSK R3/4 AWGN SNR=20: 10/10 seeds PASS, 0 retransmissions
//   DQPSK R3/4 Good fading: FAILS (23 retx / 5 seeds) — AWGN only!
//   Payload: 243 bytes/frame — 23% gain over R2/3
// R2/3 verified (2026-02-10):
//   DQPSK R2/3 Good fading SNR=20: 30/30 seeds PASS, 0 retransmissions
//   Payload: 197 bytes/frame — 40% gain over R1/2
// R1/2 verified (2026-02-10):
//   DQPSK R1/2 Good fading SNR=15: 5/5 seeds PASS, 0 retransmissions
inline CodeRate selectOFDMCodeRate(float snr_db, float fading_index) {
    // AWGN only: R3/4 at SNR >= 20 (too many retx on fading)
    if (fading_index < 0.15f && snr_db >= 20.0f) return CodeRate::R3_4;

    // Good fading or better: R2/3 at SNR >= 20
    if (fading_index < 0.65f && snr_db >= 20.0f) return CodeRate::R2_3;

    // Good-to-moderate fading: R1/2 at SNR >= 15
    if (fading_index < 1.10f && snr_db >= 15.0f) return CodeRate::R1_2;

    // All other conditions: R1/4 (most robust)
    return CodeRate::R1_4;
}

// Cap initial OFDM rate during handshake bootstrap using only chirp-era metrics.
// This avoids optimistic R2/3 starts when first post-connect OFDM quality is unknown.
inline CodeRate capInitialOFDMRate(float snr_db, float fading_index, CodeRate candidate) {
    if (candidate == CodeRate::R3_4) {
        // Keep R3/4 for near-ideal channels only.
        if (fading_index >= 0.05f || snr_db < 24.0f) {
            return CodeRate::R2_3;
        }
        return candidate;
    }

    if (candidate == CodeRate::R2_3) {
        // Conservative bootstrap for OTA: upgrade after channel proves itself.
        if (fading_index >= 0.45f || snr_db < 24.0f) {
            return CodeRate::R1_2;
        }
    }

    return candidate;
}

// Recommend waveform, modulation and rate based on SNR and fading index
//
// Fading index now combines freq_cv + temporal_cv (Doppler measurement).
// Key findings from testing (2026-02-11, updated 2026-02-23):
// - SNR < 5 dB: 10-carrier DBPSK R1/4 (~469 bps, floor -4 dB)
// - SNR 5-10 dB: MC-DPSK DQPSK R1/4 (~938 bps, floor +5 dB)
// - SNR >= 25 dB + AWGN: OFDM_CHIRP QAM64 R3/4 (~7200 bps)
// - SNR >= 22 dB + AWGN: OFDM_CHIRP QAM32 R3/4 (~6000 bps)
// - SNR >= 18 dB + AWGN: OFDM_CHIRP QAM16 R3/4 (~4800 bps)
// - SNR >= 22 dB + good fading: OFDM_CHIRP QAM16 R2/3 (~4000 bps)
// - SNR >= 20 dB + AWGN: OFDM_CHIRP DQPSK R3/4 (~3900 bps)
// - SNR >= 20 dB + good fading: OFDM_CHIRP DQPSK R2/3 (~3200 bps)
// - SNR >= 15 dB + good/moderate fading: OFDM_CHIRP DQPSK R1/2 (~2300 bps)
// - SNR >= 10 dB + good/moderate fading: OFDM_CHIRP DQPSK R1/4 (~1150 bps, 30/30 seeds)
// - Heavy+ fading (>= 1.10): DQPSK R1/4 only (~1150 bps)
//
// Calibrated fading thresholds:
//   < 0.15: True AWGN, < 0.65: Good, >= 0.65: Moderate+
//
// Low-SNR MC-DPSK modes with ZC preamble (2026-02-23):
//   10-carrier DBPSK R1/4: floor -4 dB, ~469 bps (DEFAULT for connection)
//   10-carrier DQPSK R1/4: floor +5 dB, ~938 bps
//
// Time-domain spreading modes (2026-02-24):
//   DBPSK 4× spread: floor -14 dB, ~117 bps (extreme weak signal)
//   DBPSK 2× spread: floor -8 dB, ~235 bps (very weak signal)
//   DBPSK no spread: floor -4 dB, ~469 bps (default)
// Thresholds have 1 dB margin above verified floor.
inline WaveformRecommendation recommendWaveformAndRate(float snr_db, float fading_index) {
    WaveformRecommendation rec;

    if (snr_db < -7.0f) {
        // Extreme low SNR: 4× spreading DBPSK (~117 bps)
        // Verified floor: -14 dB (test_spreading.cpp)
        // Threshold: -7 dB provides 1 dB margin above 2× floor (-8 dB)
        rec.waveform = WaveformMode::MC_DPSK;
        rec.modulation = Modulation::DBPSK;
        rec.rate = CodeRate::R1_4;
        rec.spreading = SpreadingMode::TIME_4X;
        rec.estimated_throughput_bps = 117.0f;
        rec.num_carriers = 10;
    }
    else if (snr_db < -3.0f) {
        // Very low SNR: 2× spreading DBPSK (~235 bps)
        // Verified floor: -8 dB (test_spreading.cpp)
        // Threshold: -3 dB provides 1 dB margin above no-spread floor (-4 dB)
        rec.waveform = WaveformMode::MC_DPSK;
        rec.modulation = Modulation::DBPSK;
        rec.rate = CodeRate::R1_4;
        rec.spreading = SpreadingMode::TIME_2X;
        rec.estimated_throughput_bps = 235.0f;
        rec.num_carriers = 10;
    }
    else if (snr_db < 5.0f) {
        // Low SNR: 10-carrier DBPSK R1/4 (~469 bps)
        // Verified floor: -4 dB (cli_simulator full protocol test)
        // This is 9 dB more robust than DQPSK
        rec.waveform = WaveformMode::MC_DPSK;
        rec.modulation = Modulation::DBPSK;
        rec.rate = CodeRate::R1_4;
        rec.spreading = SpreadingMode::NONE;
        rec.estimated_throughput_bps = 469.0f;
        rec.num_carriers = 10;
    }
    else if (snr_db < 10.0f) {
        // Medium-low SNR: MC-DPSK DQPSK with 10 carriers (~938 bps)
        // Verified floor: +5 dB
        rec.waveform = WaveformMode::MC_DPSK;
        rec.modulation = Modulation::DQPSK;
        rec.rate = CodeRate::R1_4;
        rec.estimated_throughput_bps = 938.0f;
        rec.num_carriers = 10;
    }
    else if (fading_index < 0.15f) {
        // True AWGN (no fading) - can use QAM modes based on SNR
        rec.waveform = WaveformMode::OFDM_CHIRP;
        if (snr_db >= 25.0f) {
            rec.modulation = Modulation::QAM64;
            rec.rate = CodeRate::R3_4;
            rec.estimated_throughput_bps = 7200.0f;
        } else if (snr_db >= 22.0f) {
            rec.modulation = Modulation::QAM32;
            rec.rate = CodeRate::R3_4;
            rec.estimated_throughput_bps = 6000.0f;
        } else if (snr_db >= 18.0f) {
            rec.modulation = Modulation::QAM16;
            rec.rate = selectOFDMCodeRate(snr_db, fading_index);
            rec.estimated_throughput_bps = (rec.rate == CodeRate::R3_4) ? 4800.0f :
                                           (rec.rate == CodeRate::R2_3) ? 4000.0f :
                                           (rec.rate == CodeRate::R1_2) ? 3000.0f : 1500.0f;
        } else {
            rec.modulation = Modulation::DQPSK;
            rec.rate = selectOFDMCodeRate(snr_db, fading_index);
            rec.estimated_throughput_bps = (rec.rate == CodeRate::R3_4) ? 3900.0f :
                                           (rec.rate == CodeRate::R2_3) ? 3200.0f :
                                           (rec.rate == CodeRate::R1_2) ? 2300.0f : 1150.0f;
        }
    }
    else if (fading_index < 0.65f && snr_db >= 10.0f) {
        // Good fading: QAM16 at high SNR, otherwise DQPSK
        rec.waveform = WaveformMode::OFDM_CHIRP;
        if (snr_db >= 22.0f) {
            rec.modulation = Modulation::QAM16;
            rec.rate = CodeRate::R2_3;  // Conservative rate for fading
            rec.estimated_throughput_bps = 4000.0f;
        } else {
            rec.modulation = Modulation::DQPSK;
            rec.rate = selectOFDMCodeRate(snr_db, fading_index);
            rec.estimated_throughput_bps = (rec.rate == CodeRate::R3_4) ? 3900.0f :
                                           (rec.rate == CodeRate::R2_3) ? 3200.0f :
                                           (rec.rate == CodeRate::R1_2) ? 2300.0f : 1150.0f;
        }
    }
    else if (fading_index < 1.10f && snr_db >= 10.0f) {
        // Moderate fading: DQPSK only (QAM not reliable)
        rec.waveform = WaveformMode::OFDM_CHIRP;
        rec.modulation = Modulation::DQPSK;
        rec.rate = selectOFDMCodeRate(snr_db, fading_index);
        rec.estimated_throughput_bps = (rec.rate == CodeRate::R3_4) ? 3900.0f :
                                       (rec.rate == CodeRate::R2_3) ? 3200.0f :
                                       (rec.rate == CodeRate::R1_2) ? 2300.0f : 1150.0f;
    }
    else if (snr_db >= 10.0f) {
        // Heavy fading at SNR >= 10: DQPSK R1/4 still viable
        rec.waveform = WaveformMode::OFDM_CHIRP;
        rec.modulation = Modulation::DQPSK;
        rec.rate = CodeRate::R1_4;
        rec.estimated_throughput_bps = 1150.0f;
    }
    else {
        // Very heavy fading or low SNR: MC-DPSK
        rec.waveform = WaveformMode::MC_DPSK;
        rec.modulation = Modulation::DQPSK;
        rec.rate = CodeRate::R1_4;
        rec.estimated_throughput_bps = 938.0f;
    }

    return rec;
}

// Recommend modulation and code rate for data mode within an established connection
// This is used after waveform negotiation to set the data transmission parameters.
// Uses selectOFDMCodeRate() for rate selection to stay consistent with recommendWaveformAndRate().
//
// QAM Modulation Ladder (2026-02-22):
//   - QAM64: 6 bits/carrier, ~7200 bps @ R3/4, requires SNR >= 25 dB AWGN only
//   - QAM32: 5 bits/carrier, ~6000 bps @ R3/4, requires SNR >= 22 dB AWGN only
//   - QAM16: 4 bits/carrier, ~4800 bps @ R3/4, requires SNR >= 18 dB AWGN or >= 22 dB good fading
//   - DQPSK: 2 bits/carrier, ~3900 bps @ R3/4, works on all channels
//
// Throughput estimates (51 data carriers, ~37.5 sym/s raw):
//   QAM64 R3/4: 306 bits/sym × 37.5 × 0.75 × ~0.85 overhead = ~7200 bps
//   QAM32 R3/4: 255 bits/sym × 37.5 × 0.75 × ~0.85 overhead = ~6000 bps
//   QAM16 R3/4: 204 bits/sym × 37.5 × 0.75 × ~0.85 overhead = ~4800 bps
//   DQPSK R3/4: 106 bits/sym × 37.5 × 0.75 × ~0.85 overhead = ~2500 bps (actual ~3900 w/ 53 carriers)
//
// For OFDM modes: modulation selected based on SNR and fading, rate from selectOFDMCodeRate()
// For MC-DPSK: Modulation and spreading based on SNR
//
// Time-domain spreading modes for MC-DPSK (2026-02-24):
//   DBPSK 4× spread: floor -14 dB, ~117 bps (extreme weak signal)
//   DBPSK 2× spread: floor -8 dB, ~235 bps (very weak signal)
//   DBPSK no spread: floor -4 dB, ~469 bps (default)
//   DQPSK no spread: floor +5 dB, ~938 bps (medium SNR)
// Thresholds have 1 dB margin above verified floor.
//
inline void recommendDataMode(float snr_db, WaveformMode waveform,
                               Modulation& mod, CodeRate& rate, float fading_index = 0.0f,
                               int* out_num_carriers = nullptr,
                               SpreadingMode* out_spreading = nullptr) {
    // MC-DPSK: Select modulation and spreading based on SNR for low SNR operation
    if (waveform == WaveformMode::MC_DPSK) {
        rate = CodeRate::R1_4;  // Always R1/4 for MC-DPSK
        if (out_num_carriers) *out_num_carriers = 10;

        if (snr_db < -7.0f) {
            // Extreme low SNR: DBPSK with 4× spreading (~117 bps, floor -14 dB)
            // Threshold: -7 dB provides 1 dB margin above 2× floor (-8 dB)
            mod = Modulation::DBPSK;
            if (out_spreading) *out_spreading = SpreadingMode::TIME_4X;
        } else if (snr_db < -3.0f) {
            // Very low SNR: DBPSK with 2× spreading (~235 bps, floor -8 dB)
            // Threshold: -3 dB provides 1 dB margin above no-spread floor (-4 dB)
            mod = Modulation::DBPSK;
            if (out_spreading) *out_spreading = SpreadingMode::TIME_2X;
        } else if (snr_db < 5.0f) {
            // Low SNR: DBPSK with no spreading (~469 bps, floor -4 dB)
            mod = Modulation::DBPSK;
            if (out_spreading) *out_spreading = SpreadingMode::NONE;
        } else {
            // Medium SNR: DQPSK with no spreading (~938 bps, floor +5 dB)
            mod = Modulation::DQPSK;
            if (out_spreading) *out_spreading = SpreadingMode::NONE;
        }
        return;
    }

    // OFDM modes don't use spreading
    if (out_spreading) *out_spreading = SpreadingMode::NONE;

    // OFDM modes: select modulation based on channel conditions
    // Higher-order QAM requires better SNR and lower fading

    if (fading_index < 0.15f) {
        // AWGN channel - can use highest modulation supported by SNR
        if (snr_db >= 25.0f) {
            mod = Modulation::QAM64;
            rate = CodeRate::R3_4;
            return;
        } else if (snr_db >= 22.0f) {
            mod = Modulation::QAM32;
            rate = CodeRate::R3_4;
            return;
        } else if (snr_db >= 18.0f) {
            mod = Modulation::QAM16;
            rate = selectOFDMCodeRate(snr_db, fading_index);
            return;
        }
    } else if (fading_index < 0.65f) {
        // Good fading - QAM16 only at high SNR, otherwise DQPSK
        if (snr_db >= 22.0f) {
            mod = Modulation::QAM16;
            rate = CodeRate::R2_3;  // Conservative rate for fading
            return;
        }
    }

    // Default: DQPSK with rate ladder (most robust)
    mod = Modulation::DQPSK;
    rate = selectOFDMCodeRate(snr_db, fading_index);
}

} // namespace protocol
} // namespace ultra
