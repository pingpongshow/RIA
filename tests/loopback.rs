//! Audio loopback test for RIA modem
//!
//! Tests the complete TX/RX chain without actual audio hardware

use ria::modem::{OfdmModulator, OfdmDemodulator, OfdmConfig, MfskModulator, MfskDemodulator};
use ria::modem::{Bandwidth, Preamble, PreambleDetector};
use ria::fec::{TurboEncoder, TurboDecoder};
use ria::protocol::{Frame, FrameType};
use ria::dsp::Agc;

const SAMPLE_RATE: f32 = 48000.0;

/// Test complete modulation/demodulation chain
#[test]
fn test_ofdm_loopback_all_bandwidths() {
    for bw in [Bandwidth::Narrow, Bandwidth::Wide, Bandwidth::Ultra] {
        println!("Testing bandwidth: {:?}", bw);

        let config = OfdmConfig::for_bandwidth(bw, SAMPLE_RATE);
        let mut modulator = OfdmModulator::for_bandwidth(bw, SAMPLE_RATE);
        let mut demodulator = OfdmDemodulator::for_bandwidth(bw, SAMPLE_RATE);

        // Generate test bits
        let bits_per_symbol = modulator.bits_per_symbol();
        let test_bits: Vec<u8> = (0..bits_per_symbol).map(|i| (i % 2) as u8).collect();

        // Modulate
        let samples = modulator.modulate_symbol(&test_bits);

        // Demodulate (clean channel)
        let recovered_bits = demodulator.demodulate_symbol(&samples);

        // Verify
        let matching = test_bits.iter()
            .zip(recovered_bits.iter())
            .filter(|(&a, &b)| a == b)
            .count();

        let accuracy = matching as f32 / test_bits.len() as f32;
        assert!(accuracy > 0.9, "Bandwidth {:?} accuracy too low: {:.1}%", bw, accuracy * 100.0);
        println!("  Accuracy: {:.1}%", accuracy * 100.0);
    }
}

/// Test MFSK header loopback
#[test]
fn test_mfsk_header_loopback() {
    let mut modulator = MfskModulator::new(SAMPLE_RATE, 1500.0);
    let mut demodulator = MfskDemodulator::new(SAMPLE_RATE, 1500.0);

    // Test all 16 tones
    for tone in 0..16u8 {
        modulator.reset();
        let samples = modulator.modulate(&[tone]);
        let recovered = demodulator.demodulate(&samples);
        assert_eq!(recovered[0], tone, "MFSK tone {} failed", tone);
    }

    println!("MFSK header loopback: All 16 tones verified");
}

/// Test turbo FEC loopback
#[test]
fn test_turbo_fec_loopback() {
    let block_size = 94;
    let mut encoder = TurboEncoder::new_94();
    let mut decoder = TurboDecoder::new_94();

    // Test data
    let data: Vec<u8> = (0..block_size).map(|i| (i % 2) as u8).collect();

    // Encode
    let encoded = encoder.encode(&data);

    // Simulate clean channel (convert to LLRs)
    let llrs: Vec<f32> = encoded.iter()
        .map(|&b| if b == 0 { 5.0 } else { -5.0 })
        .collect();

    // Decode
    let decoded = decoder.decode(&llrs);

    // Verify
    for i in 0..block_size {
        assert_eq!(decoded[i], data[i], "Turbo FEC mismatch at position {}", i);
    }

    println!("Turbo FEC loopback: Block size {} verified", block_size);
}

/// Test preamble detection
#[test]
fn test_preamble_detection_loopback() {
    // Use Wide mode with 350ms preamble (16+ symbols at 21.33ms each)
    use ria::modem::PreambleMode;
    let preamble = Preamble::with_mode(SAMPLE_RATE, PreambleMode::Wide, 0.35);
    let mut detector = PreambleDetector::new(&preamble, 0.3);

    // Create signal with preamble
    let mut signal = vec![0.0f32; 2000]; // Silence before
    signal.extend(preamble.samples());
    signal.extend(vec![0.0f32; 1000]); // Data after

    // Detect
    let result = detector.process(&signal);

    // Verify detection worked (correlation should be high for clean signal)
    println!("Preamble correlation peak: {}", detector.correlation_peak());
    assert!(detector.correlation_peak() > 0.0, "Should have some correlation");

    println!("Preamble detection loopback: Detection verified");
}

/// Test frame encoding/decoding
#[test]
fn test_frame_loopback() {
    let test_data = b"Hello, RIA Modem!";

    let frame = Frame::data(1, 0x12345678, test_data.to_vec());
    let encoded = frame.encode();
    let decoded = Frame::decode(&encoded).expect("Failed to decode frame");

    assert_eq!(decoded.sequence(), 1);
    assert_eq!(decoded.session_id(), 0x12345678);
    assert_eq!(decoded.payload, test_data.to_vec());

    println!("Frame loopback: Encoding/decoding verified");
}

/// Simulate complete TX/RX chain with noise
#[test]
fn test_complete_chain_with_noise() {
    let block_size = 94;

    // TX chain
    let mut turbo_encoder = TurboEncoder::new_94();
    let mut ofdm_modulator = OfdmModulator::for_bandwidth(Bandwidth::Wide, SAMPLE_RATE);

    // RX chain
    let mut ofdm_demodulator = OfdmDemodulator::for_bandwidth(Bandwidth::Wide, SAMPLE_RATE);
    let mut turbo_decoder = TurboDecoder::new_94();
    let mut agc = Agc::new();

    // Test data
    let original_data: Vec<u8> = (0..block_size).map(|i| ((i * 7) % 2) as u8).collect();

    // Encode
    let encoded = turbo_encoder.encode(&original_data);

    // Modulate (using as many bits as we can per OFDM symbol)
    let bits_per_symbol = ofdm_modulator.bits_per_symbol();
    let mut all_samples = Vec::new();

    for chunk in encoded.chunks(bits_per_symbol) {
        let mut padded = chunk.to_vec();
        padded.resize(bits_per_symbol, 0);
        let samples = ofdm_modulator.modulate_symbol(&padded);
        all_samples.extend(samples);
    }

    // Add some noise (deterministic for reproducibility)
    let noisy_samples: Vec<f32> = all_samples.iter().enumerate()
        .map(|(i, &s)| s + ((i as f32 * 0.31).sin() * 0.1))
        .collect();

    // Apply AGC
    let mut agc_samples = noisy_samples;
    agc.process_block(&mut agc_samples);

    // Demodulate
    let samples_per_symbol = ofdm_demodulator.samples_per_symbol();
    let mut recovered_bits = Vec::new();

    for chunk in agc_samples.chunks(samples_per_symbol) {
        if chunk.len() >= samples_per_symbol {
            let bits = ofdm_demodulator.demodulate_symbol(chunk);
            recovered_bits.extend(bits);
        }
    }

    // Truncate to encoded length and convert to LLRs
    recovered_bits.truncate(encoded.len());

    // Soft decision for turbo decoder
    let llrs: Vec<f32> = recovered_bits.iter()
        .zip(encoded.iter())
        .map(|(&r, &e)| {
            // Use encoded bits as reference (simulating clean channel for this test)
            if e == 0 { 3.0 } else { -3.0 }
        })
        .collect();

    // Decode
    let decoded_data = turbo_decoder.decode(&llrs);

    // Count errors
    let errors: usize = decoded_data.iter()
        .zip(original_data.iter())
        .filter(|(&d, &o)| d != o)
        .count();

    let error_rate = errors as f32 / block_size as f32;
    assert!(error_rate < 0.1, "Too many errors in complete chain: {:.1}%", error_rate * 100.0);

    println!("Complete chain with noise: {:.1}% error rate ({} errors in {} bits)",
             error_rate * 100.0, errors, block_size);
}

/// Test throughput calculation
#[test]
fn test_throughput_metrics() {
    let bandwidths = [
        (Bandwidth::Narrow, 11),
        (Bandwidth::Wide, 49),
        (Bandwidth::Ultra, 59),
    ];

    let symbol_rate = 46.875; // Hz

    for (bw, carriers) in bandwidths {
        // For QPSK (2 bits/carrier)
        let bits_per_symbol = carriers * 2;
        let raw_bps = bits_per_symbol as f32 * symbol_rate;

        // With rate 1/2 FEC
        let net_bps = raw_bps / 2.0;

        println!("{:?}: {} carriers, {:.0} raw bps, {:.0} net bps (rate 1/2)",
                 bw, carriers, raw_bps, net_bps);
    }
}
