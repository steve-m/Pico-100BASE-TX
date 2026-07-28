#!/usr/bin/env python3

import socket
import numpy as np
import os
import sys
import time

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 42101
FIFO_PATH = "/tmp/gqrx_fifo"
SAMPLE_RATE = 2000000
CENTER_FREQUENCY = 792000
FRAME_COUNTER_MOD = 1 << 20
RF_GAIN = 100.0
DC_TRACKING_ALPHA = 0.001
DECIMATION = 40
OUTPUT_RATE = SAMPLE_RATE // DECIMATION
LOWPASS_CUTOFF = 3000
LOWPASS_TAPS = 129
CIC_STAGES = 5

def make_lowpass(cutoff_hz, sample_rate, tap_count):
    n = np.arange(tap_count, dtype=np.float32) - (tap_count - 1) / 2
    cutoff = cutoff_hz / sample_rate
    taps = 2 * cutoff * np.sinc(2 * cutoff * n)
    taps *= np.hamming(tap_count)
    taps /= np.sum(taps)
    return taps.astype(np.float32)

def main():
    # Setup Socket with a larger receive buffer to prevent drops
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2*1024*1024) # 2MB buffer
    except:
        pass # Some systems might restrict this

    sock.bind((UDP_IP, UDP_PORT))

    if os.path.exists(FIFO_PATH):
        os.remove(FIFO_PATH)
    os.mkfifo(FIFO_PATH)

    print(f"Pico-100BASE-TX High-Speed Bridge (NumPy Optimized)")
    print(f"Listening on port {UDP_PORT}...")
    print(f"ADC sample rate: {SAMPLE_RATE} S/s, output sample rate: {OUTPUT_RATE} S/s")
    print(f"Tuned frequency: {CENTER_FREQUENCY} Hz, channel low-pass: {LOWPASS_CUTOFF} Hz")

    phase_step = 2.0 * np.pi * CENTER_FREQUENCY / SAMPLE_RATE
    oscillator_indices = None
    dc_estimate = 2048.0
    cic_taps = np.full(DECIMATION, 1.0 / DECIMATION, dtype=np.float32)
    cic_states = [np.zeros(DECIMATION - 1, dtype=np.complex64) for _ in range(CIC_STAGES)]
    decimation_remainder = np.empty(0, dtype=np.complex64)
    lowpass_taps = make_lowpass(LOWPASS_CUTOFF, OUTPUT_RATE, LOWPASS_TAPS)
    lowpass_state = np.zeros(LOWPASS_TAPS - 1, dtype=np.complex64)
    last_frame_count = None
    stats_started = time.monotonic()
    stats_packets = 0
    stats_gaps = 0
    stats_samples = 0
    stats_rail_hits = 0
    stats_peak = 0.0

    try:
        print(f"Waiting for Gqrx to open {FIFO_PATH}...")
        with open(FIFO_PATH, "wb") as fifo:
            print("Streaming started. Use 'nice -n -10' for better performance if needed.")
            while True:
                # Receive packet
                data, _ = sock.recvfrom(2048)

                if len(data) > 4:
                    frame_count = data[0] | (data[1] << 8) | ((data[2] & 0x0f) << 16)
                    if last_frame_count is not None:
                        expected = (last_frame_count + 1) % FRAME_COUNTER_MOD
                        if frame_count != expected:
                            stats_gaps += (frame_count - expected) % FRAME_COUNTER_MOD
                    last_frame_count = frame_count

                    # Fast conversion using NumPy
                    # samples = (uint16_array - 2048) / 2048
                    samples = np.frombuffer(data, dtype='<u2', offset=4)
                    sample_count = len(samples)
                    sample_float = samples.astype(np.float32)
                    dc_estimate += DC_TRACKING_ALPHA * (float(np.mean(sample_float)) - dc_estimate)
                    real_samples = RF_GAIN * (sample_float - dc_estimate) / 2048.0
                    stats_packets += 1
                    stats_samples += sample_count
                    stats_rail_hits += int(np.count_nonzero((samples < 8) | (samples > 4087)))
                    stats_peak = max(stats_peak, float(np.max(np.abs(sample_float - dc_estimate))))

                    if oscillator_indices is None or len(oscillator_indices) != sample_count:
                        oscillator_indices = np.arange(sample_count, dtype=np.float32)

                    sample_offset = ((frame_count % FRAME_COUNTER_MOD) * sample_count) % SAMPLE_RATE
                    phase = 2.0 * np.pi * (sample_offset * CENTER_FREQUENCY % SAMPLE_RATE) / SAMPLE_RATE
                    angles = phase + phase_step * oscillator_indices
                    mixed = real_samples * np.exp(-1j * angles)

                    smoothed = mixed.astype(np.complex64)
                    for stage in range(CIC_STAGES):
                        stage_input = np.concatenate((cic_states[stage], smoothed))
                        smoothed = np.convolve(stage_input, cic_taps, mode='valid').astype(np.complex64)
                        cic_states[stage] = stage_input[-(DECIMATION - 1):]

                    decimation_input = np.concatenate((decimation_remainder, smoothed))
                    decimation_count = len(decimation_input) // DECIMATION
                    decimation_len = decimation_count * DECIMATION
                    if decimation_count == 0:
                        decimation_remainder = decimation_input
                        continue

                    decimated = decimation_input[:decimation_len:DECIMATION]
                    decimation_remainder = decimation_input[decimation_len:]

                    filter_input = np.concatenate((lowpass_state, decimated.astype(np.complex64)))
                    filtered = np.convolve(filter_input, lowpass_taps, mode='valid')
                    lowpass_state = filter_input[-(LOWPASS_TAPS - 1):]

                    # Create float32 IQ array (Size is 2x number of samples)
                    iq_data = np.empty(len(filtered) * 2, dtype=np.float32)

                    iq_data[0::2] = filtered.real
                    iq_data[1::2] = filtered.imag

                    # Direct binary write
                    fifo.write(iq_data.tobytes())

                    now = time.monotonic()
                    if now - stats_started >= 1.0:
                        rail_percent = 100.0 * stats_rail_hits / max(stats_samples, 1)
                        peak_percent = 100.0 * stats_peak / 2048.0
                        print(
                            f"ADC peak {peak_percent:5.1f}%FS, rails {rail_percent:6.3f}%, "
                            f"packet gaps {stats_gaps}, packets {stats_packets}",
                            flush=True
                        )
                        stats_started = now
                        stats_packets = 0
                        stats_gaps = 0
                        stats_samples = 0
                        stats_rail_hits = 0
                        stats_peak = 0.0

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if os.path.exists(FIFO_PATH):
            os.remove(FIFO_PATH)

if __name__ == "__main__":
    try:
        import numpy
    except ImportError:
        print("Error: NumPy is required for high-speed streaming.")
        print("Install it with: pip install numpy")
        sys.exit(1)
    main()
