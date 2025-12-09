# LPC Vocoder on ESP32 TTGO T-Display

A real-time Linear Predictive Coding (LPC) vocoder implementation for the ESP32 TTGO T-Display that transforms speech into a monotone robotic voice while preserving intelligibility.

## Table of Contents

1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Pin Configuration](#pin-configuration)
4. [Theory: Channel Vocoder vs LPC Vocoder](#theory-channel-vocoder-vs-lpc-vocoder)
5. [Why LPC?](#why-lpc)
6. [LPC Mathematics](#lpc-mathematics)
7. [Implementation Details](#implementation-details)
8. [Usage](#usage)
9. [Power Options](#power-options)
10. [Limitations and Future Work](#limitations-and-future-work)

---

## Overview

This project implements an LPC vocoder that:
- Records 3 seconds of speech at 8kHz sample rate
- Analyzes the spectral envelope using Linear Predictive Coding
- Resynthesizes speech using a fixed 100Hz pulse train excitation
- Outputs a monotone "robot voice" that remains intelligible

The result is the classic robotic voice effect heard in movies, music (Kraftwerk, Daft Punk), and military/space communications.

---

## Hardware Requirements

| Component | Purpose | Notes |
|-----------|---------|-------|
| ESP32 TTGO T-Display | Main processor + screen | Has built-in TFT display |
| MAX4466 Electret Microphone | Audio input | Adjustable gain, 3.3V compatible |
| Speaker or audio amplifier | Audio output | 8Ω speaker or line-level amp |
| 2x Momentary buttons | Record/Process triggers | Optional if using built-in buttons |
| Breadboard + jumper wires | Prototyping | |

---

## Pin Configuration

```
ESP32 TTGO T-Display
┌─────────────────────┐
│                     │
│  GPIO 36 ◄──────────┼── MAX4466 OUT (Audio In)
│  3.3V    ──────────►┼── MAX4466 VCC
│  GND     ──────────►┼── MAX4466 GND
│                     │
│  GPIO 25 ──────────►┼── Speaker/Amp (Audio Out)
│                     │
│  GPIO 13 ◄──────────┼── Record Button (to GND)
│  GPIO 15 ◄──────────┼── Process Button (to GND)
│                     │
└─────────────────────┘
```

**Alternative: Use built-in buttons**
```cpp
#define BUTTON1 0   // Built-in left button
#define BUTTON2 35  // Built-in right button
```

**GPIO Selection Notes:**
- GPIO 13, 15: Safe, no boot restrictions
- GPIO 2: Use with caution (strapping pin)
- GPIO 12: Avoid (can cause boot failure)

---

## Theory: Channel Vocoder vs LPC Vocoder

### Channel Vocoder (Traditional)

The classic channel vocoder splits audio into frequency bands:

```
Input Speech ──► Bank of Bandpass Filters ──► Envelope Followers ──► Envelopes
                      │                                                  │
                      │                                                  ▼
Carrier Signal ──► Bank of Bandpass Filters ──────────────────────► Multiply ──► Sum ──► Output
```

**How it works:**
1. Split speech into N frequency bands (e.g., 20 bands from 200Hz to 7kHz)
2. Extract the amplitude envelope of each band
3. Split a carrier signal (sawtooth, pulse, etc.) into the same bands
4. Multiply each carrier band by the corresponding speech envelope
5. Sum all bands together

**Problem:** This preserves spectral *energy* but loses spectral *shape*. Formant transitions (critical for consonants like "b" vs "d" vs "g") get smeared, making speech hard to understand.

### LPC Vocoder

LPC models speech production directly:

```
                    ┌─────────────────┐
                    │   Vocal Tract   │
Glottal Pulse ──►   │   (All-Pole     │  ──► Speech
                    │    Filter)      │
                    └─────────────────┘
```

The human voice is produced by:
1. **Excitation source**: Glottal pulses (voiced) or turbulent noise (unvoiced)
2. **Vocal tract filter**: Resonant cavity that shapes the spectrum

LPC separates these:
- **Analysis**: Extract filter coefficients that model the vocal tract
- **Synthesis**: Apply filter to a new excitation (fixed-pitch pulse train)

**Advantage:** Preserves the actual spectral *shape* (formants), not just band energies. Much more intelligible.

---

## Why LPC?

We started with a channel vocoder but encountered fundamental limitations:

| Issue | Channel Vocoder | LPC Vocoder |
|-------|-----------------|-------------|
| Intelligibility | Poor - loses formant transitions | Good - preserves spectral shape |
| Consonants | Smeared, hard to distinguish | Clear, properly shaped |
| Band count needed | 20+ bands, still limited | 16 coefficients captures all |
| Computational model | Generic frequency analysis | Matches speech production |

**Key insight:** Channel vocoders treat speech as generic audio. LPC treats speech as *speech* — a source-filter model that matches how humans actually produce sound.

The tradeoff: LPC sounds less "musical" than channel vocoders. For music (Daft Punk style), channel vocoders are preferred. For intelligible robot speech, LPC wins.

---

## LPC Mathematics

### The Source-Filter Model

Speech signal s(n) is modeled as an all-pole filter excited by e(n):

```
s(n) = e(n) + a₁·s(n-1) + a₂·s(n-2) + ... + aₚ·s(n-p)
```

Where:
- s(n) = output speech sample
- e(n) = excitation (pulse train or noise)
- a₁...aₚ = LPC coefficients (p = filter order, typically 16)

### Autocorrelation Method

To find coefficients, we compute the autocorrelation of each frame:

```
R(k) = Σ s(n) · s(n+k)    for n = 0 to N-k-1
```

This measures how similar the signal is to a delayed version of itself.

### Levinson-Durbin Recursion

Solves for LPC coefficients efficiently in O(p²) time:

```
For i = 1 to p:
    λᵢ = (R(i) - Σⱼ₌₁ⁱ⁻¹ aⱼ·R(i-j)) / Eᵢ₋₁
    
    aᵢ = λᵢ
    For j = 1 to i-1:
        aⱼ = aⱼ(prev) - λᵢ·aᵢ₋ⱼ(prev)
    
    Eᵢ = Eᵢ₋₁ · (1 - λᵢ²)
```

Where:
- λᵢ = reflection coefficient (partial correlation)
- Eᵢ = prediction error energy
- The final E gives us the gain

### Frame-by-Frame Processing

Speech is processed in overlapping frames:

```
|-------- Frame 1 --------|
        |-------- Frame 2 --------|
                |-------- Frame 3 --------|
```

Parameters:
- **Frame size**: 160 samples (20ms at 8kHz) — captures several pitch periods
- **Hop size**: 80 samples (10ms) — 50% overlap for smooth transitions
- **Window**: Hamming window reduces spectral leakage

```
w(n) = 0.54 - 0.46·cos(2πn / (N-1))
```

### Synthesis

For each frame:
1. Generate excitation: pulse train at 100Hz (one impulse every 80 samples)
2. Scale by frame energy
3. Filter through all-pole LPC filter:

```cpp
output = excitation;
for (j = 0; j < LPC_ORDER; j++) {
    output += lpc[j] * filterState[j];
}
// Update delay line
filterState = [output, filterState[0], filterState[1], ...]
```

---

## Implementation Details

### Memory Layout

```
Sample rate:     8,000 Hz
Record time:     3 seconds
Buffer size:     24,000 samples
Bytes per sample: 2 (int16_t)
Total RAM:       ~96KB for both buffers
```

### Processing Pipeline

```
┌──────────────────────────────────────────────────────────────────┐
│                        RECORDING                                  │
├──────────────────────────────────────────────────────────────────┤
│  analogRead(MIC_PIN) ──► inputBuffer[i] (12-bit ADC)             │
│  Loop at 8kHz (125μs delay)                                      │
└──────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│                       PREPROCESSING                               │
├──────────────────────────────────────────────────────────────────┤
│  1. Calculate DC offset: mean of all samples                     │
│  2. Remove DC: sample -= dcOffset                                │
│  3. Find peak amplitude                                          │
│  4. Normalize: sample /= peak                                    │
└──────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│                    FRAME PROCESSING (loop)                        │
├──────────────────────────────────────────────────────────────────┤
│  For each frame (160 samples, 80 sample hop):                    │
│                                                                  │
│  1. Apply Hamming window                                         │
│  2. Compute LPC coefficients via Levinson-Durbin                 │
│  3. Measure frame energy                                         │
│  4. Generate pulse train excitation (100Hz)                      │
│  5. Filter excitation through LPC all-pole filter                │
│  6. Accumulate to output buffer                                  │
└──────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌──────────────────────────────────────────────────────────────────┐
│                        PLAYBACK                                   │
├──────────────────────────────────────────────────────────────────┤
│  outputBuffer[i] >> 8 + 128 ──► dacWrite() (8-bit DAC)           │
│  Loop at 8kHz (125μs delay)                                      │
└──────────────────────────────────────────────────────────────────┘
```

### Key Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| LPC_ORDER | 16 | Captures formants F1-F4 plus spectral tilt |
| FRAME_SIZE | 160 (20ms) | Contains 2-3 pitch periods at typical male F0 |
| HOP_SIZE | 80 (10ms) | 50% overlap for smooth interpolation |
| Carrier freq | 100Hz | Low enough for rich harmonics, typical robot voice |
| Sample rate | 8kHz | Telephone quality, sufficient for speech |

### Why 16th Order?

The LPC order determines how many spectral peaks (formants) can be modeled:
- Order 10: ~4 formants (minimum for intelligibility)
- Order 16: ~6-7 formants (good quality)
- Order 20+: Diminishing returns, more computation

Rule of thumb: order = sample_rate/1000 + 4

For 8kHz: 8 + 4 = 12 minimum, 16 is comfortable.

---

## Usage

### Controls

| Button | Action |
|--------|--------|
| Button 1 (GPIO 13) | Start recording (3-second countdown) |
| Button 2 (GPIO 15) | Process and playback |

### Workflow

1. Power on the device
2. Press **Button 1** — countdown begins
3. Speak clearly during "RECORDING" (3 seconds)
4. Press **Button 2** — processing takes ~2-3 seconds
5. Audio plays back automatically
6. Press **Button 2** again to replay, or **Button 1** to record new

### Tips for Best Results

- Speak slowly and clearly
- Strong vowels work best: "HELLO", "ROBOT", "I AM A MACHINE"
- Keep microphone 2-3 inches from mouth
- Adjust MAX4466 gain pot if too quiet/loud
- Minimize background noise

---

## Power Options

| Source | Voltage | Connection | Notes |
|--------|---------|------------|-------|
| USB | 5V | USB-C port | Easiest option |
| USB Power Bank | 5V | USB-C port | Portable |
| 18650 LiPo | 3.7V | JST connector | T-Display has built-in charger |
| 3x AA | 4.5V | 5V pin | Works directly |
| 4x AA | 6V | 5V pin | Needs small regulator |
| 9V | 9V | Via buck converter | Set converter to 5V output |

**Warning:** Never connect more than 5V directly to the ESP32. Use a buck converter or regulator for higher voltage sources.

---

## Limitations and Future Work

### Current Limitations

1. **8-bit DAC output**: The ESP32's DAC is only 8-bit, limiting dynamic range. An external I2S DAC would improve quality significantly.

2. **Fixed pitch**: The 100Hz carrier is monotone by design. Variable pitch tracking could be added for more natural sound.

3. **No voiced/unvoiced detection**: All frames use pulse excitation. Adding noise excitation for fricatives (s, f, sh) would improve consonant clarity.

4. **Offline processing**: Records first, then processes. Real-time streaming would require optimized DSP.

### Potential Improvements

- **I2S audio output**: 16-bit DAC for better quality
- **Voiced/unvoiced detection**: Use zero-crossing rate or cepstral analysis
- **Pitch control**: Knob or button to adjust carrier frequency
- **Real-time mode**: Process in chunks with circular buffers
- **Carrier selection**: Sawtooth, square, or noise options
- **Blend control**: Mix dry signal for more intelligibility

---

## References

1. Makhoul, J. (1975). "Linear Prediction: A Tutorial Review." *Proceedings of the IEEE*, 63(4), 561-580.

2. Rabiner, L. R., & Schafer, R. W. (2010). *Theory and Applications of Digital Speech Processing*. Prentice Hall.

3. Dutoit, T. (1997). *An Introduction to Text-to-Speech Synthesis*. Springer.

4. Roads, C. (1996). "The Vocoder." *The Computer Music Tutorial*, MIT Press.

---

## License

MIT License - Feel free to use, modify, and distribute.

---

## Author

Capstone project for EECE curriculum, 2024-2025.
