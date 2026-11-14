# Opus cross-frame consistency checks

`tests/test_opus_cross_frame.c` is a deterministic command-line harness for
state that has to survive frame, packet and API call boundaries.  It uses
public encoder and decoder APIs, generated PCM and no external test vectors.
The stream format and the existing encoder quality controls are unchanged.

## Running

After building Opus (`make` with `Makefile.unix`, Autotools, Meson or CMake):

```sh
./test_opus_cross_frame --quick
./test_opus_cross_frame --full
./test_opus_cross_frame --bitrate=16000,32000,64000 --complexity=0,10
./test_opus_cross_frame --full --json
```

The default and `--quick` matrix is 12, 24 and 64 kb/s at complexity 4 and 10.
`--full` uses 8, 12, 16, 24, 32, 48, 64, 96 and 128 kb/s at complexity 0, 4
and 10.  `--seed=N` changes the deterministic PCM; use the same seed when
comparing runs.  `--verbose` prints every passing assertion.  `--list` prints
the scenarios and thresholds without running the tests.

The exit status is zero only when every exact-state and lossy-quality check
passes.

## Coverage

- `steady-cbr`: constant-bitrate frame sizes and decoded sample counts for
  20 ms calls versus 100 ms multi-frame packets.
- `chunk-20-vs-100`: the same 2 s PCM and 100 ms parameter schedule fed either
  as 100 ms packets or five 20 ms packets.  The schedule crosses CELT, SILK and
  stereo/mono decisions.
- `celt-2.5-vs-5ms`: 2.5 ms CELT frames against 5 ms frames over the same
  duration, including stereo phase.
- `dtx`: active audio around a one-second digital-silence interval, checking
  DTX packets, comfort noise, decode duration and return from DTX.
- `fec-plc`: deterministic packet drops, SILK in-band FEC where the following
  packet carries LBRR, and PLC where it does not.  The entropy-coder final
  range is checked after every received packet.
- `state-isolation` and `decoder-reset`: warm codec state, reset it, replay a
  fixed input and require byte-identical encoder output or sample-identical
  decoder output.
- `plc-duration-cap`: a two-second PLC request must return at most one second.

## Exact checks

These do not have a lossy tolerance:

- decoded samples equal the input duration;
- every packet has a valid, bounded encoded length;
- a reset encoder replayed with the same controls emits byte-identical packets;
- `OPUS_GET_FORCE_CHANNELS()` is unchanged by an internal stereo-to-mono
  transition;
- a reset decoder replayed with the same packets emits identical samples;
- after FEC or PLC, the next normally decoded packet reports the same
  `OPUS_GET_FINAL_RANGE()` as the encoder.

An exact failure is state leakage or corruption, not normal lossy coding.

## Lossy thresholds

The thresholds below are in `tests/test_opus_cross_frame.c`.  They apply only
after excluding intended mode-change settle windows.  Chunking comparisons
skip 160 ms after each 600 ms and 1200 ms schedule change; FEC/PLC comparisons
skip 200 ms after each induced loss.

| Metric | Limit | Meaning |
| --- | ---: | --- |
| Settled energy delta | 4.0 dB | Per-20 ms RMS between chunkings |
| Settled waveform correlation | 0.72 | Normalized sample correlation |
| Settled waveform RMS delta | 65% | Error RMS relative to reference RMS |
| Active energy jump | 24 dB | Adjacent-window RMS change |
| Stereo phase | 0.25 | Active-window L/R correlation; catches polarity inversion |
| CBR packet size | exact | All non-DTX packets in one CBR run are equal |
| CBR total bytes | 64 bytes | 20 ms and 100 ms packetization totals |

Special cases are explicit rather than silent:

- CBR waveform comparisons at 8 and 12 kb/s check duration, bytes, phase and
  state, but skip the waveform delta gate.  At those rates mode and
  stereo/mono decisions legitimately dominate the waveform difference.
- The chunking waveform gate is deliberately wider than the CBR gate because
  a 100 ms multi-frame packet and five 20 ms calls can make different mode
  decisions around a low-bitrate transition.  Reset replay and final range
  remain exact and catch actual leakage.
- The CBR energy-jump limit is 30 dB below 16 kb/s and 24 dB otherwise.
- DTX allows a 36 dB jump at the intentional silence-to-speech boundary and
  requires quiet-window RMS to be at most 25% of active RMS.
- FEC/PLC recovery permits 4 dB energy difference, correlation 0.70 and RMS
  delta 60% after the settle window.  The final range and sample count remain
  exact.
- The 2.5 ms versus 5 ms CELT comparison permits correlation 0.70 and RMS
  delta 80% because the frame partition intentionally changes quantization.

Do not raise these limits to hide a failure in the exact group.  A large
waveform difference plus a final-range mismatch or changed parameter is state
leakage; a bounded difference with matching range and reset replay is expected
lossy behavior.

## Reproducibility

PCM generation uses fixed integer and floating-point math from the supplied
seed.  Exact replay checks are valid within one build configuration.  Float
and fixed-point builds, different architectures and different compiler math
modes can produce different but valid codec output.  Cross-build comparisons
should use the lossy metrics, not byte equality.
