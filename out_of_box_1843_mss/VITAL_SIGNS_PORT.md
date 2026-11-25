# AWR1843 single-person vital-signs port

This port keeps the SDK 3.5 xWR18xx out-of-box object-detection chain intact and
adds a parallel vital-signs consumer immediately after the HWA range FFT.

## Processing path

1. `DPU_RangeProcHWA_process()` produces `DPIF_RADARCUBE_FORMAT_1` data.
2. DSS selects TX0 / Doppler-chirp 0 / RX0 and searches 0.30-0.90 m.
3. The strongest clutter-removed range bin is tracked every 128 frames.
4. Phase unwrap, phase difference, impulse rejection, breathing/heart IIR
   filters, 1024-point FFT, peak counting and autocorrelation are run at 20 Hz.
5. DSS sends the legacy-compatible `VitalSignsDemo_OutputStats` through DPM
   buffer 2. MSS emits it as UART TLV type `0x100` every frame.

The UART payload layout is declared in `../out_of_box_1843_vital_signs.h`.
Standard out-of-box TLVs remain unchanged.

## Bring-up

- Flash `isk/out_of_box_1843_isk.bin`.
- Send `profiles/vital_signs_20fps.cfg` over the CLI UART.
- Decode TLV `0x100` as `VitalSignsDemo_OutputStats`.

## Current fixed assumptions

- One subframe, one person and one selected range bin.
- 20 frames/s. The filter coefficients and spectral frequency conversion depend
  on this rate.
- Search range is 0.30-0.90 m.
- Phase channel is TX0/chirp0/RX0.

The vital-signs state is encapsulated in `VitalSigns_State`; the multi-person
step can replace the single global instance with two tracked states without
changing the DSS-to-MSS output transport design.
