"""Wire-format helpers shared by the legacy 68xx and AWR1843 UI paths."""

import struct


LEGACY_VITAL_SIGNS_TLV = 6
AWR1843_VITAL_SIGNS_TLV = 0x100

# VitalSignsDemo_OutputStats: HH f I HH followed by 28 floats.
VITAL_SIGNS_STRUCT = struct.Struct("<HHfIHH28f")
VITAL_SIGNS_PAYLOAD_SIZE = VITAL_SIGNS_STRUCT.size


_FLOAT_FIELDS = (
    "unwrapPhasePeak_mm",
    "outputFilterBreathOut",
    "outputFilterHeartOut",
    "heartRateEst_FFT",
    "heartRateEst_FFT_4Hz",
    "heartRateEst_xCorr",
    "heartRateEst_peakCount_filtered",
    "breathingRateEst_FFT",
    "breathingRateEst_xCorr",
    "breathingRateEst_peakCount",
    "confidenceMetricBreathOut",
    "confidenceMetricBreathOut_xCorr",
    "confidenceMetricHeartOut",
    "confidenceMetricHeartOut_4Hz",
    "confidenceMetricHeartOut_xCorr",
    "sumEnergyBreathWfm",
    "sumEnergyHeartWfm",
    "motionDetectedFlag",
    "breathingRateEst_harmonicEnergy",
    "heartRateEst_harmonicEnergy",
    "reserved7",
    "reserved8",
    "reserved9",
    "reserved10",
    "reserved11",
    "reserved12",
    "reserved13",
    "reserved14",
)


def is_vital_signs_tlv(tlv_type, tlv_length):
    """Recognize new 1843 output without confusing OOB stats type 6 with it."""
    return (
        tlv_type == AWR1843_VITAL_SIGNS_TLV
        or (
            tlv_type == LEGACY_VITAL_SIGNS_TLV
            and tlv_length >= VITAL_SIGNS_PAYLOAD_SIZE
        )
    )


def decode_vital_signs_payload(payload):
    """Decode the 128-byte C structure used by both firmware variants."""
    if len(payload) < VITAL_SIGNS_PAYLOAD_SIZE:
        raise ValueError(
            f"vital-signs payload is {len(payload)} bytes; "
            f"expected at least {VITAL_SIGNS_PAYLOAD_SIZE}"
        )

    values = VITAL_SIGNS_STRUCT.unpack_from(payload)
    decoded = {
        "rangeBinIndexMax": values[0],
        "rangeBinIndexPhase": values[1],
        "maxVal": values[2],
        "processingCyclesOut": values[3],
        "rangeBinStartIndex": values[4],
        "rangeBinEndIndex": values[5],
    }
    decoded.update(zip(_FLOAT_FIELDS, values[6:]))
    return decoded
