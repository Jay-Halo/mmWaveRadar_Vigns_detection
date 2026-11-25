/*
 * Shared DSS/MSS interface for the AWR1843 vital-signs extension.
 *
 * Keep this structure binary compatible on both cores.  Its layout matches the
 * output used by the original vital_signs_68xx demo so existing host-side field
 * decoding can be reused.
 */
#ifndef OUT_OF_BOX_1843_VITAL_SIGNS_H
#define OUT_OF_BOX_1843_VITAL_SIGNS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Deliberately outside the SDK out-of-box TLV number range. */
#define MMWDEMO_OUTPUT_MSG_VITAL_SIGNS       (0x100U)

typedef struct VitalSignsDemo_OutputStats_t
{
    uint16_t rangeBinIndexMax;
    uint16_t rangeBinIndexPhase;
    float maxVal;
    uint32_t processingCyclesOut;
    uint16_t rangeBinStartIndex;
    uint16_t rangeBinEndIndex;
    float unwrapPhasePeak_mm;
    float outputFilterBreathOut;
    float outputFilterHeartOut;
    float heartRateEst_FFT;
    float heartRateEst_FFT_4Hz;
    float heartRateEst_xCorr;
    float heartRateEst_peakCount_filtered;
    float breathingRateEst_FFT;
    float breathingRateEst_xCorr;
    float breathingRateEst_peakCount;
    float confidenceMetricBreathOut;
    float confidenceMetricBreathOut_xCorr;
    float confidenceMetricHeartOut;
    float confidenceMetricHeartOut_4Hz;
    float confidenceMetricHeartOut_xCorr;
    float sumEnergyBreathWfm;
    float sumEnergyHeartWfm;
    float motionDetectedFlag;
    float breathingRateEst_harmonicEnergy;
    float heartRateEst_harmonicEnergy;
    float reserved7;
    float reserved8;
    float reserved9;
    float reserved10;
    float reserved11;
    float reserved12;
    float reserved13;
    float reserved14;
} VitalSignsDemo_OutputStats;

#ifdef __cplusplus
}
#endif

#endif /* OUT_OF_BOX_1843_VITAL_SIGNS_H */
