/*
 * Single-person vital-signs processing adapter for the xWR18xx out-of-box DPC.
 *
 * The RF range FFT remains in the SDK HWA RangeProc DPU.  This module consumes
 * one stable virtual channel from DPIF_RADARCUBE_FORMAT_1 and implements the
 * original demo's range selection -> phase -> filtering -> rate estimation
 * chain.  The state is explicitly encapsulated so a later multi-person version
 * can instantiate one state per tracked range/angle cell.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <ti/mathlib/mathlib.h>

#include "vital_signs.h"

#define VS_MAX_RANGE_BINS          (1024U)
#define VS_BREATH_BUFFER_SIZE      (256U)
#define VS_HEART_BUFFER_SIZE       (512U)
#define VS_FFT_SIZE                (1024U)
#define VS_SAMPLE_RATE_HZ          (20.0f)
#define VS_START_RANGE_M           (0.30f)
#define VS_END_RANGE_M             (0.90f)
#define VS_TRACKING_PERIOD         (128U)
#define VS_PI                      (3.14159265358979323846f)
#define VS_TWO_PI                  (2.0f * VS_PI)
#define VS_IMPULSE_THRESHOLD       (1.5f)

typedef struct VitalSigns_ComplexFloat_t
{
    float real;
    float imag;
} VitalSigns_ComplexFloat;

struct VitalSigns_State_t
{
    uint16_t numRangeBins;
    uint16_t rangeBinStart;
    uint16_t rangeBinEnd;
    uint16_t trackedRangeBin;
    uint16_t breathWriteIndex;
    uint16_t heartWriteIndex;
    uint32_t frameCount;

    float rangeMeanReal[VS_MAX_RANGE_BINS];
    float rangeMeanImag[VS_MAX_RANGE_BINS];
    float breathBuffer[VS_BREATH_BUFFER_SIZE];
    float heartBuffer[VS_HEART_BUFFER_SIZE];
    VitalSigns_ComplexFloat fftBuffer[VS_FFT_SIZE];

    float breathDelay[14];
    float heartDelay[26];
    float phasePrevious;
    float phaseCorrection;
    float unwrappedPrevious;
    float impulsePrevious2;
    float impulsePrevious1;
    float impulseCurrent;
    float breathEnergy;
    float heartEnergy;
    float peakCountHistory[10];
    uint16_t peakCountHistoryIndex;

    VitalSignsDemo_OutputStats output;
};

static const float gBreathCoefficients[12] =
{
    1.0000f, 0.0f, -1.0000f, 1.0000f, -1.9632f, 0.9644f,
    1.0000f, 0.0f, -1.0000f, 1.0000f, -1.8501f, 0.8681f
};

static const float gBreathScale[3] = {0.0602f, 0.0602f, 1.0000f};

static const float gHeartCoefficients[24] =
{
    1.0000f, 0.0f, -1.0000f, 1.0000f, -0.5306f, 0.5888f,
    1.0000f, 0.0f, -1.0000f, 1.0000f, -1.8069f, 0.8689f,
    1.0000f, 0.0f, -1.0000f, 1.0000f, -1.4991f, 0.5887f,
    1.0000f, 0.0f, -1.0000f, 1.0000f, -0.6654f, 0.2099f
};

static const float gHeartScale[5] =
    {0.4188f, 0.4188f, 0.3611f, 0.3611f, 1.0000f};

static float VitalSigns_iir(float input, const float *coefficients,
                            const float *scale, float *delay,
                            uint16_t stages)
{
    uint16_t stage;
    float output = input;

    for (stage = 0U; stage < stages; stage++)
    {
        uint16_t coefficientIndex = (uint16_t)(stage * 6U);
        uint16_t delayIndex = (uint16_t)(stage * 6U);
        float state = scale[stage] * output
                    - coefficients[coefficientIndex + 4U] * delay[delayIndex + 1U]
                    - coefficients[coefficientIndex + 5U] * delay[delayIndex + 2U];

        output = coefficients[coefficientIndex] * state
               + coefficients[coefficientIndex + 1U] * delay[delayIndex + 1U]
               + coefficients[coefficientIndex + 2U] * delay[delayIndex + 2U];
        delay[delayIndex + 2U] = delay[delayIndex + 1U];
        delay[delayIndex + 1U] = state;
    }
    return output * scale[stages];
}

static float VitalSigns_unwrap(float phase, float previous,
                               float *correction)
{
    float difference = phase - previous;

    if (difference > VS_PI)
    {
        *correction -= VS_TWO_PI;
    }
    else if (difference < -VS_PI)
    {
        *correction += VS_TWO_PI;
    }
    return phase + *correction;
}

static float VitalSigns_removeImpulse(float previous2, float previous1,
                                      float current)
{
    if ((fabsf(current - previous1) > VS_IMPULSE_THRESHOLD) &&
        (fabsf(previous1 - previous2) > VS_IMPULSE_THRESHOLD) &&
        (((current - previous1) * (previous1 - previous2)) < 0.0f))
    {
        return 0.5f * (current + previous2);
    }
    return previous1;
}

static void VitalSigns_fft(VitalSigns_ComplexFloat *data)
{
    uint16_t i;
    uint16_t j = 0U;
    uint16_t length;

    for (i = 1U; i < VS_FFT_SIZE; i++)
    {
        uint16_t bit = (uint16_t)(VS_FFT_SIZE >> 1U);
        while ((j & bit) != 0U)
        {
            j = (uint16_t)(j ^ bit);
            bit = (uint16_t)(bit >> 1U);
        }
        j = (uint16_t)(j ^ bit);
        if (i < j)
        {
            VitalSigns_ComplexFloat temporary = data[i];
            data[i] = data[j];
            data[j] = temporary;
        }
    }

    for (length = 2U; length <= VS_FFT_SIZE; length = (uint16_t)(length << 1U))
    {
        uint16_t halfLength = (uint16_t)(length >> 1U);
        float angle = -VS_TWO_PI / (float)length;
        float stepReal = cossp(angle);
        float stepImag = sinsp(angle);
        uint16_t base;

        for (base = 0U; base < VS_FFT_SIZE; base = (uint16_t)(base + length))
        {
            float twiddleReal = 1.0f;
            float twiddleImag = 0.0f;
            uint16_t k;

            for (k = 0U; k < halfLength; k++)
            {
                uint16_t evenIndex = (uint16_t)(base + k);
                uint16_t oddIndex = (uint16_t)(evenIndex + halfLength);
                float oddReal = data[oddIndex].real * twiddleReal
                              - data[oddIndex].imag * twiddleImag;
                float oddImag = data[oddIndex].real * twiddleImag
                              + data[oddIndex].imag * twiddleReal;
                float evenReal = data[evenIndex].real;
                float evenImag = data[evenIndex].imag;
                float nextTwiddleReal;

                data[evenIndex].real = evenReal + oddReal;
                data[evenIndex].imag = evenImag + oddImag;
                data[oddIndex].real = evenReal - oddReal;
                data[oddIndex].imag = evenImag - oddImag;

                nextTwiddleReal = twiddleReal * stepReal - twiddleImag * stepImag;
                twiddleImag = twiddleReal * stepImag + twiddleImag * stepReal;
                twiddleReal = nextTwiddleReal;
            }
        }
    }
}

static float VitalSigns_getRingSample(const float *buffer, uint16_t size,
                                      uint16_t writeIndex, uint16_t index)
{
    uint16_t oldest = writeIndex;
    return buffer[(uint16_t)((oldest + index) % size)];
}

static float VitalSigns_spectrumEstimate(VitalSigns_State *state,
                                         const float *waveform,
                                         uint16_t waveformSize,
                                         uint16_t writeIndex,
                                         float minimumHz,
                                         float maximumHz,
                                         uint16_t applyHeartWindow,
                                         float *confidence)
{
    uint16_t index;
    uint16_t startBin = (uint16_t)(minimumHz * VS_FFT_SIZE / VS_SAMPLE_RATE_HZ + 0.5f);
    uint16_t endBin = (uint16_t)(maximumHz * VS_FFT_SIZE / VS_SAMPLE_RATE_HZ + 0.5f);
    uint16_t maximumBin = startBin;
    float maximumPower = 0.0f;
    float bandPower = 0.0f;
    float peakPower = 0.0f;

    memset(state->fftBuffer, 0, sizeof(state->fftBuffer));
    for (index = 0U; index < waveformSize; index++)
    {
        float sample = VitalSigns_getRingSample(waveform, waveformSize,
                                                writeIndex, index);
        if (applyHeartWindow != 0U)
        {
            /* The original demo tapers the first/last 16 samples. */
            if ((index < 16U) || (index >= (uint16_t)(waveformSize - 16U)))
            {
                uint16_t edge = index < 16U ? index :
                    (uint16_t)(waveformSize - 1U - index);
                sample *= 0.54f - 0.46f * cossp(VS_PI * (float)(edge + 1U) / 17.0f);
            }
        }
        state->fftBuffer[index].real = sample;
    }

    VitalSigns_fft(state->fftBuffer);
    for (index = startBin; index <= endBin; index++)
    {
        float power = state->fftBuffer[index].real * state->fftBuffer[index].real
                    + state->fftBuffer[index].imag * state->fftBuffer[index].imag;
        bandPower += power;
        if (power > maximumPower)
        {
            maximumPower = power;
            maximumBin = index;
        }
    }

    for (index = maximumBin > 2U ? (uint16_t)(maximumBin - 2U) : 0U;
         (index <= (uint16_t)(maximumBin + 2U)) && (index <= endBin); index++)
    {
        peakPower += state->fftBuffer[index].real * state->fftBuffer[index].real
                   + state->fftBuffer[index].imag * state->fftBuffer[index].imag;
    }

    *confidence = bandPower > 0.0f ? peakPower / bandPower : 0.0f;
    return (float)maximumBin * VS_SAMPLE_RATE_HZ * 60.0f / VS_FFT_SIZE;
}

static float VitalSigns_peakCount(const float *waveform, uint16_t size,
                                  uint16_t writeIndex)
{
    uint16_t index;
    uint16_t count = 0U;
    float previous2 = VitalSigns_getRingSample(waveform, size, writeIndex, 0U);
    float previous1 = VitalSigns_getRingSample(waveform, size, writeIndex, 1U);

    for (index = 2U; index < size; index++)
    {
        float current = VitalSigns_getRingSample(waveform, size, writeIndex, index);
        if ((previous1 > previous2) && (previous1 >= current))
        {
            count++;
        }
        previous2 = previous1;
        previous1 = current;
    }
    return 60.0f * (float)count * VS_SAMPLE_RATE_HZ / (float)size;
}

static float VitalSigns_autocorrelationRate(const float *waveform,
                                            uint16_t size,
                                            uint16_t writeIndex,
                                            float minimumHz,
                                            float maximumHz,
                                            float *confidence)
{
    uint16_t minimumLag = (uint16_t)(VS_SAMPLE_RATE_HZ / maximumHz);
    uint16_t maximumLag = (uint16_t)(VS_SAMPLE_RATE_HZ / minimumHz);
    uint16_t lag;
    uint16_t bestLag = minimumLag;
    float best = -1.0f;
    float zeroLag = 0.0f;

    if (maximumLag >= size)
    {
        maximumLag = (uint16_t)(size - 1U);
    }
    for (lag = 0U; lag < size; lag++)
    {
        float value = VitalSigns_getRingSample(waveform, size, writeIndex, lag);
        zeroLag += value * value;
    }
    for (lag = minimumLag; lag <= maximumLag; lag++)
    {
        uint16_t index;
        float correlation = 0.0f;
        for (index = 0U; index < (uint16_t)(size - lag); index++)
        {
            correlation += VitalSigns_getRingSample(waveform, size, writeIndex, index) *
                           VitalSigns_getRingSample(waveform, size, writeIndex,
                                                    (uint16_t)(index + lag));
        }
        if (correlation > best)
        {
            best = correlation;
            bestLag = lag;
        }
    }
    *confidence = zeroLag > 0.0f ? best / zeroLag : 0.0f;
    return bestLag != 0U ? 60.0f * VS_SAMPLE_RATE_HZ / (float)bestLag : 0.0f;
}

uint32_t VitalSigns_getStateSize(void)
{
    return (uint32_t)sizeof(VitalSigns_State);
}

void VitalSigns_init(VitalSigns_State *state, uint16_t numRangeBins,
                     float rangeStepMeters)
{
    uint16_t end;

    memset(state, 0, sizeof(*state));
    state->numRangeBins = numRangeBins > VS_MAX_RANGE_BINS ?
                          VS_MAX_RANGE_BINS : numRangeBins;
    state->rangeBinStart = (uint16_t)(VS_START_RANGE_M / rangeStepMeters + 0.5f);
    end = (uint16_t)(VS_END_RANGE_M / rangeStepMeters + 0.5f);
    state->rangeBinEnd = end < state->numRangeBins ? end :
                         (uint16_t)(state->numRangeBins - 1U);
    if (state->rangeBinStart > state->rangeBinEnd)
    {
        state->rangeBinStart = state->rangeBinEnd;
    }
    state->trackedRangeBin = state->rangeBinStart;
    state->output.rangeBinStartIndex = state->rangeBinStart;
    state->output.rangeBinEndIndex = state->rangeBinEnd;
}

const VitalSignsDemo_OutputStats *VitalSigns_process(
    VitalSigns_State *state, const cmplx16ImRe_t *radarCube)
{
    uint16_t rangeBin;
    uint16_t strongestBin = state->rangeBinStart;
    float strongestPower = 0.0f;
    float selectedReal = 0.0f;
    float selectedImag = 0.0f;
    float phase;
    float unwrapped;
    float phaseDifference;
    float breath;
    float heart;
    float heartPeakCount;
    float peakCountFiltered = 0.0f;
    float confidenceBreath;
    float confidenceHeart;
    float confidenceHeart4;
    float confidenceBreathXcorr;
    float confidenceHeartXcorr;
    uint16_t index;

    state->frameCount++;
    for (rangeBin = state->rangeBinStart; rangeBin <= state->rangeBinEnd; rangeBin++)
    {
        float real = (float)radarCube[rangeBin].real;
        float imag = (float)radarCube[rangeBin].imag;
        float deltaReal;
        float deltaImag;
        float power;

        state->rangeMeanReal[rangeBin] = 0.01f * real +
                                        0.99f * state->rangeMeanReal[rangeBin];
        state->rangeMeanImag[rangeBin] = 0.01f * imag +
                                        0.99f * state->rangeMeanImag[rangeBin];
        deltaReal = real - state->rangeMeanReal[rangeBin];
        deltaImag = imag - state->rangeMeanImag[rangeBin];
        power = deltaReal * deltaReal + deltaImag * deltaImag;
        if (power > strongestPower)
        {
            strongestPower = power;
            strongestBin = rangeBin;
        }
    }

    if (((state->frameCount - 1U) % VS_TRACKING_PERIOD) == 0U)
    {
        state->trackedRangeBin = strongestBin;
    }
    selectedReal = (float)radarCube[state->trackedRangeBin].real;
    selectedImag = (float)radarCube[state->trackedRangeBin].imag;
    phase = atan2sp(selectedImag, selectedReal);
    unwrapped = VitalSigns_unwrap(phase, state->phasePrevious,
                                  &state->phaseCorrection);
    state->phasePrevious = phase;
    phaseDifference = unwrapped - state->unwrappedPrevious;
    state->unwrappedPrevious = unwrapped;

    state->impulsePrevious2 = state->impulsePrevious1;
    state->impulsePrevious1 = state->impulseCurrent;
    state->impulseCurrent = phaseDifference;
    phaseDifference = VitalSigns_removeImpulse(state->impulsePrevious2,
                                               state->impulsePrevious1,
                                               state->impulseCurrent);

    breath = VitalSigns_iir(phaseDifference, gBreathCoefficients,
                            gBreathScale, state->breathDelay, 2U);
    heart = VitalSigns_iir(phaseDifference, gHeartCoefficients,
                           gHeartScale, state->heartDelay, 4U);

    state->breathBuffer[state->breathWriteIndex] = breath;
    state->breathWriteIndex = (uint16_t)((state->breathWriteIndex + 1U) %
                                         VS_BREATH_BUFFER_SIZE);
    state->heartBuffer[state->heartWriteIndex] = heart;
    state->heartWriteIndex = (uint16_t)((state->heartWriteIndex + 1U) %
                                        VS_HEART_BUFFER_SIZE);

    state->output.breathingRateEst_FFT = VitalSigns_spectrumEstimate(
        state, state->breathBuffer, VS_BREATH_BUFFER_SIZE,
        state->breathWriteIndex, 0.1f, 0.5f, 0U, &confidenceBreath);
    state->output.heartRateEst_FFT = VitalSigns_spectrumEstimate(
        state, state->heartBuffer, VS_HEART_BUFFER_SIZE,
        state->heartWriteIndex, 0.8f, 2.0f, 1U, &confidenceHeart);
    state->output.heartRateEst_FFT_4Hz = VitalSigns_spectrumEstimate(
        state, state->heartBuffer, VS_HEART_BUFFER_SIZE,
        state->heartWriteIndex, 1.6f, 4.0f, 1U, &confidenceHeart4);

    state->output.breathingRateEst_peakCount = VitalSigns_peakCount(
        state->breathBuffer, VS_BREATH_BUFFER_SIZE, state->breathWriteIndex);
    heartPeakCount = VitalSigns_peakCount(state->heartBuffer,
        VS_HEART_BUFFER_SIZE, state->heartWriteIndex);
    state->peakCountHistory[state->peakCountHistoryIndex] = heartPeakCount;
    state->peakCountHistoryIndex = (uint16_t)((state->peakCountHistoryIndex + 1U) % 10U);
    for (index = 0U; index < 10U; index++)
    {
        peakCountFiltered += state->peakCountHistory[index] * 0.1f;
    }

    state->output.breathingRateEst_xCorr = VitalSigns_autocorrelationRate(
        state->breathBuffer, VS_BREATH_BUFFER_SIZE, state->breathWriteIndex,
        0.1f, 0.5f, &confidenceBreathXcorr);
    state->output.heartRateEst_xCorr = VitalSigns_autocorrelationRate(
        state->heartBuffer, VS_HEART_BUFFER_SIZE, state->heartWriteIndex,
        0.8f, 2.0f, &confidenceHeartXcorr);

    state->breathEnergy = 0.01f * breath * breath + 0.99f * state->breathEnergy;
    state->heartEnergy = 0.01f * heart * heart + 0.99f * state->heartEnergy;

    state->output.rangeBinIndexMax = strongestBin;
    state->output.rangeBinIndexPhase = state->trackedRangeBin;
    state->output.maxVal = strongestPower;
    state->output.unwrapPhasePeak_mm = unwrapped;
    state->output.outputFilterBreathOut = breath;
    state->output.outputFilterHeartOut = heart;
    state->output.heartRateEst_peakCount_filtered = peakCountFiltered;
    state->output.confidenceMetricBreathOut = confidenceBreath;
    state->output.confidenceMetricBreathOut_xCorr = confidenceBreathXcorr;
    state->output.confidenceMetricHeartOut = confidenceHeart;
    state->output.confidenceMetricHeartOut_4Hz = confidenceHeart4;
    state->output.confidenceMetricHeartOut_xCorr = confidenceHeartXcorr;
    state->output.sumEnergyBreathWfm = state->breathEnergy * 10000.0f;
    state->output.sumEnergyHeartWfm = state->heartEnergy * 10000.0f;
    state->output.motionDetectedFlag = 0.0f;
    state->output.breathingRateEst_harmonicEnergy =
        state->output.breathingRateEst_FFT;
    state->output.heartRateEst_harmonicEnergy = state->output.heartRateEst_FFT;

    return &state->output;
}
