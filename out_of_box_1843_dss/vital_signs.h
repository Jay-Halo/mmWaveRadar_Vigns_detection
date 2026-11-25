#ifndef AWR1843_VITAL_SIGNS_H
#define AWR1843_VITAL_SIGNS_H

#include <stdint.h>
#include <ti/common/sys_common.h>
#include "../out_of_box_1843_vital_signs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VitalSigns_State_t VitalSigns_State;

uint32_t VitalSigns_getStateSize(void);

void VitalSigns_init(VitalSigns_State *state,
                     uint16_t numRangeBins,
                     float rangeStepMeters);

const VitalSignsDemo_OutputStats *VitalSigns_process(
                     VitalSigns_State *state,
                     const cmplx16ImRe_t *radarCube);

#ifdef __cplusplus
}
#endif

#endif /* AWR1843_VITAL_SIGNS_H */
