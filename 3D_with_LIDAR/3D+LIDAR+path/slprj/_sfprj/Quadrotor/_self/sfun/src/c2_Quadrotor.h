#ifndef __c2_Quadrotor_h__
#define __c2_Quadrotor_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c2_ResolvedFunctionInfo;

typedef struct {
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_Quadrotor;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  real_T c2_Pp;
  boolean_T c2_Pp_not_empty;
  real_T c2_Q;
  boolean_T c2_Q_not_empty;
  real_T c2_R;
  boolean_T c2_R_not_empty;
  real_T c2_xp;
  boolean_T c2_xp_not_empty;
  real_T c2_A1;
  boolean_T c2_A1_not_empty;
  real_T c2_H;
  boolean_T c2_H_not_empty;
  real_T c2_I;
  boolean_T c2_I_not_empty;
  real_T c2_xp_dot;
  boolean_T c2_xp_dot_not_empty;
} SFc2_QuadrotorInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_Quadrotor_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_Quadrotor_get_check_sum(mxArray *plhs[]);
extern void c2_Quadrotor_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
