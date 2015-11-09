/* Include files */

#include "blascompat32.h"
#include "Quadrotor_sfun.h"
#include "c2_Quadrotor.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Quadrotor_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[19] = { "g", "Ts", "m", "K", "nargin",
  "nargout", "Z_meas", "theta_kalman", "phi_kalman", "U1", "Z_kalman", "Pp", "Q",
  "R", "xp", "A1", "H", "I", "xp_dot" };

/* Function Declarations */
static void initialize_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance);
static void initialize_params_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance);
static void enable_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance);
static void disable_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance);
static void set_sim_state_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance);
static void sf_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance);
static void c2_chartstep_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance);
static void initSimStructsc2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_xp_dot, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_I, const char_T *c2_identifier);
static real_T c2_d_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_e_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_H, const char_T *c2_identifier);
static real_T c2_f_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_g_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_A1, const char_T *c2_identifier);
static real_T c2_h_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_i_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_xp, const char_T *c2_identifier);
static real_T c2_j_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_k_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_R, const char_T *c2_identifier);
static real_T c2_l_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_m_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_Q, const char_T *c2_identifier);
static real_T c2_n_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_o_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_Pp, const char_T *c2_identifier);
static real_T c2_p_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_q_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_Z_kalman, const char_T *c2_identifier);
static real_T c2_r_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[34]);
static real_T c2_eye(SFc2_QuadrotorInstanceStruct *chartInstance);
static void c2_eml_warning(SFc2_QuadrotorInstanceStruct *chartInstance);
static void c2_b_eml_warning(SFc2_QuadrotorInstanceStruct *chartInstance, char_T
  c2_varargin_2[14]);
static void c2_s_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_sprintf, const char_T *c2_identifier, char_T c2_y[14]);
static void c2_t_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, char_T c2_y[14]);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_u_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_v_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_Quadrotor, const char_T *c2_identifier);
static uint8_T c2_w_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_QuadrotorInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_Pp_not_empty = FALSE;
  chartInstance->c2_Q_not_empty = FALSE;
  chartInstance->c2_R_not_empty = FALSE;
  chartInstance->c2_xp_not_empty = FALSE;
  chartInstance->c2_A1_not_empty = FALSE;
  chartInstance->c2_H_not_empty = FALSE;
  chartInstance->c2_I_not_empty = FALSE;
  chartInstance->c2_xp_dot_not_empty = FALSE;
  chartInstance->c2_is_active_c2_Quadrotor = 0U;
}

static void initialize_params_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance)
{
}

static void enable_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_e_hoistedGlobal;
  real_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_f_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_g_hoistedGlobal;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_h_hoistedGlobal;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_i_hoistedGlobal;
  real_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  uint8_T c2_j_hoistedGlobal;
  uint8_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  real_T *c2_Z_kalman;
  c2_Z_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(10), FALSE);
  c2_hoistedGlobal = *c2_Z_kalman;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = chartInstance->c2_A1;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  if (!chartInstance->c2_A1_not_empty) {
    sf_mex_assign(&c2_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = chartInstance->c2_H;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  if (!chartInstance->c2_H_not_empty) {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = chartInstance->c2_I;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  if (!chartInstance->c2_I_not_empty) {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_hoistedGlobal = chartInstance->c2_Pp;
  c2_e_u = c2_e_hoistedGlobal;
  c2_f_y = NULL;
  if (!chartInstance->c2_Pp_not_empty) {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_hoistedGlobal = chartInstance->c2_Q;
  c2_f_u = c2_f_hoistedGlobal;
  c2_g_y = NULL;
  if (!chartInstance->c2_Q_not_empty) {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_g_hoistedGlobal = chartInstance->c2_R;
  c2_g_u = c2_g_hoistedGlobal;
  c2_h_y = NULL;
  if (!chartInstance->c2_R_not_empty) {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_h_hoistedGlobal = chartInstance->c2_xp;
  c2_h_u = c2_h_hoistedGlobal;
  c2_i_y = NULL;
  if (!chartInstance->c2_xp_not_empty) {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_i_hoistedGlobal = chartInstance->c2_xp_dot;
  c2_i_u = c2_i_hoistedGlobal;
  c2_j_y = NULL;
  if (!chartInstance->c2_xp_dot_not_empty) {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 8, c2_j_y);
  c2_j_hoistedGlobal = chartInstance->c2_is_active_c2_Quadrotor;
  c2_j_u = c2_j_hoistedGlobal;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 9, c2_k_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_Z_kalman;
  c2_Z_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_Z_kalman = c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 0)), "Z_kalman");
  chartInstance->c2_A1 = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 1)), "A1");
  chartInstance->c2_H = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 2)), "H");
  chartInstance->c2_I = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 3)), "I");
  chartInstance->c2_Pp = c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 4)), "Pp");
  chartInstance->c2_Q = c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 5)), "Q");
  chartInstance->c2_R = c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 6)), "R");
  chartInstance->c2_xp = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 7)), "xp");
  chartInstance->c2_xp_dot = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 8)), "xp_dot");
  chartInstance->c2_is_active_c2_Quadrotor = c2_v_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 9)), "is_active_c2_Quadrotor");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_Quadrotor(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance)
{
}

static void sf_c2_Quadrotor(SFc2_QuadrotorInstanceStruct *chartInstance)
{
  real_T *c2_Z_meas;
  real_T *c2_Z_kalman;
  real_T *c2_theta_kalman;
  real_T *c2_phi_kalman;
  real_T *c2_U1;
  c2_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_Z_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_Z_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_Z_meas, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_Z_kalman, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_theta_kalman, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_phi_kalman, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_U1, 4U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_Quadrotor(chartInstance);
  sf_debug_check_for_state_inconsistency(_QuadrotorMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_Z_meas;
  real_T c2_theta_kalman;
  real_T c2_phi_kalman;
  real_T c2_U1;
  uint32_T c2_debug_family_var_map[19];
  real_T c2_g;
  real_T c2_Ts;
  real_T c2_m;
  real_T c2_K;
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 1.0;
  real_T c2_Z_kalman;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_A;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_c_y;
  real_T c2_c_a;
  real_T c2_d_y;
  real_T c2_e_hoistedGlobal;
  real_T c2_d_a;
  real_T c2_e_y;
  real_T c2_f_hoistedGlobal;
  real_T c2_g_hoistedGlobal;
  real_T c2_e_a;
  real_T c2_c_b;
  real_T c2_f_y;
  real_T c2_h_hoistedGlobal;
  real_T c2_f_a;
  real_T c2_d_b;
  real_T c2_g_y;
  real_T c2_i_hoistedGlobal;
  real_T c2_j_hoistedGlobal;
  real_T c2_g_a;
  real_T c2_e_b;
  real_T c2_h_y;
  real_T c2_k_hoistedGlobal;
  real_T c2_l_hoistedGlobal;
  real_T c2_h_a;
  real_T c2_f_b;
  real_T c2_i_y;
  real_T c2_m_hoistedGlobal;
  real_T c2_i_a;
  real_T c2_g_b;
  real_T c2_j_y;
  real_T c2_n_hoistedGlobal;
  real_T c2_g_x;
  real_T c2_k_y;
  real_T c2_l_y;
  real_T c2_h_x;
  real_T c2_xinv;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_n1x;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_n1xinv;
  real_T c2_j_a;
  real_T c2_h_b;
  real_T c2_m_y;
  real_T c2_rc;
  real_T c2_o_x;
  boolean_T c2_i_b;
  real_T c2_p_x;
  int32_T c2_i0;
  static char_T c2_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c2_u[8];
  const mxArray *c2_n_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_o_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_p_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_q_y = NULL;
  char_T c2_str[14];
  int32_T c2_i1;
  char_T c2_b_str[14];
  real_T c2_k_a;
  real_T c2_j_b;
  real_T c2_o_hoistedGlobal;
  real_T c2_p_hoistedGlobal;
  real_T c2_q_hoistedGlobal;
  real_T c2_l_a;
  real_T c2_k_b;
  real_T c2_r_y;
  real_T c2_m_a;
  real_T c2_l_b;
  real_T c2_s_y;
  real_T c2_r_hoistedGlobal;
  real_T c2_s_hoistedGlobal;
  real_T c2_n_a;
  real_T c2_m_b;
  real_T c2_t_y;
  real_T c2_t_hoistedGlobal;
  real_T c2_o_a;
  real_T c2_n_b;
  real_T *c2_b_Z_kalman;
  real_T *c2_b_Z_meas;
  real_T *c2_b_theta_kalman;
  real_T *c2_b_phi_kalman;
  real_T *c2_b_U1;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  c2_b_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_Z_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_Z_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_Z_meas;
  c2_b_hoistedGlobal = *c2_b_theta_kalman;
  c2_c_hoistedGlobal = *c2_b_phi_kalman;
  c2_d_hoistedGlobal = *c2_b_U1;
  c2_Z_meas = c2_hoistedGlobal;
  c2_theta_kalman = c2_b_hoistedGlobal;
  c2_phi_kalman = c2_c_hoistedGlobal;
  c2_U1 = c2_d_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 19U, 19U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c2_g, 0U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_Ts, 1U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_m, 2U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_K, 3U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 4U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 5U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_Z_meas, 6U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_theta_kalman, 7U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_phi_kalman, 8U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c2_U1, 9U, c2_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c2_Z_kalman, 10U,
    c2_i_sf_marshallOut, c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_Pp, 11U,
    c2_h_sf_marshallOut, c2_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_Q, 12U,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_R, 13U,
    c2_f_sf_marshallOut, c2_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_xp, 14U,
    c2_e_sf_marshallOut, c2_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_A1, 15U,
    c2_d_sf_marshallOut, c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_H, 16U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_I, 17U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_xp_dot, 18U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_Ts = 0.01;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_m = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c2_Pp_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
    chartInstance->c2_Pp = 1.0;
    chartInstance->c2_Pp_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
    chartInstance->c2_Q = 5.0;
    chartInstance->c2_Q_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
    chartInstance->c2_R = 100.0;
    chartInstance->c2_R_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    chartInstance->c2_xp = 0.0;
    chartInstance->c2_xp_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    chartInstance->c2_A1 = 0.5;
    chartInstance->c2_A1_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    chartInstance->c2_H = 1.0;
    chartInstance->c2_H_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    chartInstance->c2_I = c2_eye(chartInstance);
    chartInstance->c2_I_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    chartInstance->c2_xp_dot = 0.0;
    chartInstance->c2_xp_dot_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_x = c2_theta_kalman;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_c_x = c2_phi_kalman;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarCos(c2_d_x);
  c2_a = c2_b_x;
  c2_b = c2_d_x;
  c2_y = c2_a * c2_b;
  c2_b_a = c2_y;
  c2_b_b = c2_U1;
  c2_b_y = c2_b_a * c2_b_b;
  c2_A = c2_b_y;
  c2_e_x = c2_A;
  c2_f_x = c2_e_x;
  c2_c_y = c2_f_x;
  c2_c_a = -c2_g + c2_c_y;
  c2_d_y = c2_c_a * 0.01;
  chartInstance->c2_xp_dot += c2_d_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_e_hoistedGlobal = chartInstance->c2_xp_dot;
  c2_d_a = c2_e_hoistedGlobal;
  c2_e_y = c2_d_a * 0.01;
  chartInstance->c2_xp += c2_e_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_f_hoistedGlobal = chartInstance->c2_A1;
  c2_g_hoistedGlobal = chartInstance->c2_Pp;
  c2_e_a = c2_f_hoistedGlobal;
  c2_c_b = c2_g_hoistedGlobal;
  c2_f_y = c2_e_a * c2_c_b;
  c2_h_hoistedGlobal = chartInstance->c2_A1;
  c2_f_a = c2_f_y;
  c2_d_b = c2_h_hoistedGlobal;
  c2_g_y = c2_f_a * c2_d_b;
  chartInstance->c2_Pp = c2_g_y + chartInstance->c2_Q;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  c2_i_hoistedGlobal = chartInstance->c2_Pp;
  c2_j_hoistedGlobal = chartInstance->c2_H;
  c2_g_a = c2_i_hoistedGlobal;
  c2_e_b = c2_j_hoistedGlobal;
  c2_h_y = c2_g_a * c2_e_b;
  c2_k_hoistedGlobal = chartInstance->c2_H;
  c2_l_hoistedGlobal = chartInstance->c2_Pp;
  c2_h_a = c2_k_hoistedGlobal;
  c2_f_b = c2_l_hoistedGlobal;
  c2_i_y = c2_h_a * c2_f_b;
  c2_m_hoistedGlobal = chartInstance->c2_H;
  c2_i_a = c2_i_y;
  c2_g_b = c2_m_hoistedGlobal;
  c2_j_y = c2_i_a * c2_g_b;
  c2_n_hoistedGlobal = chartInstance->c2_R;
  c2_g_x = c2_j_y + c2_n_hoistedGlobal;
  c2_k_y = c2_g_x;
  c2_l_y = 1.0 / c2_k_y;
  c2_h_x = c2_g_x;
  c2_xinv = c2_l_y;
  c2_i_x = c2_h_x;
  c2_j_x = c2_i_x;
  c2_k_x = c2_j_x;
  c2_n1x = muDoubleScalarAbs(c2_k_x);
  c2_l_x = c2_xinv;
  c2_m_x = c2_l_x;
  c2_n_x = c2_m_x;
  c2_n1xinv = muDoubleScalarAbs(c2_n_x);
  c2_j_a = c2_n1x;
  c2_h_b = c2_n1xinv;
  c2_m_y = c2_j_a * c2_h_b;
  c2_rc = 1.0 / c2_m_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c2_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c2_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c2_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c2_o_x = c2_rc;
    c2_i_b = muDoubleScalarIsNaN(c2_o_x);
    guard3 = FALSE;
    if (c2_i_b) {
      guard3 = TRUE;
    } else {
      if (c2_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c2_p_x = c2_rc;
      for (c2_i0 = 0; c2_i0 < 8; c2_i0++) {
        c2_u[c2_i0] = c2_cv0[c2_i0];
      }

      c2_n_y = NULL;
      sf_mex_assign(&c2_n_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c2_b_u = 14.0;
      c2_o_y = NULL;
      sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_c_u = 6.0;
      c2_p_y = NULL;
      sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_d_u = c2_p_x;
      c2_q_y = NULL;
      sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_s_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c2_n_y, 14, c2_o_y, 14,
        c2_p_y), 14, c2_q_y), "sprintf", c2_str);
      for (c2_i1 = 0; c2_i1 < 14; c2_i1++) {
        c2_b_str[c2_i1] = c2_str[c2_i1];
      }

      c2_b_eml_warning(chartInstance, c2_b_str);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c2_eml_warning(chartInstance);
  }

  c2_k_a = c2_h_y;
  c2_j_b = c2_l_y;
  c2_K = c2_k_a * c2_j_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  c2_o_hoistedGlobal = chartInstance->c2_xp;
  c2_p_hoistedGlobal = chartInstance->c2_H;
  c2_q_hoistedGlobal = chartInstance->c2_xp;
  c2_l_a = c2_p_hoistedGlobal;
  c2_k_b = c2_q_hoistedGlobal;
  c2_r_y = c2_l_a * c2_k_b;
  c2_m_a = c2_K;
  c2_l_b = c2_Z_meas - c2_r_y;
  c2_s_y = c2_m_a * c2_l_b;
  c2_Z_kalman = c2_o_hoistedGlobal + c2_s_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  c2_r_hoistedGlobal = chartInstance->c2_I;
  c2_s_hoistedGlobal = chartInstance->c2_H;
  c2_n_a = c2_K;
  c2_m_b = c2_s_hoistedGlobal;
  c2_t_y = c2_n_a * c2_m_b;
  c2_t_hoistedGlobal = chartInstance->c2_Pp;
  c2_o_a = c2_r_hoistedGlobal - c2_t_y;
  c2_n_b = c2_t_hoistedGlobal;
  chartInstance->c2_Pp = c2_o_a * c2_n_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  chartInstance->c2_xp = c2_Z_kalman;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -23);
  sf_debug_symbol_scope_pop();
  *c2_b_Z_kalman = c2_Z_kalman;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_Quadrotor(SFc2_QuadrotorInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_xp_dot_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_xp_dot, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_xp_dot),
    &c2_thisId);
  sf_mex_destroy(&c2_b_xp_dot);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_xp_dot_not_empty = FALSE;
  } else {
    chartInstance->c2_xp_dot_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d0;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_xp_dot;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_xp_dot = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_xp_dot),
    &c2_thisId);
  sf_mex_destroy(&c2_b_xp_dot);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_I_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_I, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_I), &c2_thisId);
  sf_mex_destroy(&c2_b_I);
  return c2_y;
}

static real_T c2_d_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d1;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_I_not_empty = FALSE;
  } else {
    chartInstance->c2_I_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d1, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d1;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_I;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_I = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_I), &c2_thisId);
  sf_mex_destroy(&c2_b_I);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_H_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_e_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_H, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_H), &c2_thisId);
  sf_mex_destroy(&c2_b_H);
  return c2_y;
}

static real_T c2_f_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d2;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_H_not_empty = FALSE;
  } else {
    chartInstance->c2_H_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d2, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d2;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_H;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_H = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_H), &c2_thisId);
  sf_mex_destroy(&c2_b_H);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_A1_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_g_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_A1, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_A1), &c2_thisId);
  sf_mex_destroy(&c2_b_A1);
  return c2_y;
}

static real_T c2_h_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d3;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_A1_not_empty = FALSE;
  } else {
    chartInstance->c2_A1_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d3, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d3;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_A1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_A1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_A1), &c2_thisId);
  sf_mex_destroy(&c2_b_A1);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_xp_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_i_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_xp, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_xp), &c2_thisId);
  sf_mex_destroy(&c2_b_xp);
  return c2_y;
}

static real_T c2_j_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d4;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_xp_not_empty = FALSE;
  } else {
    chartInstance->c2_xp_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d4, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d4;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_xp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_xp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_xp), &c2_thisId);
  sf_mex_destroy(&c2_b_xp);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_R_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_k_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_R, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R), &c2_thisId);
  sf_mex_destroy(&c2_b_R);
  return c2_y;
}

static real_T c2_l_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d5;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_R_not_empty = FALSE;
  } else {
    chartInstance->c2_R_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d5, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d5;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_R;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_R = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R), &c2_thisId);
  sf_mex_destroy(&c2_b_R);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Q_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_m_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_Q, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Q), &c2_thisId);
  sf_mex_destroy(&c2_b_Q);
  return c2_y;
}

static real_T c2_n_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d6;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Q_not_empty = FALSE;
  } else {
    chartInstance->c2_Q_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d6, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d6;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Q;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_Q = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Q), &c2_thisId);
  sf_mex_destroy(&c2_b_Q);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Pp_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_o_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_Pp, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Pp), &c2_thisId);
  sf_mex_destroy(&c2_b_Pp);
  return c2_y;
}

static real_T c2_p_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d7;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Pp_not_empty = FALSE;
  } else {
    chartInstance->c2_Pp_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d7, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d7;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Pp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_Pp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Pp), &c2_thisId);
  sf_mex_destroy(&c2_b_Pp);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_q_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_Z_kalman, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Z_kalman),
    &c2_thisId);
  sf_mex_destroy(&c2_Z_kalman);
  return c2_y;
}

static real_T c2_r_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d8;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d8, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d8;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Z_kalman;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_Z_kalman = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Z_kalman),
    &c2_thisId);
  sf_mex_destroy(&c2_Z_kalman);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_Quadrotor_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[34];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i2;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 34), FALSE);
  for (c2_i2 = 0; c2_i2 < 34; c2_i2++) {
    c2_r0 = &c2_info[c2_i2];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i2);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i2);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[34])
{
  c2_info[0].context = "";
  c2_info[0].name = "eye";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c2_info[0].fileTimeLo = 1286811488U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[1].name = "eml_assert_valid_size_arg";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[1].fileTimeLo = 1286811494U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c2_info[2].name = "isinf";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c2_info[2].fileTimeLo = 1286811560U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c2_info[3].name = "mtimes";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[3].fileTimeLo = 1289508892U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[4].name = "eml_index_class";
  c2_info[4].dominantType = "";
  c2_info[4].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[4].fileTimeLo = 1286811578U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[5].name = "intmax";
  c2_info[5].dominantType = "char";
  c2_info[5].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[5].fileTimeLo = 1311248116U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[6].name = "eml_is_float_class";
  c2_info[6].dominantType = "char";
  c2_info[6].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[6].fileTimeLo = 1286811582U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[7].name = "min";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[7].fileTimeLo = 1311248118U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[8].name = "eml_min_or_max";
  c2_info[8].dominantType = "char";
  c2_info[8].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[8].fileTimeLo = 1303139012U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[9].name = "eml_scalar_eg";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[9].fileTimeLo = 1286811596U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[10].name = "eml_scalexp_alloc";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[10].fileTimeLo = 1286811596U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[11].name = "eml_index_class";
  c2_info[11].dominantType = "";
  c2_info[11].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[11].fileTimeLo = 1286811578U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[12].name = "eml_scalar_eg";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[12].fileTimeLo = 1286811596U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[13].name = "eml_index_class";
  c2_info[13].dominantType = "";
  c2_info[13].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[13].fileTimeLo = 1286811578U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context = "";
  c2_info[14].name = "cos";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[14].fileTimeLo = 1286811506U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[15].name = "eml_scalar_cos";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[15].fileTimeLo = 1286811522U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context = "";
  c2_info[16].name = "mtimes";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[16].fileTimeLo = 1289508892U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context = "";
  c2_info[17].name = "mrdivide";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[17].fileTimeLo = 1325113338U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 1319722766U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[18].name = "rdivide";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[18].fileTimeLo = 1286811644U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[19].name = "eml_div";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[19].fileTimeLo = 1313340610U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context = "";
  c2_info[20].name = "inv";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c2_info[20].fileTimeLo = 1305310800U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c2_info[21].name = "eml_div";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[21].fileTimeLo = 1313340610U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[22].name = "norm";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c2_info[22].fileTimeLo = 1286811626U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c2_info[23].name = "abs";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[23].fileTimeLo = 1286811494U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[24].name = "eml_scalar_abs";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[24].fileTimeLo = 1286811512U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[25].name = "mtimes";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[25].fileTimeLo = 1289508892U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[26].name = "eml_warning";
  c2_info[26].dominantType = "char";
  c2_info[26].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[26].fileTimeLo = 1286811602U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[27].name = "isnan";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[27].fileTimeLo = 1286811560U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[28].name = "eps";
  c2_info[28].dominantType = "char";
  c2_info[28].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[28].fileTimeLo = 1307644040U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[29].name = "eml_is_float_class";
  c2_info[29].dominantType = "char";
  c2_info[29].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[29].fileTimeLo = 1286811582U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[30].name = "eml_eps";
  c2_info[30].dominantType = "char";
  c2_info[30].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[30].fileTimeLo = 1307644040U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[31].name = "eml_float_model";
  c2_info[31].dominantType = "char";
  c2_info[31].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[31].fileTimeLo = 1307644042U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c2_info[32].name = "eml_flt2str";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[32].fileTimeLo = 1309443996U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[33].name = "char";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/strfun/char.m";
  c2_info[33].fileTimeLo = 1319722768U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
}

static real_T c2_eye(SFc2_QuadrotorInstanceStruct *chartInstance)
{
  return 1.0;
}

static void c2_eml_warning(SFc2_QuadrotorInstanceStruct *chartInstance)
{
  int32_T c2_i3;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i3 = 0; c2_i3 < 27; c2_i3++) {
    c2_u[c2_i3] = c2_varargin_1[c2_i3];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static void c2_b_eml_warning(SFc2_QuadrotorInstanceStruct *chartInstance, char_T
  c2_varargin_2[14])
{
  int32_T c2_i4;
  static char_T c2_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c2_u[33];
  const mxArray *c2_y = NULL;
  int32_T c2_i5;
  char_T c2_b_u[14];
  const mxArray *c2_b_y = NULL;
  for (c2_i4 = 0; c2_i4 < 33; c2_i4++) {
    c2_u[c2_i4] = c2_varargin_1[c2_i4];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c2_i5 = 0; c2_i5 < 14; c2_i5++) {
    c2_b_u[c2_i5] = c2_varargin_2[c2_i5];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c2_y, 14, c2_b_y));
}

static void c2_s_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_sprintf, const char_T *c2_identifier, char_T c2_y[14])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_sprintf), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_sprintf);
}

static void c2_t_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, char_T c2_y[14])
{
  char_T c2_cv1[14];
  int32_T c2_i6;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c2_i6 = 0; c2_i6 < 14; c2_i6++) {
    c2_y[c2_i6] = c2_cv1[c2_i6];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_u_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i7;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i7, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i7;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_v_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_Quadrotor, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_w_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_Quadrotor), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Quadrotor);
  return c2_y;
}

static uint8_T c2_w_emlrt_marshallIn(SFc2_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_QuadrotorInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_Quadrotor_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1505731793U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3221568000U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3795764620U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3468402359U);
}

mxArray *sf_c2_Quadrotor_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("CS6CFBeQPsPD5UrZJUZLqG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c2_Quadrotor(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"Z_kalman\",},{M[4],M[0],T\"A1\",S'l','i','p'{{M1x2[136 138],M[0],}}},{M[4],M[0],T\"H\",S'l','i','p'{{M1x2[139 140],M[0],}}},{M[4],M[0],T\"I\",S'l','i','p'{{M1x2[141 142],M[0],}}},{M[4],M[0],T\"Pp\",S'l','i','p'{{M1x2[126 128],M[0],}}},{M[4],M[0],T\"Q\",S'l','i','p'{{M1x2[129 130],M[0],}}},{M[4],M[0],T\"R\",S'l','i','p'{{M1x2[131 132],M[0],}}},{M[4],M[0],T\"xp\",S'l','i','p'{{M1x2[133 135],M[0],}}},{M[4],M[0],T\"xp_dot\",S'l','i','p'{{M1x2[143 149],M[0],}}},{M[8],M[0],T\"is_active_c2_Quadrotor\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 10, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Quadrotor_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_QuadrotorInstanceStruct *chartInstance;
    chartInstance = (SFc2_QuadrotorInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_QuadrotorMachineNumber_,
          2,
          1,
          1,
          5,
          0,
          0,
          0,
          0,
          0,
          &(chartInstance->chartNumber),
          &(chartInstance->instanceNumber),
          ssGetPath(S),
          (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_QuadrotorMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_QuadrotorMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_QuadrotorMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Z_meas");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Z_kalman");
          _SFD_SET_DATA_PROPS(2,1,1,0,"theta_kalman");
          _SFD_SET_DATA_PROPS(3,1,1,0,"phi_kalman");
          _SFD_SET_DATA_PROPS(4,1,1,0,"U1");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,567);
        _SFD_CV_INIT_EML_IF(0,1,0,150,164,-1,263);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)c2_i_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c2_Z_meas;
          real_T *c2_Z_kalman;
          real_T *c2_theta_kalman;
          real_T *c2_phi_kalman;
          real_T *c2_U1;
          c2_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_Z_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_Z_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_Z_meas);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_Z_kalman);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_theta_kalman);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_phi_kalman);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_U1);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_QuadrotorMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "9mJDJgrA6frDphDvY5VxDD";
}

static void sf_opaque_initialize_c2_Quadrotor(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_QuadrotorInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*)
    chartInstanceVar);
  initialize_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_Quadrotor(void *chartInstanceVar)
{
  enable_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_Quadrotor(void *chartInstanceVar)
{
  disable_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_Quadrotor(void *chartInstanceVar)
{
  sf_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_Quadrotor(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_Quadrotor();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_Quadrotor(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_Quadrotor();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_Quadrotor(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_Quadrotor(S);
}

static void sf_opaque_set_sim_state_c2_Quadrotor(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_Quadrotor(S, st);
}

static void sf_opaque_terminate_c2_Quadrotor(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_QuadrotorInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Quadrotor_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Quadrotor((SFc2_QuadrotorInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Quadrotor(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_Quadrotor((SFc2_QuadrotorInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_Quadrotor(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Quadrotor_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3573946881U));
  ssSetChecksum1(S,(2549441258U));
  ssSetChecksum2(S,(2934885173U));
  ssSetChecksum3(S,(1808447325U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_Quadrotor(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Quadrotor(SimStruct *S)
{
  SFc2_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuadrotorInstanceStruct *)malloc(sizeof
    (SFc2_QuadrotorInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_QuadrotorInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_Quadrotor;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_Quadrotor;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_Quadrotor;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_Quadrotor;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_Quadrotor;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_Quadrotor;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_Quadrotor;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_Quadrotor;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_Quadrotor;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_Quadrotor;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_Quadrotor;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_Quadrotor_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Quadrotor(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Quadrotor(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Quadrotor(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Quadrotor_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
