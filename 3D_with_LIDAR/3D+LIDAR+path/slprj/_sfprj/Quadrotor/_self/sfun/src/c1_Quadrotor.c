/* Include files */

#include "blascompat32.h"
#include "Quadrotor_sfun.h"
#include "c1_Quadrotor.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Quadrotor_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[21] = { "g", "Ts", "m", "K", "nargin",
  "nargout", "X_meas", "theta_kalman", "phi_kalman", "psi_kalman", "U1", "X_dot",
  "X_kalman", "Pp", "Q", "R", "xp", "A1", "H", "I", "xp_dot" };

/* Function Declarations */
static void initialize_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance);
static void initialize_params_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance);
static void enable_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance);
static void disable_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance);
static void set_sim_state_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance);
static void sf_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance);
static void c1_chartstep_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance);
static void initSimStructsc1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_xp_dot, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_I, const char_T *c1_identifier);
static real_T c1_d_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_e_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_H, const char_T *c1_identifier);
static real_T c1_f_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_g_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_A1, const char_T *c1_identifier);
static real_T c1_h_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_i_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_xp, const char_T *c1_identifier);
static real_T c1_j_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_k_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_R, const char_T *c1_identifier);
static real_T c1_l_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_m_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_Q, const char_T *c1_identifier);
static real_T c1_n_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_o_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_Pp, const char_T *c1_identifier);
static real_T c1_p_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_q_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_X_kalman, const char_T *c1_identifier);
static real_T c1_r_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[36]);
static real_T c1_eye(SFc1_QuadrotorInstanceStruct *chartInstance);
static void c1_eml_warning(SFc1_QuadrotorInstanceStruct *chartInstance);
static void c1_b_eml_warning(SFc1_QuadrotorInstanceStruct *chartInstance, char_T
  c1_varargin_2[14]);
static void c1_s_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_sprintf, const char_T *c1_identifier, char_T c1_y[14]);
static void c1_t_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, char_T c1_y[14]);
static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_u_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_v_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_Quadrotor, const char_T *c1_identifier);
static uint8_T c1_w_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_QuadrotorInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_Pp_not_empty = FALSE;
  chartInstance->c1_Q_not_empty = FALSE;
  chartInstance->c1_R_not_empty = FALSE;
  chartInstance->c1_xp_not_empty = FALSE;
  chartInstance->c1_A1_not_empty = FALSE;
  chartInstance->c1_H_not_empty = FALSE;
  chartInstance->c1_I_not_empty = FALSE;
  chartInstance->c1_xp_dot_not_empty = FALSE;
  chartInstance->c1_is_active_c1_Quadrotor = 0U;
}

static void initialize_params_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance)
{
}

static void enable_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  real_T c1_g_hoistedGlobal;
  real_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_h_hoistedGlobal;
  real_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  real_T c1_i_hoistedGlobal;
  real_T c1_i_u;
  const mxArray *c1_j_y = NULL;
  uint8_T c1_j_hoistedGlobal;
  uint8_T c1_j_u;
  const mxArray *c1_k_y = NULL;
  real_T *c1_X_kalman;
  c1_X_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(10), FALSE);
  c1_hoistedGlobal = *c1_X_kalman;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = chartInstance->c1_A1;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  if (!chartInstance->c1_A1_not_empty) {
    sf_mex_assign(&c1_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = chartInstance->c1_H;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  if (!chartInstance->c1_H_not_empty) {
    sf_mex_assign(&c1_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = chartInstance->c1_I;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  if (!chartInstance->c1_I_not_empty) {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = chartInstance->c1_Pp;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  if (!chartInstance->c1_Pp_not_empty) {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = chartInstance->c1_Q;
  c1_f_u = c1_f_hoistedGlobal;
  c1_g_y = NULL;
  if (!chartInstance->c1_Q_not_empty) {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = chartInstance->c1_R;
  c1_g_u = c1_g_hoistedGlobal;
  c1_h_y = NULL;
  if (!chartInstance->c1_R_not_empty) {
    sf_mex_assign(&c1_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_h_hoistedGlobal = chartInstance->c1_xp;
  c1_h_u = c1_h_hoistedGlobal;
  c1_i_y = NULL;
  if (!chartInstance->c1_xp_not_empty) {
    sf_mex_assign(&c1_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_i_hoistedGlobal = chartInstance->c1_xp_dot;
  c1_i_u = c1_i_hoistedGlobal;
  c1_j_y = NULL;
  if (!chartInstance->c1_xp_dot_not_empty) {
    sf_mex_assign(&c1_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_j_hoistedGlobal = chartInstance->c1_is_active_c1_Quadrotor;
  c1_j_u = c1_j_hoistedGlobal;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 9, c1_k_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_X_kalman;
  c1_X_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  *c1_X_kalman = c1_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 0)), "X_kalman");
  chartInstance->c1_A1 = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 1)), "A1");
  chartInstance->c1_H = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 2)), "H");
  chartInstance->c1_I = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 3)), "I");
  chartInstance->c1_Pp = c1_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 4)), "Pp");
  chartInstance->c1_Q = c1_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 5)), "Q");
  chartInstance->c1_R = c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 6)), "R");
  chartInstance->c1_xp = c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 7)), "xp");
  chartInstance->c1_xp_dot = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 8)), "xp_dot");
  chartInstance->c1_is_active_c1_Quadrotor = c1_v_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 9)), "is_active_c1_Quadrotor");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_Quadrotor(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance)
{
}

static void sf_c1_Quadrotor(SFc1_QuadrotorInstanceStruct *chartInstance)
{
  real_T *c1_X_meas;
  real_T *c1_X_kalman;
  real_T *c1_theta_kalman;
  real_T *c1_phi_kalman;
  real_T *c1_psi_kalman;
  real_T *c1_U1;
  real_T *c1_X_dot;
  c1_X_dot = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c1_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_psi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_X_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_X_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c1_X_meas, 0U);
  _SFD_DATA_RANGE_CHECK(*c1_X_kalman, 1U);
  _SFD_DATA_RANGE_CHECK(*c1_theta_kalman, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_phi_kalman, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_psi_kalman, 4U);
  _SFD_DATA_RANGE_CHECK(*c1_U1, 5U);
  _SFD_DATA_RANGE_CHECK(*c1_X_dot, 6U);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_Quadrotor(chartInstance);
  sf_debug_check_for_state_inconsistency(_QuadrotorMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c1_chartstep_c1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_e_hoistedGlobal;
  real_T c1_f_hoistedGlobal;
  real_T c1_X_meas;
  real_T c1_theta_kalman;
  real_T c1_phi_kalman;
  real_T c1_psi_kalman;
  real_T c1_U1;
  real_T c1_X_dot;
  uint32_T c1_debug_family_var_map[21];
  real_T c1_g;
  real_T c1_Ts;
  real_T c1_m;
  real_T c1_K;
  real_T c1_nargin = 6.0;
  real_T c1_nargout = 1.0;
  real_T c1_X_kalman;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T c1_d_x;
  real_T c1_a;
  real_T c1_b;
  real_T c1_y;
  real_T c1_e_x;
  real_T c1_f_x;
  real_T c1_g_x;
  real_T c1_h_x;
  real_T c1_b_a;
  real_T c1_b_b;
  real_T c1_b_y;
  real_T c1_i_x;
  real_T c1_j_x;
  real_T c1_c_a;
  real_T c1_c_b;
  real_T c1_c_y;
  real_T c1_d_a;
  real_T c1_d_b;
  real_T c1_d_y;
  real_T c1_A;
  real_T c1_k_x;
  real_T c1_l_x;
  real_T c1_e_y;
  real_T c1_e_a;
  real_T c1_f_y;
  real_T c1_f_a;
  real_T c1_g_y;
  real_T c1_g_hoistedGlobal;
  real_T c1_g_a;
  real_T c1_h_y;
  real_T c1_h_hoistedGlobal;
  real_T c1_i_hoistedGlobal;
  real_T c1_h_a;
  real_T c1_e_b;
  real_T c1_i_y;
  real_T c1_j_hoistedGlobal;
  real_T c1_i_a;
  real_T c1_f_b;
  real_T c1_j_y;
  real_T c1_k_hoistedGlobal;
  real_T c1_l_hoistedGlobal;
  real_T c1_j_a;
  real_T c1_g_b;
  real_T c1_k_y;
  real_T c1_m_hoistedGlobal;
  real_T c1_n_hoistedGlobal;
  real_T c1_k_a;
  real_T c1_h_b;
  real_T c1_l_y;
  real_T c1_o_hoistedGlobal;
  real_T c1_l_a;
  real_T c1_i_b;
  real_T c1_m_y;
  real_T c1_p_hoistedGlobal;
  real_T c1_m_x;
  real_T c1_n_y;
  real_T c1_o_y;
  real_T c1_n_x;
  real_T c1_xinv;
  real_T c1_o_x;
  real_T c1_p_x;
  real_T c1_q_x;
  real_T c1_n1x;
  real_T c1_r_x;
  real_T c1_s_x;
  real_T c1_t_x;
  real_T c1_n1xinv;
  real_T c1_m_a;
  real_T c1_j_b;
  real_T c1_p_y;
  real_T c1_rc;
  real_T c1_u_x;
  boolean_T c1_k_b;
  real_T c1_v_x;
  int32_T c1_i0;
  static char_T c1_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c1_u[8];
  const mxArray *c1_q_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_r_y = NULL;
  real_T c1_c_u;
  const mxArray *c1_s_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_t_y = NULL;
  char_T c1_str[14];
  int32_T c1_i1;
  char_T c1_b_str[14];
  real_T c1_n_a;
  real_T c1_l_b;
  real_T c1_q_hoistedGlobal;
  real_T c1_r_hoistedGlobal;
  real_T c1_s_hoistedGlobal;
  real_T c1_o_a;
  real_T c1_m_b;
  real_T c1_u_y;
  real_T c1_p_a;
  real_T c1_n_b;
  real_T c1_v_y;
  real_T c1_t_hoistedGlobal;
  real_T c1_u_hoistedGlobal;
  real_T c1_q_a;
  real_T c1_o_b;
  real_T c1_w_y;
  real_T c1_v_hoistedGlobal;
  real_T c1_r_a;
  real_T c1_p_b;
  real_T *c1_b_X_kalman;
  real_T *c1_b_X_meas;
  real_T *c1_b_theta_kalman;
  real_T *c1_b_phi_kalman;
  real_T *c1_b_psi_kalman;
  real_T *c1_b_U1;
  real_T *c1_b_X_dot;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  c1_b_X_dot = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c1_b_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_b_psi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_b_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_b_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_X_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_X_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_X_meas;
  c1_b_hoistedGlobal = *c1_b_theta_kalman;
  c1_c_hoistedGlobal = *c1_b_phi_kalman;
  c1_d_hoistedGlobal = *c1_b_psi_kalman;
  c1_e_hoistedGlobal = *c1_b_U1;
  c1_f_hoistedGlobal = *c1_b_X_dot;
  c1_X_meas = c1_hoistedGlobal;
  c1_theta_kalman = c1_b_hoistedGlobal;
  c1_phi_kalman = c1_c_hoistedGlobal;
  c1_psi_kalman = c1_d_hoistedGlobal;
  c1_U1 = c1_e_hoistedGlobal;
  c1_X_dot = c1_f_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 21U, 21U, c1_debug_family_names,
    c1_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c1_g, 0U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c1_Ts, 1U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_m, 2U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c1_K, 3U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargin, 4U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargout, 5U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c1_X_meas, 6U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_theta_kalman, 7U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_phi_kalman, 8U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_psi_kalman, 9U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_U1, 10U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c1_X_dot, 11U, c1_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c1_X_kalman, 12U,
    c1_i_sf_marshallOut, c1_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_Pp, 13U,
    c1_h_sf_marshallOut, c1_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_Q, 14U,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_R, 15U,
    c1_f_sf_marshallOut, c1_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_xp, 16U,
    c1_e_sf_marshallOut, c1_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_A1, 17U,
    c1_d_sf_marshallOut, c1_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_H, 18U,
    c1_c_sf_marshallOut, c1_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_I, 19U,
    c1_b_sf_marshallOut, c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_xp_dot, 20U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  c1_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  c1_Ts = 0.01;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
  c1_m = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c1_Pp_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
    chartInstance->c1_Pp = 1.0;
    chartInstance->c1_Pp_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
    chartInstance->c1_Q = 5.0;
    chartInstance->c1_Q_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
    chartInstance->c1_R = 100.0;
    chartInstance->c1_R_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
    chartInstance->c1_xp = 0.0;
    chartInstance->c1_xp_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
    chartInstance->c1_A1 = 0.5;
    chartInstance->c1_A1_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
    chartInstance->c1_H = 1.0;
    chartInstance->c1_H_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
    chartInstance->c1_I = c1_eye(chartInstance);
    chartInstance->c1_I_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
    chartInstance->c1_xp_dot = 0.0;
    chartInstance->c1_xp_dot_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 14);
  c1_x = c1_psi_kalman;
  c1_b_x = c1_x;
  c1_b_x = muDoubleScalarSin(c1_b_x);
  c1_c_x = c1_phi_kalman;
  c1_d_x = c1_c_x;
  c1_d_x = muDoubleScalarSin(c1_d_x);
  c1_a = c1_b_x;
  c1_b = c1_d_x;
  c1_y = c1_a * c1_b;
  c1_e_x = c1_psi_kalman;
  c1_f_x = c1_e_x;
  c1_f_x = muDoubleScalarCos(c1_f_x);
  c1_g_x = c1_theta_kalman;
  c1_h_x = c1_g_x;
  c1_h_x = muDoubleScalarSin(c1_h_x);
  c1_b_a = c1_f_x;
  c1_b_b = c1_h_x;
  c1_b_y = c1_b_a * c1_b_b;
  c1_i_x = c1_phi_kalman;
  c1_j_x = c1_i_x;
  c1_j_x = muDoubleScalarCos(c1_j_x);
  c1_c_a = c1_b_y;
  c1_c_b = c1_j_x;
  c1_c_y = c1_c_a * c1_c_b;
  c1_d_a = c1_y + c1_c_y;
  c1_d_b = c1_U1;
  c1_d_y = c1_d_a * c1_d_b;
  c1_A = c1_d_y;
  c1_k_x = c1_A;
  c1_l_x = c1_k_x;
  c1_e_y = c1_l_x;
  c1_e_a = c1_X_dot;
  c1_f_y = c1_e_a * 1.2;
  c1_f_a = c1_e_y - c1_f_y;
  c1_g_y = c1_f_a * 0.01;
  chartInstance->c1_xp_dot += c1_g_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  c1_g_hoistedGlobal = chartInstance->c1_xp_dot;
  c1_g_a = c1_g_hoistedGlobal;
  c1_h_y = c1_g_a * 0.01;
  chartInstance->c1_xp += c1_h_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 16);
  c1_h_hoistedGlobal = chartInstance->c1_A1;
  c1_i_hoistedGlobal = chartInstance->c1_Pp;
  c1_h_a = c1_h_hoistedGlobal;
  c1_e_b = c1_i_hoistedGlobal;
  c1_i_y = c1_h_a * c1_e_b;
  c1_j_hoistedGlobal = chartInstance->c1_A1;
  c1_i_a = c1_i_y;
  c1_f_b = c1_j_hoistedGlobal;
  c1_j_y = c1_i_a * c1_f_b;
  chartInstance->c1_Pp = c1_j_y + chartInstance->c1_Q;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
  c1_k_hoistedGlobal = chartInstance->c1_Pp;
  c1_l_hoistedGlobal = chartInstance->c1_H;
  c1_j_a = c1_k_hoistedGlobal;
  c1_g_b = c1_l_hoistedGlobal;
  c1_k_y = c1_j_a * c1_g_b;
  c1_m_hoistedGlobal = chartInstance->c1_H;
  c1_n_hoistedGlobal = chartInstance->c1_Pp;
  c1_k_a = c1_m_hoistedGlobal;
  c1_h_b = c1_n_hoistedGlobal;
  c1_l_y = c1_k_a * c1_h_b;
  c1_o_hoistedGlobal = chartInstance->c1_H;
  c1_l_a = c1_l_y;
  c1_i_b = c1_o_hoistedGlobal;
  c1_m_y = c1_l_a * c1_i_b;
  c1_p_hoistedGlobal = chartInstance->c1_R;
  c1_m_x = c1_m_y + c1_p_hoistedGlobal;
  c1_n_y = c1_m_x;
  c1_o_y = 1.0 / c1_n_y;
  c1_n_x = c1_m_x;
  c1_xinv = c1_o_y;
  c1_o_x = c1_n_x;
  c1_p_x = c1_o_x;
  c1_q_x = c1_p_x;
  c1_n1x = muDoubleScalarAbs(c1_q_x);
  c1_r_x = c1_xinv;
  c1_s_x = c1_r_x;
  c1_t_x = c1_s_x;
  c1_n1xinv = muDoubleScalarAbs(c1_t_x);
  c1_m_a = c1_n1x;
  c1_j_b = c1_n1xinv;
  c1_p_y = c1_m_a * c1_j_b;
  c1_rc = 1.0 / c1_p_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c1_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c1_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c1_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c1_u_x = c1_rc;
    c1_k_b = muDoubleScalarIsNaN(c1_u_x);
    guard3 = FALSE;
    if (c1_k_b) {
      guard3 = TRUE;
    } else {
      if (c1_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c1_v_x = c1_rc;
      for (c1_i0 = 0; c1_i0 < 8; c1_i0++) {
        c1_u[c1_i0] = c1_cv0[c1_i0];
      }

      c1_q_y = NULL;
      sf_mex_assign(&c1_q_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c1_b_u = 14.0;
      c1_r_y = NULL;
      sf_mex_assign(&c1_r_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c1_c_u = 6.0;
      c1_s_y = NULL;
      sf_mex_assign(&c1_s_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c1_d_u = c1_v_x;
      c1_t_y = NULL;
      sf_mex_assign(&c1_t_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c1_s_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c1_q_y, 14, c1_r_y, 14,
        c1_s_y), 14, c1_t_y), "sprintf", c1_str);
      for (c1_i1 = 0; c1_i1 < 14; c1_i1++) {
        c1_b_str[c1_i1] = c1_str[c1_i1];
      }

      c1_b_eml_warning(chartInstance, c1_b_str);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c1_eml_warning(chartInstance);
  }

  c1_n_a = c1_k_y;
  c1_l_b = c1_o_y;
  c1_K = c1_n_a * c1_l_b;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
  c1_q_hoistedGlobal = chartInstance->c1_xp;
  c1_r_hoistedGlobal = chartInstance->c1_H;
  c1_s_hoistedGlobal = chartInstance->c1_xp;
  c1_o_a = c1_r_hoistedGlobal;
  c1_m_b = c1_s_hoistedGlobal;
  c1_u_y = c1_o_a * c1_m_b;
  c1_p_a = c1_K;
  c1_n_b = c1_X_meas - c1_u_y;
  c1_v_y = c1_p_a * c1_n_b;
  c1_X_kalman = c1_q_hoistedGlobal + c1_v_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
  c1_t_hoistedGlobal = chartInstance->c1_I;
  c1_u_hoistedGlobal = chartInstance->c1_H;
  c1_q_a = c1_K;
  c1_o_b = c1_u_hoistedGlobal;
  c1_w_y = c1_q_a * c1_o_b;
  c1_v_hoistedGlobal = chartInstance->c1_Pp;
  c1_r_a = c1_t_hoistedGlobal - c1_w_y;
  c1_p_b = c1_v_hoistedGlobal;
  chartInstance->c1_Pp = c1_r_a * c1_p_b;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  chartInstance->c1_xp = c1_X_kalman;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -23);
  sf_debug_symbol_scope_pop();
  *c1_b_X_kalman = c1_X_kalman;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_Quadrotor(SFc1_QuadrotorInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_xp_dot_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_xp_dot, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_xp_dot),
    &c1_thisId);
  sf_mex_destroy(&c1_b_xp_dot);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_xp_dot_not_empty = FALSE;
  } else {
    chartInstance->c1_xp_dot_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d0;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_xp_dot;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_xp_dot = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_xp_dot),
    &c1_thisId);
  sf_mex_destroy(&c1_b_xp_dot);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_I_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_I, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_I), &c1_thisId);
  sf_mex_destroy(&c1_b_I);
  return c1_y;
}

static real_T c1_d_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d1;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_I_not_empty = FALSE;
  } else {
    chartInstance->c1_I_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d1;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_I;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_I = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_I), &c1_thisId);
  sf_mex_destroy(&c1_b_I);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_H_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_e_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_H, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_H), &c1_thisId);
  sf_mex_destroy(&c1_b_H);
  return c1_y;
}

static real_T c1_f_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d2;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_H_not_empty = FALSE;
  } else {
    chartInstance->c1_H_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d2, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d2;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_H;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_H = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_H), &c1_thisId);
  sf_mex_destroy(&c1_b_H);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_A1_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_g_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_A1, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_A1), &c1_thisId);
  sf_mex_destroy(&c1_b_A1);
  return c1_y;
}

static real_T c1_h_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d3;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_A1_not_empty = FALSE;
  } else {
    chartInstance->c1_A1_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d3, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d3;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_A1;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_A1 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_A1), &c1_thisId);
  sf_mex_destroy(&c1_b_A1);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_xp_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_i_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_xp, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_xp), &c1_thisId);
  sf_mex_destroy(&c1_b_xp);
  return c1_y;
}

static real_T c1_j_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d4;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_xp_not_empty = FALSE;
  } else {
    chartInstance->c1_xp_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d4, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d4;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_xp;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_xp = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_xp), &c1_thisId);
  sf_mex_destroy(&c1_b_xp);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_R_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_k_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_R, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_R), &c1_thisId);
  sf_mex_destroy(&c1_b_R);
  return c1_y;
}

static real_T c1_l_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d5;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_R_not_empty = FALSE;
  } else {
    chartInstance->c1_R_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d5, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d5;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_R;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_R = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_R), &c1_thisId);
  sf_mex_destroy(&c1_b_R);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_Q_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_m_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_Q, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Q), &c1_thisId);
  sf_mex_destroy(&c1_b_Q);
  return c1_y;
}

static real_T c1_n_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d6;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_Q_not_empty = FALSE;
  } else {
    chartInstance->c1_Q_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d6, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d6;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_Q;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_Q = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Q), &c1_thisId);
  sf_mex_destroy(&c1_b_Q);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_Pp_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_o_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_Pp, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Pp), &c1_thisId);
  sf_mex_destroy(&c1_b_Pp);
  return c1_y;
}

static real_T c1_p_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d7;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_Pp_not_empty = FALSE;
  } else {
    chartInstance->c1_Pp_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d7, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d7;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_Pp;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_Pp = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Pp), &c1_thisId);
  sf_mex_destroy(&c1_b_Pp);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_q_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_X_kalman, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_X_kalman),
    &c1_thisId);
  sf_mex_destroy(&c1_X_kalman);
  return c1_y;
}

static real_T c1_r_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d8;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d8, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d8;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_X_kalman;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_X_kalman = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_X_kalman),
    &c1_thisId);
  sf_mex_destroy(&c1_X_kalman);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_Quadrotor_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[36];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i2;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_info_helper(c1_info);
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 36), FALSE);
  for (c1_i2 = 0; c1_i2 < 36; c1_i2++) {
    c1_r0 = &c1_info[c1_i2];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i2);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i2);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[36])
{
  c1_info[0].context = "";
  c1_info[0].name = "eye";
  c1_info[0].dominantType = "double";
  c1_info[0].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c1_info[0].fileTimeLo = 1286811488U;
  c1_info[0].fileTimeHi = 0U;
  c1_info[0].mFileTimeLo = 0U;
  c1_info[0].mFileTimeHi = 0U;
  c1_info[1].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[1].name = "eml_assert_valid_size_arg";
  c1_info[1].dominantType = "double";
  c1_info[1].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c1_info[1].fileTimeLo = 1286811494U;
  c1_info[1].fileTimeHi = 0U;
  c1_info[1].mFileTimeLo = 0U;
  c1_info[1].mFileTimeHi = 0U;
  c1_info[2].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c1_info[2].name = "isinf";
  c1_info[2].dominantType = "double";
  c1_info[2].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c1_info[2].fileTimeLo = 1286811560U;
  c1_info[2].fileTimeHi = 0U;
  c1_info[2].mFileTimeLo = 0U;
  c1_info[2].mFileTimeHi = 0U;
  c1_info[3].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c1_info[3].name = "mtimes";
  c1_info[3].dominantType = "double";
  c1_info[3].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[3].fileTimeLo = 1289508892U;
  c1_info[3].fileTimeHi = 0U;
  c1_info[3].mFileTimeLo = 0U;
  c1_info[3].mFileTimeHi = 0U;
  c1_info[4].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c1_info[4].name = "eml_index_class";
  c1_info[4].dominantType = "";
  c1_info[4].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[4].fileTimeLo = 1286811578U;
  c1_info[4].fileTimeHi = 0U;
  c1_info[4].mFileTimeLo = 0U;
  c1_info[4].mFileTimeHi = 0U;
  c1_info[5].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c1_info[5].name = "intmax";
  c1_info[5].dominantType = "char";
  c1_info[5].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[5].fileTimeLo = 1311248116U;
  c1_info[5].fileTimeHi = 0U;
  c1_info[5].mFileTimeLo = 0U;
  c1_info[5].mFileTimeHi = 0U;
  c1_info[6].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[6].name = "eml_is_float_class";
  c1_info[6].dominantType = "char";
  c1_info[6].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[6].fileTimeLo = 1286811582U;
  c1_info[6].fileTimeHi = 0U;
  c1_info[6].mFileTimeLo = 0U;
  c1_info[6].mFileTimeHi = 0U;
  c1_info[7].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[7].name = "min";
  c1_info[7].dominantType = "double";
  c1_info[7].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[7].fileTimeLo = 1311248118U;
  c1_info[7].fileTimeHi = 0U;
  c1_info[7].mFileTimeLo = 0U;
  c1_info[7].mFileTimeHi = 0U;
  c1_info[8].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[8].name = "eml_min_or_max";
  c1_info[8].dominantType = "char";
  c1_info[8].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c1_info[8].fileTimeLo = 1303139012U;
  c1_info[8].fileTimeHi = 0U;
  c1_info[8].mFileTimeLo = 0U;
  c1_info[8].mFileTimeHi = 0U;
  c1_info[9].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[9].name = "eml_scalar_eg";
  c1_info[9].dominantType = "double";
  c1_info[9].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[9].fileTimeLo = 1286811596U;
  c1_info[9].fileTimeHi = 0U;
  c1_info[9].mFileTimeLo = 0U;
  c1_info[9].mFileTimeHi = 0U;
  c1_info[10].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[10].name = "eml_scalexp_alloc";
  c1_info[10].dominantType = "double";
  c1_info[10].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[10].fileTimeLo = 1286811596U;
  c1_info[10].fileTimeHi = 0U;
  c1_info[10].mFileTimeLo = 0U;
  c1_info[10].mFileTimeHi = 0U;
  c1_info[11].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[11].name = "eml_index_class";
  c1_info[11].dominantType = "";
  c1_info[11].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[11].fileTimeLo = 1286811578U;
  c1_info[11].fileTimeHi = 0U;
  c1_info[11].mFileTimeLo = 0U;
  c1_info[11].mFileTimeHi = 0U;
  c1_info[12].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c1_info[12].name = "eml_scalar_eg";
  c1_info[12].dominantType = "double";
  c1_info[12].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[12].fileTimeLo = 1286811596U;
  c1_info[12].fileTimeHi = 0U;
  c1_info[12].mFileTimeLo = 0U;
  c1_info[12].mFileTimeHi = 0U;
  c1_info[13].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[13].name = "eml_index_class";
  c1_info[13].dominantType = "";
  c1_info[13].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[13].fileTimeLo = 1286811578U;
  c1_info[13].fileTimeHi = 0U;
  c1_info[13].mFileTimeLo = 0U;
  c1_info[13].mFileTimeHi = 0U;
  c1_info[14].context = "";
  c1_info[14].name = "sin";
  c1_info[14].dominantType = "double";
  c1_info[14].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c1_info[14].fileTimeLo = 1286811550U;
  c1_info[14].fileTimeHi = 0U;
  c1_info[14].mFileTimeLo = 0U;
  c1_info[14].mFileTimeHi = 0U;
  c1_info[15].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c1_info[15].name = "eml_scalar_sin";
  c1_info[15].dominantType = "double";
  c1_info[15].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c1_info[15].fileTimeLo = 1286811536U;
  c1_info[15].fileTimeHi = 0U;
  c1_info[15].mFileTimeLo = 0U;
  c1_info[15].mFileTimeHi = 0U;
  c1_info[16].context = "";
  c1_info[16].name = "mtimes";
  c1_info[16].dominantType = "double";
  c1_info[16].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[16].fileTimeLo = 1289508892U;
  c1_info[16].fileTimeHi = 0U;
  c1_info[16].mFileTimeLo = 0U;
  c1_info[16].mFileTimeHi = 0U;
  c1_info[17].context = "";
  c1_info[17].name = "cos";
  c1_info[17].dominantType = "double";
  c1_info[17].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c1_info[17].fileTimeLo = 1286811506U;
  c1_info[17].fileTimeHi = 0U;
  c1_info[17].mFileTimeLo = 0U;
  c1_info[17].mFileTimeHi = 0U;
  c1_info[18].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c1_info[18].name = "eml_scalar_cos";
  c1_info[18].dominantType = "double";
  c1_info[18].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c1_info[18].fileTimeLo = 1286811522U;
  c1_info[18].fileTimeHi = 0U;
  c1_info[18].mFileTimeLo = 0U;
  c1_info[18].mFileTimeHi = 0U;
  c1_info[19].context = "";
  c1_info[19].name = "mrdivide";
  c1_info[19].dominantType = "double";
  c1_info[19].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[19].fileTimeLo = 1325113338U;
  c1_info[19].fileTimeHi = 0U;
  c1_info[19].mFileTimeLo = 1319722766U;
  c1_info[19].mFileTimeHi = 0U;
  c1_info[20].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[20].name = "rdivide";
  c1_info[20].dominantType = "double";
  c1_info[20].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[20].fileTimeLo = 1286811644U;
  c1_info[20].fileTimeHi = 0U;
  c1_info[20].mFileTimeLo = 0U;
  c1_info[20].mFileTimeHi = 0U;
  c1_info[21].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[21].name = "eml_div";
  c1_info[21].dominantType = "double";
  c1_info[21].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[21].fileTimeLo = 1313340610U;
  c1_info[21].fileTimeHi = 0U;
  c1_info[21].mFileTimeLo = 0U;
  c1_info[21].mFileTimeHi = 0U;
  c1_info[22].context = "";
  c1_info[22].name = "inv";
  c1_info[22].dominantType = "double";
  c1_info[22].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c1_info[22].fileTimeLo = 1305310800U;
  c1_info[22].fileTimeHi = 0U;
  c1_info[22].mFileTimeLo = 0U;
  c1_info[22].mFileTimeHi = 0U;
  c1_info[23].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c1_info[23].name = "eml_div";
  c1_info[23].dominantType = "double";
  c1_info[23].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[23].fileTimeLo = 1313340610U;
  c1_info[23].fileTimeHi = 0U;
  c1_info[23].mFileTimeLo = 0U;
  c1_info[23].mFileTimeHi = 0U;
  c1_info[24].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c1_info[24].name = "norm";
  c1_info[24].dominantType = "double";
  c1_info[24].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c1_info[24].fileTimeLo = 1286811626U;
  c1_info[24].fileTimeHi = 0U;
  c1_info[24].mFileTimeLo = 0U;
  c1_info[24].mFileTimeHi = 0U;
  c1_info[25].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c1_info[25].name = "abs";
  c1_info[25].dominantType = "double";
  c1_info[25].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[25].fileTimeLo = 1286811494U;
  c1_info[25].fileTimeHi = 0U;
  c1_info[25].mFileTimeLo = 0U;
  c1_info[25].mFileTimeHi = 0U;
  c1_info[26].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[26].name = "eml_scalar_abs";
  c1_info[26].dominantType = "double";
  c1_info[26].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[26].fileTimeLo = 1286811512U;
  c1_info[26].fileTimeHi = 0U;
  c1_info[26].mFileTimeLo = 0U;
  c1_info[26].mFileTimeHi = 0U;
  c1_info[27].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c1_info[27].name = "mtimes";
  c1_info[27].dominantType = "double";
  c1_info[27].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[27].fileTimeLo = 1289508892U;
  c1_info[27].fileTimeHi = 0U;
  c1_info[27].mFileTimeLo = 0U;
  c1_info[27].mFileTimeHi = 0U;
  c1_info[28].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c1_info[28].name = "eml_warning";
  c1_info[28].dominantType = "char";
  c1_info[28].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c1_info[28].fileTimeLo = 1286811602U;
  c1_info[28].fileTimeHi = 0U;
  c1_info[28].mFileTimeLo = 0U;
  c1_info[28].mFileTimeHi = 0U;
  c1_info[29].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c1_info[29].name = "isnan";
  c1_info[29].dominantType = "double";
  c1_info[29].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c1_info[29].fileTimeLo = 1286811560U;
  c1_info[29].fileTimeHi = 0U;
  c1_info[29].mFileTimeLo = 0U;
  c1_info[29].mFileTimeHi = 0U;
  c1_info[30].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c1_info[30].name = "eps";
  c1_info[30].dominantType = "char";
  c1_info[30].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[30].fileTimeLo = 1307644040U;
  c1_info[30].fileTimeHi = 0U;
  c1_info[30].mFileTimeLo = 0U;
  c1_info[30].mFileTimeHi = 0U;
  c1_info[31].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[31].name = "eml_is_float_class";
  c1_info[31].dominantType = "char";
  c1_info[31].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[31].fileTimeLo = 1286811582U;
  c1_info[31].fileTimeHi = 0U;
  c1_info[31].mFileTimeLo = 0U;
  c1_info[31].mFileTimeHi = 0U;
  c1_info[32].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[32].name = "eml_eps";
  c1_info[32].dominantType = "char";
  c1_info[32].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c1_info[32].fileTimeLo = 1307644040U;
  c1_info[32].fileTimeHi = 0U;
  c1_info[32].mFileTimeLo = 0U;
  c1_info[32].mFileTimeHi = 0U;
  c1_info[33].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c1_info[33].name = "eml_float_model";
  c1_info[33].dominantType = "char";
  c1_info[33].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c1_info[33].fileTimeLo = 1307644042U;
  c1_info[33].fileTimeHi = 0U;
  c1_info[33].mFileTimeLo = 0U;
  c1_info[33].mFileTimeHi = 0U;
  c1_info[34].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c1_info[34].name = "eml_flt2str";
  c1_info[34].dominantType = "double";
  c1_info[34].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c1_info[34].fileTimeLo = 1309443996U;
  c1_info[34].fileTimeHi = 0U;
  c1_info[34].mFileTimeLo = 0U;
  c1_info[34].mFileTimeHi = 0U;
  c1_info[35].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c1_info[35].name = "char";
  c1_info[35].dominantType = "double";
  c1_info[35].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/strfun/char.m";
  c1_info[35].fileTimeLo = 1319722768U;
  c1_info[35].fileTimeHi = 0U;
  c1_info[35].mFileTimeLo = 0U;
  c1_info[35].mFileTimeHi = 0U;
}

static real_T c1_eye(SFc1_QuadrotorInstanceStruct *chartInstance)
{
  return 1.0;
}

static void c1_eml_warning(SFc1_QuadrotorInstanceStruct *chartInstance)
{
  int32_T c1_i3;
  static char_T c1_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c1_u[27];
  const mxArray *c1_y = NULL;
  for (c1_i3 = 0; c1_i3 < 27; c1_i3++) {
    c1_u[c1_i3] = c1_varargin_1[c1_i3];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c1_y));
}

static void c1_b_eml_warning(SFc1_QuadrotorInstanceStruct *chartInstance, char_T
  c1_varargin_2[14])
{
  int32_T c1_i4;
  static char_T c1_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c1_u[33];
  const mxArray *c1_y = NULL;
  int32_T c1_i5;
  char_T c1_b_u[14];
  const mxArray *c1_b_y = NULL;
  for (c1_i4 = 0; c1_i4 < 33; c1_i4++) {
    c1_u[c1_i4] = c1_varargin_1[c1_i4];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c1_i5 = 0; c1_i5 < 14; c1_i5++) {
    c1_b_u[c1_i5] = c1_varargin_2[c1_i5];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c1_y, 14, c1_b_y));
}

static void c1_s_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_sprintf, const char_T *c1_identifier, char_T c1_y[14])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_sprintf), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_sprintf);
}

static void c1_t_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, char_T c1_y[14])
{
  char_T c1_cv1[14];
  int32_T c1_i6;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c1_i6 = 0; c1_i6 < 14; c1_i6++) {
    c1_y[c1_i6] = c1_cv1[c1_i6];
  }

  sf_mex_destroy(&c1_u);
}

static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_u_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i7;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i7, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i7;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_v_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_Quadrotor, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_w_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_Quadrotor), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_Quadrotor);
  return c1_y;
}

static uint8_T c1_w_emlrt_marshallIn(SFc1_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_QuadrotorInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c1_Quadrotor_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(343804555U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3811591295U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1457802559U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3162783063U);
}

mxArray *sf_c1_Quadrotor_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("GZgZ7Vm21Ej0dwnpGrJwIE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
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

static const mxArray *sf_get_sim_state_info_c1_Quadrotor(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"X_kalman\",},{M[4],M[0],T\"A1\",S'l','i','p'{{M1x2[153 155],M[0],}}},{M[4],M[0],T\"H\",S'l','i','p'{{M1x2[156 157],M[0],}}},{M[4],M[0],T\"I\",S'l','i','p'{{M1x2[158 159],M[0],}}},{M[4],M[0],T\"Pp\",S'l','i','p'{{M1x2[143 145],M[0],}}},{M[4],M[0],T\"Q\",S'l','i','p'{{M1x2[146 147],M[0],}}},{M[4],M[0],T\"R\",S'l','i','p'{{M1x2[148 149],M[0],}}},{M[4],M[0],T\"xp\",S'l','i','p'{{M1x2[150 152],M[0],}}},{M[4],M[0],T\"xp_dot\",S'l','i','p'{{M1x2[160 166],M[0],}}},{M[8],M[0],T\"is_active_c1_Quadrotor\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 10, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_Quadrotor_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_QuadrotorInstanceStruct *chartInstance;
    chartInstance = (SFc1_QuadrotorInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_QuadrotorMachineNumber_,
          1,
          1,
          1,
          7,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"X_meas");
          _SFD_SET_DATA_PROPS(1,2,0,1,"X_kalman");
          _SFD_SET_DATA_PROPS(2,1,1,0,"theta_kalman");
          _SFD_SET_DATA_PROPS(3,1,1,0,"phi_kalman");
          _SFD_SET_DATA_PROPS(4,1,1,0,"psi_kalman");
          _SFD_SET_DATA_PROPS(5,1,1,0,"U1");
          _SFD_SET_DATA_PROPS(6,1,1,0,"X_dot");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,641);
        _SFD_CV_INIT_EML_IF(0,1,0,167,181,-1,280);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)c1_i_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_i_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c1_X_meas;
          real_T *c1_X_kalman;
          real_T *c1_theta_kalman;
          real_T *c1_phi_kalman;
          real_T *c1_psi_kalman;
          real_T *c1_U1;
          real_T *c1_X_dot;
          c1_X_dot = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c1_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c1_psi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c1_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c1_X_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_X_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c1_X_meas);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_X_kalman);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_theta_kalman);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_phi_kalman);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_psi_kalman);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_U1);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_X_dot);
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
  return "ZfxxQE31IEuUdTwkikIDi";
}

static void sf_opaque_initialize_c1_Quadrotor(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_QuadrotorInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*)
    chartInstanceVar);
  initialize_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_Quadrotor(void *chartInstanceVar)
{
  enable_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_Quadrotor(void *chartInstanceVar)
{
  disable_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_Quadrotor(void *chartInstanceVar)
{
  sf_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_Quadrotor(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_Quadrotor();/* state var info */
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

extern void sf_internal_set_sim_state_c1_Quadrotor(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_Quadrotor();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_Quadrotor(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_Quadrotor(S);
}

static void sf_opaque_set_sim_state_c1_Quadrotor(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c1_Quadrotor(S, st);
}

static void sf_opaque_terminate_c1_Quadrotor(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_QuadrotorInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Quadrotor_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_Quadrotor((SFc1_QuadrotorInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_Quadrotor(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_Quadrotor((SFc1_QuadrotorInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_Quadrotor(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Quadrotor_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2400794157U));
  ssSetChecksum1(S,(1086815260U));
  ssSetChecksum2(S,(4141195827U));
  ssSetChecksum3(S,(1289154588U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c1_Quadrotor(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_Quadrotor(SimStruct *S)
{
  SFc1_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuadrotorInstanceStruct *)malloc(sizeof
    (SFc1_QuadrotorInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_QuadrotorInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_Quadrotor;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_Quadrotor;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_Quadrotor;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_Quadrotor;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_Quadrotor;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_Quadrotor;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_Quadrotor;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_Quadrotor;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_Quadrotor;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_Quadrotor;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_Quadrotor;
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

void c1_Quadrotor_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_Quadrotor(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_Quadrotor(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_Quadrotor(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_Quadrotor_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
