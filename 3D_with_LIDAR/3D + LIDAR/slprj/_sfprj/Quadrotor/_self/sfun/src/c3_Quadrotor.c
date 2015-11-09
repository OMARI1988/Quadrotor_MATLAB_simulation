/* Include files */

#include "blascompat32.h"
#include "Quadrotor_sfun.h"
#include "c3_Quadrotor.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Quadrotor_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c3_debug_family_names[19] = { "g", "Ts", "m", "K", "nargin",
  "nargout", "Y_meas", "theta_kalman", "phi_kalman", "U1", "Y_kalman", "Pp", "Q",
  "R", "xp", "A1", "H", "I", "xp_dot" };

/* Function Declarations */
static void initialize_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance);
static void initialize_params_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance);
static void enable_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance);
static void disable_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance);
static void set_sim_state_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance, const mxArray *c3_st);
static void finalize_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance);
static void sf_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance);
static void c3_chartstep_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance);
static void initSimStructsc3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static real_T c3_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_xp_dot, const char_T *c3_identifier);
static real_T c3_b_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_c_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_I, const char_T *c3_identifier);
static real_T c3_d_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_e_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_H, const char_T *c3_identifier);
static real_T c3_f_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_g_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_A1, const char_T *c3_identifier);
static real_T c3_h_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_i_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_xp, const char_T *c3_identifier);
static real_T c3_j_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_k_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_R, const char_T *c3_identifier);
static real_T c3_l_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_m_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_Q, const char_T *c3_identifier);
static real_T c3_n_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_o_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_Pp, const char_T *c3_identifier);
static real_T c3_p_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_q_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_Y_kalman, const char_T *c3_identifier);
static real_T c3_r_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[34]);
static real_T c3_eye(SFc3_QuadrotorInstanceStruct *chartInstance);
static void c3_eml_warning(SFc3_QuadrotorInstanceStruct *chartInstance);
static void c3_b_eml_warning(SFc3_QuadrotorInstanceStruct *chartInstance, char_T
  c3_varargin_2[14]);
static void c3_s_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_sprintf, const char_T *c3_identifier, char_T c3_y[14]);
static void c3_t_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, char_T c3_y[14]);
static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_u_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_v_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_is_active_c3_Quadrotor, const char_T *c3_identifier);
static uint8_T c3_w_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info(SFc3_QuadrotorInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_Pp_not_empty = FALSE;
  chartInstance->c3_Q_not_empty = FALSE;
  chartInstance->c3_R_not_empty = FALSE;
  chartInstance->c3_xp_not_empty = FALSE;
  chartInstance->c3_A1_not_empty = FALSE;
  chartInstance->c3_H_not_empty = FALSE;
  chartInstance->c3_I_not_empty = FALSE;
  chartInstance->c3_xp_dot_not_empty = FALSE;
  chartInstance->c3_is_active_c3_Quadrotor = 0U;
}

static void initialize_params_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance)
{
}

static void enable_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_b_hoistedGlobal;
  real_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_c_hoistedGlobal;
  real_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_d_hoistedGlobal;
  real_T c3_d_u;
  const mxArray *c3_e_y = NULL;
  real_T c3_e_hoistedGlobal;
  real_T c3_e_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_f_hoistedGlobal;
  real_T c3_f_u;
  const mxArray *c3_g_y = NULL;
  real_T c3_g_hoistedGlobal;
  real_T c3_g_u;
  const mxArray *c3_h_y = NULL;
  real_T c3_h_hoistedGlobal;
  real_T c3_h_u;
  const mxArray *c3_i_y = NULL;
  real_T c3_i_hoistedGlobal;
  real_T c3_i_u;
  const mxArray *c3_j_y = NULL;
  uint8_T c3_j_hoistedGlobal;
  uint8_T c3_j_u;
  const mxArray *c3_k_y = NULL;
  real_T *c3_Y_kalman;
  c3_Y_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(10), FALSE);
  c3_hoistedGlobal = *c3_Y_kalman;
  c3_u = c3_hoistedGlobal;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_hoistedGlobal = chartInstance->c3_A1;
  c3_b_u = c3_b_hoistedGlobal;
  c3_c_y = NULL;
  if (!chartInstance->c3_A1_not_empty) {
    sf_mex_assign(&c3_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_c_hoistedGlobal = chartInstance->c3_H;
  c3_c_u = c3_c_hoistedGlobal;
  c3_d_y = NULL;
  if (!chartInstance->c3_H_not_empty) {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_d_hoistedGlobal = chartInstance->c3_I;
  c3_d_u = c3_d_hoistedGlobal;
  c3_e_y = NULL;
  if (!chartInstance->c3_I_not_empty) {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 3, c3_e_y);
  c3_e_hoistedGlobal = chartInstance->c3_Pp;
  c3_e_u = c3_e_hoistedGlobal;
  c3_f_y = NULL;
  if (!chartInstance->c3_Pp_not_empty) {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_f_hoistedGlobal = chartInstance->c3_Q;
  c3_f_u = c3_f_hoistedGlobal;
  c3_g_y = NULL;
  if (!chartInstance->c3_Q_not_empty) {
    sf_mex_assign(&c3_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 5, c3_g_y);
  c3_g_hoistedGlobal = chartInstance->c3_R;
  c3_g_u = c3_g_hoistedGlobal;
  c3_h_y = NULL;
  if (!chartInstance->c3_R_not_empty) {
    sf_mex_assign(&c3_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 6, c3_h_y);
  c3_h_hoistedGlobal = chartInstance->c3_xp;
  c3_h_u = c3_h_hoistedGlobal;
  c3_i_y = NULL;
  if (!chartInstance->c3_xp_not_empty) {
    sf_mex_assign(&c3_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_i_y, sf_mex_create("y", &c3_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 7, c3_i_y);
  c3_i_hoistedGlobal = chartInstance->c3_xp_dot;
  c3_i_u = c3_i_hoistedGlobal;
  c3_j_y = NULL;
  if (!chartInstance->c3_xp_dot_not_empty) {
    sf_mex_assign(&c3_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c3_y, 8, c3_j_y);
  c3_j_hoistedGlobal = chartInstance->c3_is_active_c3_Quadrotor;
  c3_j_u = c3_j_hoistedGlobal;
  c3_k_y = NULL;
  sf_mex_assign(&c3_k_y, sf_mex_create("y", &c3_j_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 9, c3_k_y);
  sf_mex_assign(&c3_st, c3_y, FALSE);
  return c3_st;
}

static void set_sim_state_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T *c3_Y_kalman;
  c3_Y_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  *c3_Y_kalman = c3_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c3_u, 0)), "Y_kalman");
  chartInstance->c3_A1 = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 1)), "A1");
  chartInstance->c3_H = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 2)), "H");
  chartInstance->c3_I = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 3)), "I");
  chartInstance->c3_Pp = c3_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 4)), "Pp");
  chartInstance->c3_Q = c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 5)), "Q");
  chartInstance->c3_R = c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 6)), "R");
  chartInstance->c3_xp = c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 7)), "xp");
  chartInstance->c3_xp_dot = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 8)), "xp_dot");
  chartInstance->c3_is_active_c3_Quadrotor = c3_v_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c3_u, 9)), "is_active_c3_Quadrotor");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_Quadrotor(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance)
{
}

static void sf_c3_Quadrotor(SFc3_QuadrotorInstanceStruct *chartInstance)
{
  real_T *c3_Y_meas;
  real_T *c3_Y_kalman;
  real_T *c3_theta_kalman;
  real_T *c3_phi_kalman;
  real_T *c3_U1;
  c3_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c3_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c3_Y_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_Y_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c3_Y_meas, 0U);
  _SFD_DATA_RANGE_CHECK(*c3_Y_kalman, 1U);
  _SFD_DATA_RANGE_CHECK(*c3_theta_kalman, 2U);
  _SFD_DATA_RANGE_CHECK(*c3_phi_kalman, 3U);
  _SFD_DATA_RANGE_CHECK(*c3_U1, 4U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_Quadrotor(chartInstance);
  sf_debug_check_for_state_inconsistency(_QuadrotorMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c3_chartstep_c3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_c_hoistedGlobal;
  real_T c3_d_hoistedGlobal;
  real_T c3_Y_meas;
  real_T c3_theta_kalman;
  real_T c3_phi_kalman;
  real_T c3_U1;
  uint32_T c3_debug_family_var_map[19];
  real_T c3_g;
  real_T c3_Ts;
  real_T c3_m;
  real_T c3_K;
  real_T c3_nargin = 4.0;
  real_T c3_nargout = 1.0;
  real_T c3_Y_kalman;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_a;
  real_T c3_b;
  real_T c3_y;
  real_T c3_b_a;
  real_T c3_b_b;
  real_T c3_b_y;
  real_T c3_A;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_c_y;
  real_T c3_c_a;
  real_T c3_d_y;
  real_T c3_e_hoistedGlobal;
  real_T c3_d_a;
  real_T c3_e_y;
  real_T c3_f_hoistedGlobal;
  real_T c3_g_hoistedGlobal;
  real_T c3_e_a;
  real_T c3_c_b;
  real_T c3_f_y;
  real_T c3_h_hoistedGlobal;
  real_T c3_f_a;
  real_T c3_d_b;
  real_T c3_g_y;
  real_T c3_i_hoistedGlobal;
  real_T c3_j_hoistedGlobal;
  real_T c3_g_a;
  real_T c3_e_b;
  real_T c3_h_y;
  real_T c3_k_hoistedGlobal;
  real_T c3_l_hoistedGlobal;
  real_T c3_h_a;
  real_T c3_f_b;
  real_T c3_i_y;
  real_T c3_m_hoistedGlobal;
  real_T c3_i_a;
  real_T c3_g_b;
  real_T c3_j_y;
  real_T c3_n_hoistedGlobal;
  real_T c3_g_x;
  real_T c3_k_y;
  real_T c3_l_y;
  real_T c3_h_x;
  real_T c3_xinv;
  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_k_x;
  real_T c3_n1x;
  real_T c3_l_x;
  real_T c3_m_x;
  real_T c3_n_x;
  real_T c3_n1xinv;
  real_T c3_j_a;
  real_T c3_h_b;
  real_T c3_m_y;
  real_T c3_rc;
  real_T c3_o_x;
  boolean_T c3_i_b;
  real_T c3_p_x;
  int32_T c3_i0;
  static char_T c3_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c3_u[8];
  const mxArray *c3_n_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_o_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_p_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_q_y = NULL;
  char_T c3_str[14];
  int32_T c3_i1;
  char_T c3_b_str[14];
  real_T c3_k_a;
  real_T c3_j_b;
  real_T c3_o_hoistedGlobal;
  real_T c3_p_hoistedGlobal;
  real_T c3_q_hoistedGlobal;
  real_T c3_l_a;
  real_T c3_k_b;
  real_T c3_r_y;
  real_T c3_m_a;
  real_T c3_l_b;
  real_T c3_s_y;
  real_T c3_r_hoistedGlobal;
  real_T c3_s_hoistedGlobal;
  real_T c3_n_a;
  real_T c3_m_b;
  real_T c3_t_y;
  real_T c3_t_hoistedGlobal;
  real_T c3_o_a;
  real_T c3_n_b;
  real_T *c3_b_Y_kalman;
  real_T *c3_b_Y_meas;
  real_T *c3_b_theta_kalman;
  real_T *c3_b_phi_kalman;
  real_T *c3_b_U1;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  c3_b_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c3_b_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_b_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c3_b_Y_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_b_Y_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *c3_b_Y_meas;
  c3_b_hoistedGlobal = *c3_b_theta_kalman;
  c3_c_hoistedGlobal = *c3_b_phi_kalman;
  c3_d_hoistedGlobal = *c3_b_U1;
  c3_Y_meas = c3_hoistedGlobal;
  c3_theta_kalman = c3_b_hoistedGlobal;
  c3_phi_kalman = c3_c_hoistedGlobal;
  c3_U1 = c3_d_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 19U, 19U, c3_debug_family_names,
    c3_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c3_g, 0U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c3_Ts, 1U, c3_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_m, 2U, c3_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c3_K, 3U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_nargin, 4U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_nargout, 5U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c3_Y_meas, 6U, c3_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_theta_kalman, 7U, c3_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_phi_kalman, 8U, c3_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_U1, 9U, c3_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c3_Y_kalman, 10U,
    c3_i_sf_marshallOut, c3_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_Pp, 11U,
    c3_h_sf_marshallOut, c3_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_Q, 12U,
    c3_g_sf_marshallOut, c3_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_R, 13U,
    c3_f_sf_marshallOut, c3_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_xp, 14U,
    c3_e_sf_marshallOut, c3_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_A1, 15U,
    c3_d_sf_marshallOut, c3_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_H, 16U,
    c3_c_sf_marshallOut, c3_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_I, 17U,
    c3_b_sf_marshallOut, c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c3_xp_dot, 18U,
    c3_sf_marshallOut, c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_g = 9.81;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_Ts = 0.01;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  c3_m = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 9);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c3_Pp_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 10);
    chartInstance->c3_Pp = 1.0;
    chartInstance->c3_Pp_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 10);
    chartInstance->c3_Q = 5.0;
    chartInstance->c3_Q_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 10);
    chartInstance->c3_R = 100.0;
    chartInstance->c3_R_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
    chartInstance->c3_xp = 0.0;
    chartInstance->c3_xp_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
    chartInstance->c3_A1 = 0.5;
    chartInstance->c3_A1_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
    chartInstance->c3_H = 1.0;
    chartInstance->c3_H_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
    chartInstance->c3_I = c3_eye(chartInstance);
    chartInstance->c3_I_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
    chartInstance->c3_xp_dot = 0.0;
    chartInstance->c3_xp_dot_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
  c3_x = c3_theta_kalman;
  c3_b_x = c3_x;
  c3_b_x = muDoubleScalarCos(c3_b_x);
  c3_c_x = c3_phi_kalman;
  c3_d_x = c3_c_x;
  c3_d_x = muDoubleScalarCos(c3_d_x);
  c3_a = c3_b_x;
  c3_b = c3_d_x;
  c3_y = c3_a * c3_b;
  c3_b_a = c3_y;
  c3_b_b = c3_U1;
  c3_b_y = c3_b_a * c3_b_b;
  c3_A = c3_b_y;
  c3_e_x = c3_A;
  c3_f_x = c3_e_x;
  c3_c_y = c3_f_x;
  c3_c_a = -c3_g + c3_c_y;
  c3_d_y = c3_c_a * 0.01;
  chartInstance->c3_xp_dot += c3_d_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 15);
  c3_e_hoistedGlobal = chartInstance->c3_xp_dot;
  c3_d_a = c3_e_hoistedGlobal;
  c3_e_y = c3_d_a * 0.01;
  chartInstance->c3_xp += c3_e_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 16);
  c3_f_hoistedGlobal = chartInstance->c3_A1;
  c3_g_hoistedGlobal = chartInstance->c3_Pp;
  c3_e_a = c3_f_hoistedGlobal;
  c3_c_b = c3_g_hoistedGlobal;
  c3_f_y = c3_e_a * c3_c_b;
  c3_h_hoistedGlobal = chartInstance->c3_A1;
  c3_f_a = c3_f_y;
  c3_d_b = c3_h_hoistedGlobal;
  c3_g_y = c3_f_a * c3_d_b;
  chartInstance->c3_Pp = c3_g_y + chartInstance->c3_Q;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
  c3_i_hoistedGlobal = chartInstance->c3_Pp;
  c3_j_hoistedGlobal = chartInstance->c3_H;
  c3_g_a = c3_i_hoistedGlobal;
  c3_e_b = c3_j_hoistedGlobal;
  c3_h_y = c3_g_a * c3_e_b;
  c3_k_hoistedGlobal = chartInstance->c3_H;
  c3_l_hoistedGlobal = chartInstance->c3_Pp;
  c3_h_a = c3_k_hoistedGlobal;
  c3_f_b = c3_l_hoistedGlobal;
  c3_i_y = c3_h_a * c3_f_b;
  c3_m_hoistedGlobal = chartInstance->c3_H;
  c3_i_a = c3_i_y;
  c3_g_b = c3_m_hoistedGlobal;
  c3_j_y = c3_i_a * c3_g_b;
  c3_n_hoistedGlobal = chartInstance->c3_R;
  c3_g_x = c3_j_y + c3_n_hoistedGlobal;
  c3_k_y = c3_g_x;
  c3_l_y = 1.0 / c3_k_y;
  c3_h_x = c3_g_x;
  c3_xinv = c3_l_y;
  c3_i_x = c3_h_x;
  c3_j_x = c3_i_x;
  c3_k_x = c3_j_x;
  c3_n1x = muDoubleScalarAbs(c3_k_x);
  c3_l_x = c3_xinv;
  c3_m_x = c3_l_x;
  c3_n_x = c3_m_x;
  c3_n1xinv = muDoubleScalarAbs(c3_n_x);
  c3_j_a = c3_n1x;
  c3_h_b = c3_n1xinv;
  c3_m_y = c3_j_a * c3_h_b;
  c3_rc = 1.0 / c3_m_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c3_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c3_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c3_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c3_o_x = c3_rc;
    c3_i_b = muDoubleScalarIsNaN(c3_o_x);
    guard3 = FALSE;
    if (c3_i_b) {
      guard3 = TRUE;
    } else {
      if (c3_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c3_p_x = c3_rc;
      for (c3_i0 = 0; c3_i0 < 8; c3_i0++) {
        c3_u[c3_i0] = c3_cv0[c3_i0];
      }

      c3_n_y = NULL;
      sf_mex_assign(&c3_n_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c3_b_u = 14.0;
      c3_o_y = NULL;
      sf_mex_assign(&c3_o_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c3_c_u = 6.0;
      c3_p_y = NULL;
      sf_mex_assign(&c3_p_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c3_d_u = c3_p_x;
      c3_q_y = NULL;
      sf_mex_assign(&c3_q_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c3_s_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c3_n_y, 14, c3_o_y, 14,
        c3_p_y), 14, c3_q_y), "sprintf", c3_str);
      for (c3_i1 = 0; c3_i1 < 14; c3_i1++) {
        c3_b_str[c3_i1] = c3_str[c3_i1];
      }

      c3_b_eml_warning(chartInstance, c3_b_str);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c3_eml_warning(chartInstance);
  }

  c3_k_a = c3_h_y;
  c3_j_b = c3_l_y;
  c3_K = c3_k_a * c3_j_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 19);
  c3_o_hoistedGlobal = chartInstance->c3_xp;
  c3_p_hoistedGlobal = chartInstance->c3_H;
  c3_q_hoistedGlobal = chartInstance->c3_xp;
  c3_l_a = c3_p_hoistedGlobal;
  c3_k_b = c3_q_hoistedGlobal;
  c3_r_y = c3_l_a * c3_k_b;
  c3_m_a = c3_K;
  c3_l_b = c3_Y_meas - c3_r_y;
  c3_s_y = c3_m_a * c3_l_b;
  c3_Y_kalman = c3_o_hoistedGlobal + c3_s_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 22);
  c3_r_hoistedGlobal = chartInstance->c3_I;
  c3_s_hoistedGlobal = chartInstance->c3_H;
  c3_n_a = c3_K;
  c3_m_b = c3_s_hoistedGlobal;
  c3_t_y = c3_n_a * c3_m_b;
  c3_t_hoistedGlobal = chartInstance->c3_Pp;
  c3_o_a = c3_r_hoistedGlobal - c3_t_y;
  c3_n_b = c3_t_hoistedGlobal;
  chartInstance->c3_Pp = c3_o_a * c3_n_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
  chartInstance->c3_xp = c3_Y_kalman;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -23);
  sf_debug_symbol_scope_pop();
  *c3_b_Y_kalman = c3_Y_kalman;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_Quadrotor(SFc3_QuadrotorInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_xp_dot_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_xp_dot, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_xp_dot),
    &c3_thisId);
  sf_mex_destroy(&c3_b_xp_dot);
  return c3_y;
}

static real_T c3_b_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_xp_dot_not_empty = FALSE;
  } else {
    chartInstance->c3_xp_dot_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d0;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_xp_dot;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_xp_dot = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_xp_dot),
    &c3_thisId);
  sf_mex_destroy(&c3_b_xp_dot);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_I_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_c_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_I, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_I), &c3_thisId);
  sf_mex_destroy(&c3_b_I);
  return c3_y;
}

static real_T c3_d_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d1;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_I_not_empty = FALSE;
  } else {
    chartInstance->c3_I_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d1, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d1;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_I;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_I = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_I), &c3_thisId);
  sf_mex_destroy(&c3_b_I);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_H_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_e_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_H, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_H), &c3_thisId);
  sf_mex_destroy(&c3_b_H);
  return c3_y;
}

static real_T c3_f_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d2;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_H_not_empty = FALSE;
  } else {
    chartInstance->c3_H_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d2, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d2;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_H;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_H = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_H), &c3_thisId);
  sf_mex_destroy(&c3_b_H);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_A1_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_g_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_A1, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_A1), &c3_thisId);
  sf_mex_destroy(&c3_b_A1);
  return c3_y;
}

static real_T c3_h_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d3;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_A1_not_empty = FALSE;
  } else {
    chartInstance->c3_A1_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d3, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d3;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_A1;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_A1 = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_A1), &c3_thisId);
  sf_mex_destroy(&c3_b_A1);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_xp_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_i_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_xp, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_xp), &c3_thisId);
  sf_mex_destroy(&c3_b_xp);
  return c3_y;
}

static real_T c3_j_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d4;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_xp_not_empty = FALSE;
  } else {
    chartInstance->c3_xp_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d4, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d4;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_xp;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_xp = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_xp), &c3_thisId);
  sf_mex_destroy(&c3_b_xp);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_R_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_k_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_R, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_R), &c3_thisId);
  sf_mex_destroy(&c3_b_R);
  return c3_y;
}

static real_T c3_l_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d5;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_R_not_empty = FALSE;
  } else {
    chartInstance->c3_R_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d5, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d5;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_R;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_R = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_R), &c3_thisId);
  sf_mex_destroy(&c3_b_R);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_Q_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_m_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_Q, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_Q), &c3_thisId);
  sf_mex_destroy(&c3_b_Q);
  return c3_y;
}

static real_T c3_n_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d6;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_Q_not_empty = FALSE;
  } else {
    chartInstance->c3_Q_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d6, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d6;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_Q;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_Q = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_Q), &c3_thisId);
  sf_mex_destroy(&c3_b_Q);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_Pp_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_o_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_Pp, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_Pp), &c3_thisId);
  sf_mex_destroy(&c3_b_Pp);
  return c3_y;
}

static real_T c3_p_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d7;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_Pp_not_empty = FALSE;
  } else {
    chartInstance->c3_Pp_not_empty = TRUE;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d7, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d7;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_Pp;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_Pp = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_Pp), &c3_thisId);
  sf_mex_destroy(&c3_b_Pp);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_q_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_Y_kalman, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Y_kalman),
    &c3_thisId);
  sf_mex_destroy(&c3_Y_kalman);
  return c3_y;
}

static real_T c3_r_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d8;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d8, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d8;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_Y_kalman;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_Y_kalman = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Y_kalman),
    &c3_thisId);
  sf_mex_destroy(&c3_Y_kalman);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_Quadrotor_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo;
  c3_ResolvedFunctionInfo c3_info[34];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i2;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  c3_info_helper(c3_info);
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 34), FALSE);
  for (c3_i2 = 0; c3_i2 < 34; c3_i2++) {
    c3_r0 = &c3_info[c3_i2];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context", "nameCaptureInfo",
                    c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name", "nameCaptureInfo", c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved", "nameCaptureInfo",
                    c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c3_i2);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c3_i2);
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[34])
{
  c3_info[0].context = "";
  c3_info[0].name = "eye";
  c3_info[0].dominantType = "double";
  c3_info[0].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c3_info[0].fileTimeLo = 1286811488U;
  c3_info[0].fileTimeHi = 0U;
  c3_info[0].mFileTimeLo = 0U;
  c3_info[0].mFileTimeHi = 0U;
  c3_info[1].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c3_info[1].name = "eml_assert_valid_size_arg";
  c3_info[1].dominantType = "double";
  c3_info[1].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c3_info[1].fileTimeLo = 1286811494U;
  c3_info[1].fileTimeHi = 0U;
  c3_info[1].mFileTimeLo = 0U;
  c3_info[1].mFileTimeHi = 0U;
  c3_info[2].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c3_info[2].name = "isinf";
  c3_info[2].dominantType = "double";
  c3_info[2].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c3_info[2].fileTimeLo = 1286811560U;
  c3_info[2].fileTimeHi = 0U;
  c3_info[2].mFileTimeLo = 0U;
  c3_info[2].mFileTimeHi = 0U;
  c3_info[3].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c3_info[3].name = "mtimes";
  c3_info[3].dominantType = "double";
  c3_info[3].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[3].fileTimeLo = 1289508892U;
  c3_info[3].fileTimeHi = 0U;
  c3_info[3].mFileTimeLo = 0U;
  c3_info[3].mFileTimeHi = 0U;
  c3_info[4].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c3_info[4].name = "eml_index_class";
  c3_info[4].dominantType = "";
  c3_info[4].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[4].fileTimeLo = 1286811578U;
  c3_info[4].fileTimeHi = 0U;
  c3_info[4].mFileTimeLo = 0U;
  c3_info[4].mFileTimeHi = 0U;
  c3_info[5].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c3_info[5].name = "intmax";
  c3_info[5].dominantType = "char";
  c3_info[5].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c3_info[5].fileTimeLo = 1311248116U;
  c3_info[5].fileTimeHi = 0U;
  c3_info[5].mFileTimeLo = 0U;
  c3_info[5].mFileTimeHi = 0U;
  c3_info[6].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c3_info[6].name = "eml_is_float_class";
  c3_info[6].dominantType = "char";
  c3_info[6].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c3_info[6].fileTimeLo = 1286811582U;
  c3_info[6].fileTimeHi = 0U;
  c3_info[6].mFileTimeLo = 0U;
  c3_info[6].mFileTimeHi = 0U;
  c3_info[7].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c3_info[7].name = "min";
  c3_info[7].dominantType = "double";
  c3_info[7].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c3_info[7].fileTimeLo = 1311248118U;
  c3_info[7].fileTimeHi = 0U;
  c3_info[7].mFileTimeLo = 0U;
  c3_info[7].mFileTimeHi = 0U;
  c3_info[8].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c3_info[8].name = "eml_min_or_max";
  c3_info[8].dominantType = "char";
  c3_info[8].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c3_info[8].fileTimeLo = 1303139012U;
  c3_info[8].fileTimeHi = 0U;
  c3_info[8].mFileTimeLo = 0U;
  c3_info[8].mFileTimeHi = 0U;
  c3_info[9].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c3_info[9].name = "eml_scalar_eg";
  c3_info[9].dominantType = "double";
  c3_info[9].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[9].fileTimeLo = 1286811596U;
  c3_info[9].fileTimeHi = 0U;
  c3_info[9].mFileTimeLo = 0U;
  c3_info[9].mFileTimeHi = 0U;
  c3_info[10].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c3_info[10].name = "eml_scalexp_alloc";
  c3_info[10].dominantType = "double";
  c3_info[10].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c3_info[10].fileTimeLo = 1286811596U;
  c3_info[10].fileTimeHi = 0U;
  c3_info[10].mFileTimeLo = 0U;
  c3_info[10].mFileTimeHi = 0U;
  c3_info[11].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c3_info[11].name = "eml_index_class";
  c3_info[11].dominantType = "";
  c3_info[11].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[11].fileTimeLo = 1286811578U;
  c3_info[11].fileTimeHi = 0U;
  c3_info[11].mFileTimeLo = 0U;
  c3_info[11].mFileTimeHi = 0U;
  c3_info[12].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c3_info[12].name = "eml_scalar_eg";
  c3_info[12].dominantType = "double";
  c3_info[12].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[12].fileTimeLo = 1286811596U;
  c3_info[12].fileTimeHi = 0U;
  c3_info[12].mFileTimeLo = 0U;
  c3_info[12].mFileTimeHi = 0U;
  c3_info[13].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c3_info[13].name = "eml_index_class";
  c3_info[13].dominantType = "";
  c3_info[13].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[13].fileTimeLo = 1286811578U;
  c3_info[13].fileTimeHi = 0U;
  c3_info[13].mFileTimeLo = 0U;
  c3_info[13].mFileTimeHi = 0U;
  c3_info[14].context = "";
  c3_info[14].name = "cos";
  c3_info[14].dominantType = "double";
  c3_info[14].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c3_info[14].fileTimeLo = 1286811506U;
  c3_info[14].fileTimeHi = 0U;
  c3_info[14].mFileTimeLo = 0U;
  c3_info[14].mFileTimeHi = 0U;
  c3_info[15].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c3_info[15].name = "eml_scalar_cos";
  c3_info[15].dominantType = "double";
  c3_info[15].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c3_info[15].fileTimeLo = 1286811522U;
  c3_info[15].fileTimeHi = 0U;
  c3_info[15].mFileTimeLo = 0U;
  c3_info[15].mFileTimeHi = 0U;
  c3_info[16].context = "";
  c3_info[16].name = "mtimes";
  c3_info[16].dominantType = "double";
  c3_info[16].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[16].fileTimeLo = 1289508892U;
  c3_info[16].fileTimeHi = 0U;
  c3_info[16].mFileTimeLo = 0U;
  c3_info[16].mFileTimeHi = 0U;
  c3_info[17].context = "";
  c3_info[17].name = "mrdivide";
  c3_info[17].dominantType = "double";
  c3_info[17].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[17].fileTimeLo = 1325113338U;
  c3_info[17].fileTimeHi = 0U;
  c3_info[17].mFileTimeLo = 1319722766U;
  c3_info[17].mFileTimeHi = 0U;
  c3_info[18].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[18].name = "rdivide";
  c3_info[18].dominantType = "double";
  c3_info[18].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[18].fileTimeLo = 1286811644U;
  c3_info[18].fileTimeHi = 0U;
  c3_info[18].mFileTimeLo = 0U;
  c3_info[18].mFileTimeHi = 0U;
  c3_info[19].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[19].name = "eml_div";
  c3_info[19].dominantType = "double";
  c3_info[19].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[19].fileTimeLo = 1313340610U;
  c3_info[19].fileTimeHi = 0U;
  c3_info[19].mFileTimeLo = 0U;
  c3_info[19].mFileTimeHi = 0U;
  c3_info[20].context = "";
  c3_info[20].name = "inv";
  c3_info[20].dominantType = "double";
  c3_info[20].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c3_info[20].fileTimeLo = 1305310800U;
  c3_info[20].fileTimeHi = 0U;
  c3_info[20].mFileTimeLo = 0U;
  c3_info[20].mFileTimeHi = 0U;
  c3_info[21].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c3_info[21].name = "eml_div";
  c3_info[21].dominantType = "double";
  c3_info[21].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[21].fileTimeLo = 1313340610U;
  c3_info[21].fileTimeHi = 0U;
  c3_info[21].mFileTimeLo = 0U;
  c3_info[21].mFileTimeHi = 0U;
  c3_info[22].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c3_info[22].name = "norm";
  c3_info[22].dominantType = "double";
  c3_info[22].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c3_info[22].fileTimeLo = 1286811626U;
  c3_info[22].fileTimeHi = 0U;
  c3_info[22].mFileTimeLo = 0U;
  c3_info[22].mFileTimeHi = 0U;
  c3_info[23].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c3_info[23].name = "abs";
  c3_info[23].dominantType = "double";
  c3_info[23].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c3_info[23].fileTimeLo = 1286811494U;
  c3_info[23].fileTimeHi = 0U;
  c3_info[23].mFileTimeLo = 0U;
  c3_info[23].mFileTimeHi = 0U;
  c3_info[24].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c3_info[24].name = "eml_scalar_abs";
  c3_info[24].dominantType = "double";
  c3_info[24].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c3_info[24].fileTimeLo = 1286811512U;
  c3_info[24].fileTimeHi = 0U;
  c3_info[24].mFileTimeLo = 0U;
  c3_info[24].mFileTimeHi = 0U;
  c3_info[25].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c3_info[25].name = "mtimes";
  c3_info[25].dominantType = "double";
  c3_info[25].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[25].fileTimeLo = 1289508892U;
  c3_info[25].fileTimeHi = 0U;
  c3_info[25].mFileTimeLo = 0U;
  c3_info[25].mFileTimeHi = 0U;
  c3_info[26].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c3_info[26].name = "eml_warning";
  c3_info[26].dominantType = "char";
  c3_info[26].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c3_info[26].fileTimeLo = 1286811602U;
  c3_info[26].fileTimeHi = 0U;
  c3_info[26].mFileTimeLo = 0U;
  c3_info[26].mFileTimeHi = 0U;
  c3_info[27].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c3_info[27].name = "isnan";
  c3_info[27].dominantType = "double";
  c3_info[27].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c3_info[27].fileTimeLo = 1286811560U;
  c3_info[27].fileTimeHi = 0U;
  c3_info[27].mFileTimeLo = 0U;
  c3_info[27].mFileTimeHi = 0U;
  c3_info[28].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c3_info[28].name = "eps";
  c3_info[28].dominantType = "char";
  c3_info[28].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c3_info[28].fileTimeLo = 1307644040U;
  c3_info[28].fileTimeHi = 0U;
  c3_info[28].mFileTimeLo = 0U;
  c3_info[28].mFileTimeHi = 0U;
  c3_info[29].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c3_info[29].name = "eml_is_float_class";
  c3_info[29].dominantType = "char";
  c3_info[29].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c3_info[29].fileTimeLo = 1286811582U;
  c3_info[29].fileTimeHi = 0U;
  c3_info[29].mFileTimeLo = 0U;
  c3_info[29].mFileTimeHi = 0U;
  c3_info[30].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c3_info[30].name = "eml_eps";
  c3_info[30].dominantType = "char";
  c3_info[30].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c3_info[30].fileTimeLo = 1307644040U;
  c3_info[30].fileTimeHi = 0U;
  c3_info[30].mFileTimeLo = 0U;
  c3_info[30].mFileTimeHi = 0U;
  c3_info[31].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c3_info[31].name = "eml_float_model";
  c3_info[31].dominantType = "char";
  c3_info[31].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c3_info[31].fileTimeLo = 1307644042U;
  c3_info[31].fileTimeHi = 0U;
  c3_info[31].mFileTimeLo = 0U;
  c3_info[31].mFileTimeHi = 0U;
  c3_info[32].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c3_info[32].name = "eml_flt2str";
  c3_info[32].dominantType = "double";
  c3_info[32].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c3_info[32].fileTimeLo = 1309443996U;
  c3_info[32].fileTimeHi = 0U;
  c3_info[32].mFileTimeLo = 0U;
  c3_info[32].mFileTimeHi = 0U;
  c3_info[33].context =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c3_info[33].name = "char";
  c3_info[33].dominantType = "double";
  c3_info[33].resolved =
    "[ILXE]C:/Program Files/MATLAB/R2012a/toolbox/eml/lib/matlab/strfun/char.m";
  c3_info[33].fileTimeLo = 1319722768U;
  c3_info[33].fileTimeHi = 0U;
  c3_info[33].mFileTimeLo = 0U;
  c3_info[33].mFileTimeHi = 0U;
}

static real_T c3_eye(SFc3_QuadrotorInstanceStruct *chartInstance)
{
  return 1.0;
}

static void c3_eml_warning(SFc3_QuadrotorInstanceStruct *chartInstance)
{
  int32_T c3_i3;
  static char_T c3_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c3_u[27];
  const mxArray *c3_y = NULL;
  for (c3_i3 = 0; c3_i3 < 27; c3_i3++) {
    c3_u[c3_i3] = c3_varargin_1[c3_i3];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c3_y));
}

static void c3_b_eml_warning(SFc3_QuadrotorInstanceStruct *chartInstance, char_T
  c3_varargin_2[14])
{
  int32_T c3_i4;
  static char_T c3_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c3_u[33];
  const mxArray *c3_y = NULL;
  int32_T c3_i5;
  char_T c3_b_u[14];
  const mxArray *c3_b_y = NULL;
  for (c3_i4 = 0; c3_i4 < 33; c3_i4++) {
    c3_u[c3_i4] = c3_varargin_1[c3_i4];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c3_i5 = 0; c3_i5 < 14; c3_i5++) {
    c3_b_u[c3_i5] = c3_varargin_2[c3_i5];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c3_y, 14, c3_b_y));
}

static void c3_s_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_sprintf, const char_T *c3_identifier, char_T c3_y[14])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_sprintf), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_sprintf);
}

static void c3_t_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId, char_T c3_y[14])
{
  char_T c3_cv1[14];
  int32_T c3_i6;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c3_i6 = 0; c3_i6 < 14; c3_i6++) {
    c3_y[c3_i6] = c3_cv1[c3_i6];
  }

  sf_mex_destroy(&c3_u);
}

static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static int32_T c3_u_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i7;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i7, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i7;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_v_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_b_is_active_c3_Quadrotor, const char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_w_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_Quadrotor), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_Quadrotor);
  return c3_y;
}

static uint8_T c3_w_emlrt_marshallIn(SFc3_QuadrotorInstanceStruct *chartInstance,
  const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void init_dsm_address_info(SFc3_QuadrotorInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c3_Quadrotor_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(251009768U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4088020741U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3899175670U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4234006379U);
}

mxArray *sf_c3_Quadrotor_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("54PsOtPhDhzllmAGkvESz");
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

static const mxArray *sf_get_sim_state_info_c3_Quadrotor(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"Y_kalman\",},{M[4],M[0],T\"A1\",S'l','i','p'{{M1x2[136 138],M[0],}}},{M[4],M[0],T\"H\",S'l','i','p'{{M1x2[139 140],M[0],}}},{M[4],M[0],T\"I\",S'l','i','p'{{M1x2[141 142],M[0],}}},{M[4],M[0],T\"Pp\",S'l','i','p'{{M1x2[126 128],M[0],}}},{M[4],M[0],T\"Q\",S'l','i','p'{{M1x2[129 130],M[0],}}},{M[4],M[0],T\"R\",S'l','i','p'{{M1x2[131 132],M[0],}}},{M[4],M[0],T\"xp\",S'l','i','p'{{M1x2[133 135],M[0],}}},{M[4],M[0],T\"xp_dot\",S'l','i','p'{{M1x2[143 149],M[0],}}},{M[8],M[0],T\"is_active_c3_Quadrotor\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 10, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_Quadrotor_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_QuadrotorInstanceStruct *chartInstance;
    chartInstance = (SFc3_QuadrotorInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_QuadrotorMachineNumber_,
          3,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"Y_meas");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Y_kalman");
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
          (MexFcnForType)c3_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_i_sf_marshallOut,(MexInFcnForType)c3_i_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_i_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_i_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c3_Y_meas;
          real_T *c3_Y_kalman;
          real_T *c3_theta_kalman;
          real_T *c3_phi_kalman;
          real_T *c3_U1;
          c3_U1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c3_phi_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c3_theta_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c3_Y_kalman = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c3_Y_meas = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_Y_meas);
          _SFD_SET_DATA_VALUE_PTR(1U, c3_Y_kalman);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_theta_kalman);
          _SFD_SET_DATA_VALUE_PTR(3U, c3_phi_kalman);
          _SFD_SET_DATA_VALUE_PTR(4U, c3_U1);
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
  return "01Yt7r9mvv6kgKbp23oyGC";
}

static void sf_opaque_initialize_c3_Quadrotor(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_QuadrotorInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*)
    chartInstanceVar);
  initialize_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_Quadrotor(void *chartInstanceVar)
{
  enable_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_Quadrotor(void *chartInstanceVar)
{
  disable_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_Quadrotor(void *chartInstanceVar)
{
  sf_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_Quadrotor(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_Quadrotor();/* state var info */
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

extern void sf_internal_set_sim_state_c3_Quadrotor(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_Quadrotor();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_Quadrotor(SimStruct* S)
{
  return sf_internal_get_sim_state_c3_Quadrotor(S);
}

static void sf_opaque_set_sim_state_c3_Quadrotor(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c3_Quadrotor(S, st);
}

static void sf_opaque_terminate_c3_Quadrotor(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_QuadrotorInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Quadrotor_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_Quadrotor((SFc3_QuadrotorInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_Quadrotor(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_Quadrotor((SFc3_QuadrotorInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_Quadrotor(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Quadrotor_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,3,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3881891500U));
  ssSetChecksum1(S,(1528812925U));
  ssSetChecksum2(S,(3610971103U));
  ssSetChecksum3(S,(3666570878U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c3_Quadrotor(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_Quadrotor(SimStruct *S)
{
  SFc3_QuadrotorInstanceStruct *chartInstance;
  chartInstance = (SFc3_QuadrotorInstanceStruct *)malloc(sizeof
    (SFc3_QuadrotorInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_QuadrotorInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c3_Quadrotor;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c3_Quadrotor;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c3_Quadrotor;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_Quadrotor;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_Quadrotor;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c3_Quadrotor;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c3_Quadrotor;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c3_Quadrotor;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_Quadrotor;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_Quadrotor;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_Quadrotor;
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

void c3_Quadrotor_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_Quadrotor(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_Quadrotor(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_Quadrotor(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_Quadrotor_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
