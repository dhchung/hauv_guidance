#include <algorithm>
#include <functional>
#include <vector>
#include <math.h>


class CGuidance
{
public:
    CGuidance();
    ~CGuidance();
    //6DoF

    void SetGains(float p_gain_pos_X, //Surge
                  float i_gain_pos_X,
                  float d_gain_pos_X,
                  float p_gain_pos_Y, //Sway
                  float i_gain_pos_Y,
                  float d_gain_pos_Y,
                  float p_gain_pos_Z, //Heave
                  float i_gain_pos_Z,
                  float d_gain_pos_Z,
                  float p_gain_att_K, //Roll
                  float i_gain_att_K,
                  float d_gain_att_K,
                  float p_gain_att_M, //Pitch
                  float i_gain_att_M,
                  float d_gain_att_M,
                  float p_gain_att_N, //Yaw
                  float i_gain_att_N,
                  float d_gain_att_N,
                  float p_cte = 0.0f,
                  float d_cte = 0.0f);

    void SetGains(float p_pos,
                  float i_pos,
                  float d_pos,
                  float p_att,
                  float i_att,
                  float d_att,
                  float p_cte = 0.0f,
                  float d_cte = 0.0f);

    void SetForgetFactor(float i_pos_forget_X,
                         float i_pos_forget_Y,
                         float i_pos_forget_Z,
                         float i_att_forget_K,
                         float i_att_forget_M,
                         float i_att_forget_N);

    void SetForgetFactor(float i_pos_forget,
                         float i_att_forget);

    void ResetIncrementalError();

    std::vector<float> ProcessGuidance(float time,
                                       std::vector<float> curr_pos,
                                       std::vector<float> curr_att,
                                       std::vector<float> des_pos,
                                       bool LOS_ctrl = true,
                                       std::vector<float> des_att = std::vector<float>(3, 0.0f),
                                       std::vector<float> prev_wpt = std::vector<float>(3, 0.0f),
                                       bool cross_track_ctrl = false);

    float GetLeastErrorDiff(float radian);
    float GetNormOfVector(std::vector<float> vec);

private:
    std::vector<float> m_prev_error;
    std::vector<float> m_prev_cte_error;

    std::vector<float> m_p_gain;
    std::vector<float> m_i_gain;
    std::vector<float> m_d_gain;

    float m_cte_p_gain;
    float m_cte_d_gain;

    float m_previous_time;

    std::vector<float> m_i_forget_factor;
    std::vector<float> m_incremental_error;
};