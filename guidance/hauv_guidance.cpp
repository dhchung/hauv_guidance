#include "hauv_guidance.h"
CGuidance::CGuidance() {
    m_prev_error = std::vector<float>(6, 0.0f);
    m_prev_cte_error = std::vector<float>{3, 0.0f};

    m_incremental_error = std::vector<float>(6, 0.0f);
    m_p_gain = std::vector<float>(6, 0.0f);
    m_i_gain = std::vector<float>(6, 0.0f);
    m_d_gain = std::vector<float>(6, 0.0f);
    m_cte_p_gain = 0.0f;
    m_cte_d_gain = 0.0f;

    m_previous_time = 0.0f;
    m_i_forget_factor = std::vector<float>(6, 0.0f);
}
CGuidance::~CGuidance() {

}

void CGuidance::SetGains(float p_gain_pos_X,            //Surge
                         float i_gain_pos_X,
                         float d_gain_pos_X,
                         float p_gain_pos_Y,            //Sway
                         float i_gain_pos_Y,
                         float d_gain_pos_Y,
                         float p_gain_pos_Z,            //Heave
                         float i_gain_pos_Z,
                         float d_gain_pos_Z,
                         float p_gain_att_K,            //Roll
                         float i_gain_att_K,
                         float d_gain_att_K,
                         float p_gain_att_M,            //Pitch
                         float i_gain_att_M,
                         float d_gain_att_M,
                         float p_gain_att_N,            //Yaw
                         float i_gain_att_N,
                         float d_gain_att_N,
                         float p_cte,
                         float d_cte) {

    m_p_gain = std::vector<float>{p_gain_pos_X, p_gain_pos_Y, p_gain_pos_Z, p_gain_att_K, p_gain_att_M, p_gain_att_N};
    m_i_gain = std::vector<float>{i_gain_pos_X, i_gain_pos_Y, i_gain_pos_Z, i_gain_att_K, i_gain_att_M, i_gain_att_N};
    m_d_gain = std::vector<float>{d_gain_pos_X, d_gain_pos_Y, d_gain_pos_Z, d_gain_att_K, d_gain_att_M, d_gain_att_N};
    m_cte_p_gain = p_cte;
    m_cte_d_gain = d_cte;
}

void CGuidance::SetGains(float p_pos,
                         float i_pos,
                         float d_pos,
                         float p_att,
                         float i_att,
                         float d_att,
                         float p_cte,
                         float d_cte) {

    m_p_gain = std::vector<float>{p_pos, p_pos, p_pos, p_att, p_att, p_att};
    m_i_gain = std::vector<float>{i_pos, i_pos, i_pos, i_att, i_att, i_att};
    m_d_gain = std::vector<float>{d_pos, d_pos, d_pos, d_att, d_att, d_att};
    m_cte_p_gain = p_cte;
    m_cte_d_gain = d_cte;
}

void CGuidance::SetForgetFactor(float i_pos_forget_X,
                                float i_pos_forget_Y,
                                float i_pos_forget_Z,
                                float i_att_forget_K,
                                float i_att_forget_M,
                                float i_att_forget_N) {

    m_i_forget_factor = std::vector<float>{i_pos_forget_X, i_pos_forget_Y, i_pos_forget_Z,
                                           i_att_forget_K, i_att_forget_M, i_att_forget_N}; 
}
void CGuidance::SetForgetFactor(float i_pos_forget,
                                float i_att_forget) {

    m_i_forget_factor = std::vector<float>{i_pos_forget, i_pos_forget, i_pos_forget,
                                           i_att_forget, i_att_forget, i_att_forget};
}

std::vector<float> CGuidance::ProcessGuidance(float time,
                                              std::vector<float> curr_pos,
                                              std::vector<float> curr_att,
                                              std::vector<float> des_pos,
                                              bool LOS_ctrl,
                                              std::vector<float> des_att,
                                              std::vector<float> prev_wpt,
                                              bool cross_track_ctrl) {

    //6DoF Motion
    std::vector<float> output(6, 0.0f);

    float time_diff = time - m_previous_time;
    std::vector<float> position_error(3,0.0f);
    std::vector<float> attitude_error(3,0.0f);
    std::transform(des_pos.begin(),
                   des_pos.end(),
                   curr_pos.begin(),
                   position_error.begin(),
                   std::minus<float>());

    if(LOS_ctrl) {
        des_att = std::vector<float>{0.0f,  //roll
                                     0.0f,  //pitch
                                     GetLeastErrorDiff(atan2(position_error[0], position_error[1]))};
    }

    std::transform(des_att.begin(),
                   des_att.end(),
                   curr_att.begin(),
                   attitude_error.begin(),
                   [&](float des, float curr){return GetLeastErrorDiff(des-curr);});

    std::vector<float> error;
    error.insert(error.begin(), position_error.begin(), position_error.end());
    error.insert(error.end(), attitude_error.begin(), attitude_error.end());

    // P error : position_error, attitude_error


    std::vector<float> position_error_diff(3, 0.0f);
    std::vector<float> attitude_error_diff(3, 0.0f);

    std::transform(position_error.begin(),
                   position_error.end(),
                   m_prev_error.begin(),
                   position_error_diff.begin(),
                   std::minus<float>());

    std::transform(attitude_error.begin(),
                   attitude_error.end(),
                   m_prev_error.begin()+3,
                   attitude_error_diff.begin(),
                   [&](float des, float curr){return GetLeastErrorDiff(des-curr);});

    std::vector<float> error_diff;
    error_diff.insert(error_diff.begin(), position_error_diff.begin(), position_error_diff.end());
    error_diff.insert(error_diff.end(), attitude_error_diff.begin(), attitude_error_diff.end());

    //Incremental

    for(size_t i = 0; i< m_incremental_error.size(); ++i) {
        if(i<3) {
            m_incremental_error[i] = m_i_forget_factor[i]*m_incremental_error[i] + position_error[i];
        }else {
            m_incremental_error[i] = m_i_forget_factor[i]*m_incremental_error[i] + attitude_error[i-3];
        }
    }

    for(size_t i = 0; i < output.size(); ++i)  {
        output[i] = m_p_gain[i] * error[i] +
                    m_i_gain[i] * m_incremental_error[i]+
                    m_d_gain[i] * error_diff[i]/time_diff;
    }

    m_prev_error = error;
    m_previous_time = time;


    if(cross_track_ctrl) {
        std::vector<float> cte_error(3, 0.0f);
        //length of prev_wpt ~ curr_wpt
        std::vector<float> prev_wpt_to_curr_wpt(3,0.0f);
        std::vector<float> prev_wpt_to_curr_pos(3,0.0f);

        for(size_t i = 0; i < 3; ++i) {
            prev_wpt_to_curr_wpt[i] = des_pos[i] - prev_wpt[i];
            prev_wpt_to_curr_pos[i] = curr_pos[i] - prev_wpt[i];
        }

        float length = GetNormOfVector(prev_wpt_to_curr_wpt);
        
        float dot_product = 0;
        for(size_t i = 0; i < 3; ++i) {
            dot_product += prev_wpt_to_curr_pos[i] * prev_wpt_to_curr_wpt[i];
        }
        for(size_t i = 0; i < 3; ++i) {
            cte_error[i] = dot_product * prev_wpt_to_curr_wpt[i] / (length * length);
        }

        std::vector<float> cte_error_diff(3,0.0f);
        std::transform(cte_error.begin(),
                       cte_error.end(),
                       m_prev_cte_error.begin(),
                       cte_error_diff.begin(),
                       std::minus<float>());        

        for(size_t i = 0; i < 3; ++i)  {
            output[i] += m_cte_p_gain * cte_error[i] +
                         m_cte_d_gain * cte_error_diff[i]/time_diff;
        }

        m_prev_cte_error = cte_error;
        
    } else {

    }
    return output;
}

float CGuidance::GetLeastErrorDiff(float radian) {
    if(std::fabs(radian) > M_PI) {
        int rotation = floor(radian/(2.0f*M_PI)+0.5);
        return radian - (2.0f*M_PI)*rotation;
    } else {
        return radian;
    }
}

void CGuidance::ResetIncrementalError() {
    m_incremental_error = std::vector<float>(6, 0.0f);
}

float CGuidance::GetNormOfVector(std::vector<float> vec) {
    float norm = 0;
    for(size_t i = 0; i < vec.size(); ++i) {
        norm += vec[i] * vec[i];
    }
    return sqrtf(norm);
}