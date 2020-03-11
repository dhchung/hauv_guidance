#include "hauv_guidance.h"

int main() {

    CGuidance cg;
    cg.ProcessGuidance(0,
                       std::vector<float>(3,0),
                       std::vector<float>(3,0),
                       std::vector<float>(3,0),
                       false,
                       std::vector<float>{0, 0, -370*M_PI/180.0f});
    return 0;
}