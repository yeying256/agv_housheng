#ifndef __PARAM_H
#define __PARAM_H
#include <unordered_map>
#include <vector>
namespace hfsm_ns
{

struct pose
{
    float bias_x;
    float bias_y;
    float theta;
    float err[3];
};

struct workbench
{
    std::vector<pose> p;
    int left_ar;
    int right_ar;
};

// pose: 
//   x: 1.3025910252302508
//   y: 0.010416443354787438
//   theta: 0.006375339095745183

std::unordered_map<int8_t, workbench> retUmap();


void printf_yellow(const char *s);


void printf_green(const char *s);


}


#endif
