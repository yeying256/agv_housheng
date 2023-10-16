#include <unordered_map>

struct workbench
{
    float bias_x;
    float bias_y;
    float dist;
};

std::unordered_map<int8_t, workbench> retUmap()
{
    workbench w1{0,0,1};
    workbench w2{1,0,1};
    workbench w3{0,1,2};
    std::unordered_map<int8_t, workbench> tempUmap{
        {1, w1},
        {2, w2},
        {3, w3}};
    return tempUmap;
}
