#include <hfsm/ShutdownState.h>

namespace hfsm_ns
{
    void ShutdownState::start()
    {
        printf_yellow("ShutdownState start");
        system("rosnode kill /map_server");
        this->_context->run_ = false;
    }
}
