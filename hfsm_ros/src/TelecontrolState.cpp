#include "hfsm/TelecontrolState.h"

namespace hfsm_ns{
    void TelecontrolState::start()
    {
        printf_yellow("TelecontrolState start");
    }

    void TelecontrolState::stop()
    {
        printf_green("TelecontrolState stop");
    }

    void TelecontrolState::update()
    {
        TransState("Auto");
    }
}