#include "ValorAuto.h"

ValorAuto::ValorAuto() {
}

frc2::Command* ValorAuto::getCurrentAuto() {
    std::string auto_name = "Wait";
    auto selected_trajectories = autos[auto_name];
    return selected_trajectories;
}