#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain* _drivetrain) :
        drivetrain(_drivetrain) {

    table = nt::NetworkTableInstance::GetDefault().GetTable("Auto");
    table->PutNumber("Current Auto", 0);

    frc2::SequentialCommandGroup *wait = new frc2::SequentialCommandGroup();
    wait->AddCommands(frc2::WaitCommand((units::second_t)1.5));
    autos["wait"] = wait;
}

frc2::Command* ValorAuto::getCurrentAuto() {

    int selectedAuto = table->GetNumber("Current Auto", 0);
    std::string auto_name = "wait";
    auto selected_trajectories = autos[auto_name];
    return selected_trajectories;
}