package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto {
    private SequentialCommandGroup cmdGrp;

    public Auto(Command... commands) {
        cmdGrp = new SequentialCommandGroup(commands);
    }

    public SequentialCommandGroup getCmdGrp() {
        return cmdGrp;
    }
}
