package frc.robot.systems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

 public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();



    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void update() {

    }

    public void getInputs() {
    }
}
