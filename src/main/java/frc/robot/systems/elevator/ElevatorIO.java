package frc.robot.systems.elevator;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public ElevatorIOData data = new ElevatorIOData(
                false, // motorConnected
                0.0,   // positionRad
                0.0,   // velocityRadPerSec
                0.0,   // appliedVoltage
                0.0    // temperatureCelsius
        );
    }

    record ElevatorIOData(
            boolean motorConnected,
            double positionRad,
            double velocityRadPerSec,
            double appliedVoltage,
            double temperatureCelsius
    ) {}

    void updateInputs(ElevatorIOInputs inputs);

    void runVolts(Voltage volts);

    void stop();

    void runPosition(double positionRad, double feedforward);

     void setBrakeMode(boolean enabled);

     void runVelocity(double velocity);
}
