package frc.robot.systems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public ElevatorIOData data = new ElevatorIOData(
                false, // motorConnected
                0.0,   // positionRad
                0.0,   // velocityRadPerSec
                0.0,   // appliedVoltage
                0.0,   // positionMeters
                0.0,   // velocityMetersPerSec
                0.0    // temperatureCelsius
        );
    }

    record ElevatorIOData(
            boolean motorConnected,
            double positionRad,
            double velocityRadPerSec,
            double appliedVoltage,
            double positionMeters,
            double velocityMetersPerSec,
            double temperatureCelsius
    ) {}

    void updateInputs(ElevatorIOInputs inputs);

    void runVolts(double volts);

    void stop();

    void runPosition(double positionRad, double feedforward);

     void setBrakeMode(boolean enabled);
}
