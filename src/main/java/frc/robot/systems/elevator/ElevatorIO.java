package frc.robot.systems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public ElevatorIOData data = new ElevatorIOData(
                false, // motorConnected
                0.0,   // positionRad
                0.0,   // velocityRadPerSec
                0.0    // appliedVoltage
        );
    }

    record ElevatorIOData(
            boolean motorConnected,
            double positionRad,
            double velocityRadPerSec,
            double appliedVoltage
    ) {}

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void stop() {}

    default void runPosition(double positionRad, double feedforward) {}

    default void setBrakeMode(boolean enabled) {}
}
