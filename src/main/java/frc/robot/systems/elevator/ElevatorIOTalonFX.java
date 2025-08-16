package frc.robot.systems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
    // The one and only...
    private final TalonFX talon;

    // Motion Requests
    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final VelocityDutyCycle velocityOut = new VelocityDutyCycle(0.0)
            .withUpdateFreqHz(0.0)
            .withFeedForward(0.0);

    // Signals to read from the motor
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> temperature;

    public ElevatorIOTalonFX(int id, String canBus) {
        talon = new TalonFX(id, canBus);

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        temperature = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                velocity,
                appliedVolts,
                temperature
        );

        PhoenixUtil.registerSignals(
                true,
                position,
                velocity,
                appliedVolts,
                temperature
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.data = new ElevatorIOData(
                BaseStatusSignal.isAllGood(position, velocity, appliedVolts, temperature),
                Units.rotationsToRadians(position.getValueAsDouble()),
                Units.rotationsToRadians(velocity.getValueAsDouble()),
                appliedVolts.getValueAsDouble(), // volts
                temperature.getValueAsDouble()
        );
    }

    @Override
    public void runVolts(Voltage volts) {
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        talon.stopMotor();
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
        talon.setControl(
            motionMagicVoltage.withPosition(positionRad).withFeedForward(feedforward)
        );
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void runVelocity(double velocity) {

    }
}
