package frc.robot.systems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> temperature;


    public ElevatorIOTalonFX(int id, String canBus) {
        motor = new TalonFX(id, canBus);

        // Configure the motor
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        var posRad = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        var velRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());

        inputs.data = new ElevatorIOData(
                BaseStatusSignal.isAllGood(),
                posRad,
                velRadPerSec,
        );
    }

    @Override
    public void runVolts(double volts) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void runPosition(double positionRad, double feedforward) {

    }

    @Override
    public void setBrakeMode(boolean enabled) {

    }
}
