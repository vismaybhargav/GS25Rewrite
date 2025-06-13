package frc.robot.systems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.Constants;
import frc.robot.HardwareMap;

/**
 * Elevator IO implementation using a TalonFX motor controller.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
	private TalonFX elevatorMotor;

	/**
	 * Creates an ElevatorIOTalonFX object and initializes motor.
	 */
	public ElevatorIOTalonFX() {
		elevatorMotor = new TalonFX(HardwareMap.CAN_ID_KRAKEN_ELEVATOR);

		motionRequest = new MotionMagicVoltage(0);

		var talonFXConfigs = new TalonFXConfiguration();

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		// apply sw limit
		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true; // enable top limit
		swLimitSwitch.ReverseSoftLimitEnable = true; // enable bottom limit
		swLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_UPPER_THRESHOLD
			.in(Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = Constants.ELEVATOR_ROTS_TO_INCHES;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
		slot0Configs.kG = Constants.ELEVATOR_KG;
		slot0Configs.kS = Constants.ELEVATOR_KS;
		slot0Configs.kV = Constants.ELEVATOR_KV;
		slot0Configs.kA = Constants.ELEVATOR_KA;
		slot0Configs.kP = Constants.ELEVATOR_KP;
		slot0Configs.kI = Constants.ELEVATOR_KI;
		slot0Configs.kD = Constants.ELEVATOR_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ELEVATOR_CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = Constants.ELEVATOR_TARGET_ACCEL;
		motionMagicConfigs.MotionMagicExpo_kV = Constants.ELEVATOR_EXPO_KV;

		elevatorMotor.getConfigurator().apply(talonFXConfigs);
	}
}
