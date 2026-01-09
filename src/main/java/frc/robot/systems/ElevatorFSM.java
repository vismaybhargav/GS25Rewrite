package frc.robot.systems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.TeleopInput;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorFSM extends DualSetFSMSystem<ElevatorFSM.ElevatorSystemState, ElevatorFSM.ElevatorWantedState> {
	public enum ElevatorSystemState {
		MANUAL,
		GOING_GROUND,
		GOING_L2,
		GOING_L3,
		GOING_L4
	}

	public enum ElevatorWantedState {
		MANUAL(Meters.of(0)),
		GO_GROUND(Inches.of(0)),
		GO_L2(Inches.of(7.6)),
		GO_L3(Inches.of(19.1)),
		GO_L4(Inches.of(36));

		private Distance setpoint;

		/**
		 * Constructor for ElevatorWantedState.
		 * @param theSetpoint The setpoint for the wanted state.
		 */
		ElevatorWantedState(Distance theSetpoint) {
			setpoint = theSetpoint;
		}


		/**
		 * Gets the setpoint for the wanted state.
		 * @return The setpoint distance.
		 */
		public Distance getSetpoint() {
			return setpoint;
		}
	}

	private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
		.withControlMode(ControlMode.CLOSED_LOOP)
		// Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
		.withMechanismCircumference(Inches.of(1.45).div(2).times(Math.PI).times(2))
		// Feedback Constants (PID Constants)
		.withClosedLoopController(3, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
		.withSimClosedLoopController(3, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
		// Feedforward Constants
		.withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.001))
		.withSimFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.001))
		.withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
		// Gearing from the motor rotor to final shaft.
		.withGearing(new MechanismGearing(25))
		.withMotorInverted(false)
		.withIdleMode(MotorMode.BRAKE)
		.withStatorCurrentLimit(Amps.of(40))
		.withClosedLoopRampRate(Seconds.of(0.25))
		.withOpenLoopRampRate(Seconds.of(0.25));

	private TalonFX elevatorMotor = new TalonFX(1, "canbus");

	private SmartMotorController elevatorMotorController = new TalonFXWrapper(elevatorMotor, DCMotor.getKrakenX60(1), motorConfig);

	private ElevatorConfig elevatorConfig = new ElevatorConfig(elevatorMotorController)
		.withStartingHeight(Meters.of(0))
		.withHardLimits(Meters.of(0), Inches.of(37.3))
		.withTelemetry("Elevator", TelemetryVerbosity.HIGH)
		.withMass(Pounds.of(10.056));

	private Elevator elevator = new Elevator(elevatorConfig);

	/**
	 * Constructor for Elevator system.
	 */
	public ElevatorFSM() {
		elevator.setHeight(ElevatorWantedState.GO_L2.getSetpoint());

		reset();
	}

	@Override
	public void reset() {
		setSystemState(ElevatorSystemState.MANUAL);
		requestWantedState(ElevatorWantedState.MANUAL);

		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		Logger.recordOutput("Elevator/Wanted State", getWantedState());
		Logger.recordOutput("Elevator/Current State", getSystemState());

		elevator.updateTelemetry();

		if (Robot.isSimulation()) {
			elevator.simIterate();
		}

		switch (getSystemState()) {
			case MANUAL:
				handleManualState(input);
				break;
			case GOING_GROUND:
			case GOING_L2:
			case GOING_L3:
			case GOING_L4:
				handleSetpointState();
				break;
			default:
				throw new IllegalStateException("[ELEVATOR] Invalid Current State: " + getSystemState().toString());
		}
	}

	@Override
	protected ElevatorSystemState nextState(TeleopInput input) {
		if (input == null) {
			return ElevatorSystemState.MANUAL;
		}

		switch (getWantedState()) {
			case MANUAL:
				return ElevatorSystemState.MANUAL;
			case GO_GROUND:
				return ElevatorSystemState.GOING_GROUND;
			case GO_L2:
				return ElevatorSystemState.GOING_L2;
			case GO_L3:
				return ElevatorSystemState.GOING_L3;
			case GO_L4:
				return ElevatorSystemState.GOING_L4;
			default:
				throw new IllegalStateException("[ELEVATOR] Invalid Wanted State: " + getWantedState().toString());
		}
	}


	/**
	 * Handles the setpoint state.
	 */
	private void handleSetpointState() {
		if (!(getWantedState() == ElevatorWantedState.MANUAL)) {
			throw new IllegalStateException("[ELEVATOR] Wanted State: " + getWantedState() + " does not match Setpoint State handler.");
		}

		elevator.setHeight(getWantedState().getSetpoint());
	}

	private void handleManualState(TeleopInput input) {
		// NOTHING YET
	}
}
