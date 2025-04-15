package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.CommandSwerveDrivetrain;

// WPILib Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DriveConstants;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum DriveFSMState {
		TELEOP_STATE
	}

	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_AT_12_VOLTS;
		// kSpeedAt12Volts desired top speed
	private static final AngularVelocity MAX_ANGULAR_RATE = DriveConstants.MAX_ANGULAR_VELO_RPS;
		//3/4 rps angle velo

	/* ======================== Private variables ======================== */
	private DriveFSMState currentState;

	private CommandSwerveDrivetrain drivetrain;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		drivetrain = TunerConstants.createDrivetrain();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public DriveFSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = DriveFSMState.TELEOP_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				handleTeleopState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 */
	public void updateAutonomous() { }

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private DriveFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				return DriveFSMState.TELEOP_STATE;
			default:
				throw new IllegalStateException("Invalid State: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handles the TELEOP_STATE, when the robot is controlled by the driver.
	 * @param input the input of the driver controller
	 */
	public void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}

		double xSpeed = MathUtil.applyDeadband(
			input.getDriveLeftJoystickY(),
			DriveConstants.TRANSLATION_DEADBAND
		) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
			input.getDriveLeftJoystickX(),
			DriveConstants.TRANSLATION_DEADBAND
		) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
			input.getDriveRightJoystickX(),
			DriveConstants.ROTATION_DEADBAND
		) * MAX_ANGULAR_RATE.in(RadiansPerSecond);

		
	}
}
