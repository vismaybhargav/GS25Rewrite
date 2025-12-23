package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import frc.robot.TeleopInput;
import frc.robot.systems.Drivetrain.DriveWantedState;

public class Superstructure extends SingleSetFSMSystem<Superstructure.SuperSystemState> {
	public enum SuperSystemState {
		IDLING,
		PATHFINDING
	}

	private Drivetrain drivetrain;

	/**
	 * Constructor for Superstructure system.
	 * @param driveSystem The drivetrain subsystem
	 */
	public Superstructure(
		Drivetrain driveSystem
	) {
		drivetrain = driveSystem;

		reset();
	}

	@Override
	public void reset() {
		setSystemState(SuperSystemState.IDLING);

		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		Logger.recordOutput("Superstructure/Current State", getSystemState());

		switch (getSystemState()) {
			case IDLING:
				handleIdleState(input);
				break;
			case PATHFINDING:
				handlePathfindState(input);
				break;
			default:
				throw new IllegalStateException("Invalid State: " + getSystemState().toString());
		}

		setSystemState(nextState(input));
	}

	@Override
	protected SuperSystemState nextState(TeleopInput input) {
		if (input == null) {
			return SuperSystemState.IDLING;
		}

		switch (getSystemState()) {
			case IDLING:
				if (input.isPathfindButtonPressed()) {
					return SuperSystemState.PATHFINDING;
				}
				return SuperSystemState.IDLING;
			case PATHFINDING:
				if (!input.isPathfindButtonPressed()) {
					return SuperSystemState.IDLING;
				}
				return SuperSystemState.PATHFINDING;
			default:
				throw new IllegalStateException("Invalid State: " + getSystemState().toString());
		}
	}

	private void handleIdleState(TeleopInput input) {
		drivetrain.requestWantedState(DriveWantedState.TELEOP);
	}

	private void handlePathfindState(TeleopInput input) {
		drivetrain.requestWantedState(DriveWantedState.PATHFIND);
	}
}
