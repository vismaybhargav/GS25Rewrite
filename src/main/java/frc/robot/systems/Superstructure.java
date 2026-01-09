package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import frc.robot.TeleopInput;
import frc.robot.systems.DrivetrainFSM.DriveWantedState;
import frc.robot.systems.ElevatorFSM.ElevatorWantedState;

public class Superstructure extends SingleSetFSMSystem<Superstructure.SuperSystemState> {
	public enum SuperSystemState {
		IDLING,
		PATHFINDING,
		MANUAL_GROUND,
		MANUAL_L2,
		MANUAL_L3,
		MANUAL_L4,
	}

	private DrivetrainFSM drivetrain;
	private ElevatorFSM elevator;

	/**
	 * Constructor for Superstructure system.
	 * @param driveSystem The drivetrain subsystem
	 * @param elevatorSystem The elevator subsystem
	 */
	public Superstructure(
		DrivetrainFSM driveSystem,
		ElevatorFSM elevatorSystem
	) {
		drivetrain = driveSystem;
		elevator = elevatorSystem;

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
			case MANUAL_GROUND:
			case MANUAL_L4:
			case MANUAL_L3:
			case MANUAL_L2:
				handleManualSetpointState(input);
			default:
				throw new IllegalStateException("[SUPERSTRUCTURE] Invalid State: " + getSystemState().toString());
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

				if (input.isL4ButtonPressed()) {
					return SuperSystemState.MANUAL_L4;
				}

				if (input.isL3ButtonPressed()) {
					return SuperSystemState.MANUAL_L3;
				}

				if (input.isL2ButtonPressed()) {
					return SuperSystemState.MANUAL_L2;
				}

				if (input.isGroundButtonPressed()) {
					return SuperSystemState.MANUAL_GROUND;
				}


				return SuperSystemState.IDLING;
			case PATHFINDING:
				if (!input.isPathfindButtonPressed()) {
					return SuperSystemState.IDLING;
				}
				return SuperSystemState.PATHFINDING;

			case MANUAL_GROUND:
			default:
				throw new IllegalStateException("[SUPERSTRUCTURE] Invalid Next State: " + getSystemState().toString());
		}
	}

	private void handleIdleState(TeleopInput input) {
		drivetrain.requestWantedState(DriveWantedState.TELEOP);
		elevator.requestWantedState(ElevatorWantedState.MANUAL);
	}

	private void handlePathfindState(TeleopInput input) {
		drivetrain.requestWantedState(DriveWantedState.PATHFIND);
		elevator.requestWantedState(elevator.getWantedState());
	}

	private void handleManualSetpointState(TeleopInput input) {
		drivetrain.requestWantedState(drivetrain.getWantedState());

		switch (getSystemState()) {
			case MANUAL_GROUND:
				elevator.requestWantedState(ElevatorWantedState.GO_GROUND);
				break;
			case MANUAL_L2:
				elevator.requestWantedState(ElevatorWantedState.GO_L2);
				break;
			case MANUAL_L3:
				elevator.requestWantedState(ElevatorWantedState.GO_L3);
				break;
			case MANUAL_L4:
				elevator.requestWantedState(ElevatorWantedState.GO_L4);
				break;
			default:
				throw new IllegalStateException("[SUPERSTRUCTURE] Invalid Manual Setpoint State: " + getSystemState().toString());
		}
	}
}
