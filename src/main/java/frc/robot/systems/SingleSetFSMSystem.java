package frc.robot.systems;

import frc.robot.TeleopInput;

public abstract class SingleSetFSMSystem<S extends Enum<S>> {
	private S systemState;

	/**
	 * Request a wanted state for the system. Not guaranteed to change to this state immediately.
	 * @param state the wanted state to request
	 */
	protected void setSystemState(S state) {
		systemState = state;
	}

	/**
	 * Get the current state of this FSM system.
	 * @return current state
	 */
	public S getSystemState() {
		return systemState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public abstract void reset();

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public abstract void update(TeleopInput input);

	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	protected abstract S nextState(TeleopInput input);
}
