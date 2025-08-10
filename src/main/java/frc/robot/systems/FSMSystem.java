package frc.robot.systems;

import frc.robot.TeleopInput;

public abstract class FSMSystem<W, C> {

	/* ======================== Constants ======================== */

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	protected W wantedState;
	protected C currentState;

	/* ======================== Constructor ======================== */

	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		reset();
	}

	/* ======================== Public methods ======================== */

	/**
	 * Get the current FSM state.
	 * @return current FSM state
	 */
    public C getCurrentState() {
        return currentState;
    }

	/**
	 * Get the previous FSM state.
	 * @return previous FSM state
	 */
	public W getWantedState() {
        return wantedState;
    }

	/**
	 * Reset this system to its start state and call one tick of update.
	 * This may be called from mode init when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	abstract public void reset();

    abstract public void update(TeleopInput input);
	/**
	 * Updates the logging information for the system.
	 */
	public void updateLogging() { }

	/* ======================== Private methods ======================== */
	/**
 	 * Advance the FSM to the next state and store the previous state.
	 *
	 * This is a function of the inputs and the current state of this FSM. This
	 * method should not have any side effects on outputs. In other words, this
	 * method should only read or get values to decide what state to go to.
 	 * @param input Global TeleopInput if robot in teleop mode or null if
 	 *        the robot is in autonomous mode.
 	 */
	protected abstract C advanceState(TeleopInput input);

    protected abstract void handleStates(TeleopInput input);

	/* ------------------------ FSM state handlers ------------------------ */


	/* ---- Autonomous Commands ---- */

}
