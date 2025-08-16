package frc.robot.systems;

import frc.robot.TeleopInput;

public abstract class FSMSystem<W extends Enum<W>, S extends Enum<S>> {

    /**
     * The wanted state of the FSM is the state that the FSM is trying to reach.
     * This is the state that the FSM needs to be checking to advance to in {@code advanceState(TeleopInput input)}.
     */
    protected W wantedState;

    /**
     * The system state of the FSM is the current state of the FSM.
     * This is the state that the FSM is currently in and will be used to determine what
     * actions to take in {@code handleStates(TeleopInput input)}.
     */
    protected S systemState;

    /**
     * Create FSMSystem and initialize to starting state. Also perform any
     * one-time initialization or configuration of hardware required. Note
     * the constructor is called only once when the robot boots.
     */
    public FSMSystem() {
        reset();
    }

    /**
     * Set the wanted state of the FSM.
     * @param wantedState the state that the FSM is trying to reach
     */
    public void requestWantedState(W wantedState) {
        this.wantedState = wantedState;
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

    /**
     * Update method to be called every periodic of the robot. Make sure to add this into {@code Robot.java}
     * @param input Global TeleopInput if robot in teleop mode or null if the robot is in autonomous mode.
     */
    abstract public void update(TeleopInput input);

    /**
     * Updates the logging information for the system.
     */
    public void updateLogging() { }

    /**
     * Advance the FSM to the next state and store the previous state.
     *
     * This is a function of the inputs and the current state of this FSM. This
     * method should not have any side effects on outputs. In other words, this
     * method should only read or get values to decide what state to go to.
     * @param input Global TeleopInput if robot in teleop mode or null if
     *        the robot is in autonomous mode.
     */
    abstract protected S advanceState(TeleopInput input);

    /**
     * The state handler for the FSM. This method needs to be called in the update method So that the states
     * are handled correctly.
     * @param input Global TeleopInput if robot in teleop mode or null if not instantiated
     */
    abstract protected void handleStates(TeleopInput input);
}
