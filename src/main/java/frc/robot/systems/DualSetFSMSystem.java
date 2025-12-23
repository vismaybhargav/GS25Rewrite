package frc.robot.systems;

public abstract class DualSetFSMSystem<S extends Enum<S>, W extends Enum<W>> extends SingleSetFSMSystem<S> {
	private W wantedState;

	/**
	 * Request a wanted state for the system. Not guaranteed to change to this state immediately.
	 * @param state the wanted state to request
	 */
	public void requestWantedState(W state) {
		wantedState = state;
	}

	/**
	 * Get the current wanted state of this FSM system.
	 * @return current wanted state
	 */
	public W getWantedState() {
		return wantedState;
	}
}
