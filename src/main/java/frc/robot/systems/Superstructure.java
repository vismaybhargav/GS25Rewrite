package frc.robot.systems;

import frc.robot.TeleopInput;
import frc.robot.systems.drivetrain.Drivetrain;
import frc.robot.systems.elevator.Elevator;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends FSMSystem<Superstructure.SuperWantedState, Superstructure.SuperSystemState> {
    public enum SuperWantedState {
        IDLE
    }

    public enum SuperSystemState {
        IDLE
    }

    private Elevator elevator;
    private Drivetrain drivetrain;

    public Superstructure(Drivetrain drivetrain, Elevator elevator) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
    }

    @Override
    public void reset() {
        systemState = SuperSystemState.IDLE;
        wantedState = SuperWantedState.IDLE;
    }

    @Override
    public void update(TeleopInput input) {
        handleStates(input);
        systemState = advanceState(input);
    }

    @Override
    protected SuperSystemState advanceState(TeleopInput input) {
        if(input == null) {
            return SuperSystemState.IDLE;
        }

        switch (wantedState) {
            case IDLE:
                return SuperSystemState.IDLE;
            default:
                throw new IllegalStateException("Unexpected value: " + wantedState);
        }
    }

    @Override
    protected void handleStates(TeleopInput input) {
        switch (systemState) {
            case IDLE:
                handleIdleState(input);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + systemState);
        }
    }

    private void handleIdleState(TeleopInput input) {
        elevator.requestWantedState(Elevator.ElevatorWantedState.MANUAL);
    }

    @AutoLogOutput(key = "Superstructure/Wanted State")
    public SuperWantedState getWantedState() {
        return wantedState;
    }

    @AutoLogOutput(key = "Superstructure/System State")
    public SuperSystemState getSystemState() {
        return systemState;
    }

}
