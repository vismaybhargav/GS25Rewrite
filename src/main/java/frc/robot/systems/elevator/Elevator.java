package frc.robot.systems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;
import frc.robot.TeleopInput;
import frc.robot.systems.FSMSystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends FSMSystem<Elevator.ElevatorWantedState, Elevator.ElevatorSystemState> {
    public enum ElevatorStage {
        GROUND (ElevatorConstants.ELEVATOR_TARGET_GROUND),
        L2     (ElevatorConstants.ELEVATOR_TARGET_L2),
        L3     (ElevatorConstants.ELEVATOR_TARGET_L3),
        L4     (ElevatorConstants.ELEVATOR_TARGET_l4);

        private final Distance elevatorHeight;

        ElevatorStage(Distance elevatorHeight) {
            this.elevatorHeight = elevatorHeight;
        }

        public Distance getHeight() {
            return elevatorHeight;
        }
    }

    public enum ElevatorWantedState {
        MANUAL,
        GO_TO_GROUND,
        GO_TO_L2,
        GO_TO_L3,
        GO_TO_L4,
    }

    public enum ElevatorSystemState {
        MANUAL,
        GOING_TO_GROUND,
        GOING_TO_L2,
        GOING_TO_L3,
        GOING_TO_L4,
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private DigitalInput groundLimitSwitch;

    public Elevator(ElevatorIO io) {
        super();
        this.io = io;
    }

    @Override
    public void update(TeleopInput input) {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        handleStates(input);

        systemState = advanceState(input);
    }

    @Override
    public void reset() {
        systemState = ElevatorSystemState.MANUAL;
        wantedState = ElevatorWantedState.MANUAL;
    }

    @Override
    protected ElevatorSystemState advanceState(TeleopInput input) {
        if(input == null) {
            return ElevatorSystemState.MANUAL;
        }

        switch (wantedState) {
            case MANUAL:
                if (input.isGroundButtonPressed()
                        && !isBottomLimitReached()
                        && !input.isL4ButtonPressed()
                        && !input.isL2ButtonPressed()
                        && !input.isL3ButtonPressed()) {
                    return ElevatorSystemState.GOING_TO_GROUND;
                }
                if (input.isL2ButtonPressed()
                        && !input.isL4ButtonPressed()
                        && !input.isGroundButtonPressed()
                        && !input.isL3ButtonPressed()) {
                    return ElevatorSystemState.GOING_TO_L2;
                }
                if (input.isL3ButtonPressed()
                        && !input.isL4ButtonPressed()
                        && !input.isGroundButtonPressed()
                        && !input.isL2ButtonPressed()) {
                    return ElevatorSystemState.GOING_TO_L3;
                }
                if (input.isL4ButtonPressed()
                        && !input.isGroundButtonPressed()
                        && !input.isL2ButtonPressed()
                        && !input.isL3ButtonPressed()) {
                    return ElevatorSystemState.GOING_TO_L4;
                }
                return ElevatorSystemState.MANUAL;
            case GO_TO_GROUND:
                return isBottomLimitReached() || inRangeOfStage(ElevatorStage.GROUND)
                        ? ElevatorSystemState.MANUAL
                        : ElevatorSystemState.GOING_TO_GROUND;
            case GO_TO_L2:
                return inRangeOfStage(ElevatorStage.L2)
                        ? ElevatorSystemState.MANUAL
                        : ElevatorSystemState.GOING_TO_L2;
            case GO_TO_L3:
                return inRangeOfStage(ElevatorStage.L3)
                        ? ElevatorSystemState.MANUAL
                        : ElevatorSystemState.GOING_TO_L3;
            case GO_TO_L4:
                return inRangeOfStage(ElevatorStage.L4)
                        ? ElevatorSystemState.MANUAL
                        : ElevatorSystemState.GOING_TO_L4;
            default:
                return ElevatorSystemState.MANUAL; // Fallback to manual state if no match
        }
    }

    private boolean inRangeOfStage(ElevatorStage elevatorStage) {

    }

    private boolean isBottomLimitReached() {
        if (Robot.isSimulation()) {
            return false;
        }

        return groundLimitSwitch.get();
    }

    @Override
    protected void handleStates(TeleopInput input) {
       switch (wantedState) {
           case MANUAL -> handleManualState(input);
           case GO_TO_GROUND, GO_TO_L2, GO_TO_L3, GO_TO_L4 -> handleStageState(wantedState);
       }
    }

    private void handleStageState(ElevatorWantedState wantedState) {
        switch(wantedState) {
            case: GO_TO_GROUND -> {
                
            }
        }
    }

    private void handleManualState(TeleopInput input) {

    }
}
