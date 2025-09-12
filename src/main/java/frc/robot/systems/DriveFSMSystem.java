// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TeleopInput;
import frc.robot.CommandSwerveDrivetrain;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class DriveFSMSystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    // FSM States
    public enum DriveState {
        TELEOP,
        PRE_PATHFIND,
        PATHFIND,
        FINAL_ALIGN
    }
    
    private DriveState currentState = DriveState.TELEOP;
    
    // Vision measurement consumer
    private Consumer<Pose2d> visionConsumer;
    
    public DriveFSMSystem() {
        // TODO: Initialize drivetrain and other components
        this.drivetrain = null; // This will need to be properly initialized
    }
    
    public void reset() {
        currentState = DriveState.TELEOP;
    }
    
    public void update(TeleopInput input) {
        // Update FSM based on input
        switch (currentState) {
            case TELEOP:
                if (input.isPathfindButtonPressed()) {
                    currentState = DriveState.PRE_PATHFIND;
                }
                break;
            case PRE_PATHFIND:
                if (input.isPathfindButtonPressed()) {
                    currentState = DriveState.PATHFIND;
                } else {
                    currentState = DriveState.TELEOP;
                }
                break;
            case PATHFIND:
                if (input.isPathfindButtonPressed()) {
                    currentState = DriveState.FINAL_ALIGN;
                } else {
                    currentState = DriveState.TELEOP;
                }
                break;
            case FINAL_ALIGN:
                if (isPathfindingFinished()) {
                    currentState = DriveState.TELEOP;
                }
                break;
        }
        
        // Log current state
        Logger.recordOutput("DriveFSM/CurrentState", currentState.toString());
    }
    
    private boolean isPathfindingFinished() {
        // TODO: Implement pathfinding completion logic
        return false;
    }
    
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        if (visionConsumer != null) {
            // TODO: Process vision measurement
        }
    }
    
    public Pose2d getPose() {
        // TODO: Return current robot pose
        return new Pose2d();
    }
}
