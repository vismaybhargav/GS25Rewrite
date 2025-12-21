// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.Constants.VisionConstants.REEF_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_REEF_CAM;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_STATION_CAM;
import static frc.robot.Constants.VisionConstants.STATION_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.TAG_ID_TEST_REEF_LEFT;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
// WPILib Imports
import frc.robot.systems.DriveFSMSystem;

// Systems

// Local
import frc.robot.vision.Vision;
import frc.robot.vision.VisionIOLimelight;
import frc.robot.vision.VisionIOPhotonPoseEstimator;
import frc.robot.vision.VisionIOPhotonPoseEstimatorSim;
import frc.robot.vision.VisionIOPhotonVision;
import frc.robot.vision.VisionIOPhotonVisionSim;
import frc.robot.vision.VisionIOYALL;
import limelight.networktables.Orientation3d;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveSystem;
	private Vision vision;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		Logger.recordMetadata("GS25 Rewrite", "FSM BASED");

		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter());
			Logger.addDataReceiver(new NT4Publisher());
		} else if (isSimulation()) {
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			setUseTiming(false);
			var logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}

		Logger.start(); // Start Logging!


		input = new TeleopInput();

		// Instantiate all systems here
		driveSystem = new DriveFSMSystem();

		if (isReal()) {
			if (Features.USE_LIMELIGHT) {
				if (Features.USE_YALL) {
					vision = new Vision(
						driveSystem::addVisionMeasurement,
						() -> driveSystem.getPose().getRotation(),
						new VisionIOYALL("limelight-four",
							() -> {
								var pigeon = driveSystem.getDrivetrain().getPigeon2();
								return new Orientation3d(
									pigeon.getRotation3d(),
									pigeon.getAngularVelocityZDevice().getValue(),
									pigeon.getAngularVelocityYDevice().getValue(),
									pigeon.getAngularVelocityXDevice().getValue()
								);
							}
						));
				} else {
					vision = new Vision(
						driveSystem::addVisionMeasurement,
						() -> driveSystem.getPose().getRotation(),
						new VisionIOLimelight("limelight-four", () -> driveSystem.getDrivetrain().getPigeon2().getRotation2d())
					);
				}
			} else {
				if (Features.PHOTON_POSE_ESTIMATOR_ENABLED) {
					vision = new Vision(
						driveSystem::addVisionMeasurement,
						() -> driveSystem.getPose().getRotation(),
						new VisionIOPhotonPoseEstimator(REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM),
						new VisionIOPhotonPoseEstimator(STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM));
				} else {
					vision = new Vision(
						driveSystem::addVisionMeasurement,
						() -> driveSystem.getPose().getRotation(),
						new VisionIOPhotonVision(REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM),
						new VisionIOPhotonVision(STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM));
				}
			}
		} else {
			if (Features.PHOTON_POSE_ESTIMATOR_ENABLED) {
				vision = new Vision(
						driveSystem::addVisionMeasurement,
						() -> driveSystem.getPose().getRotation(),
						new VisionIOPhotonPoseEstimatorSim(
								REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM, driveSystem::getPose),
						new VisionIOPhotonPoseEstimatorSim(
								STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM, driveSystem::getPose));
			} else {
				vision = new Vision(
						driveSystem::addVisionMeasurement,
						() -> driveSystem.getPose().getRotation(),
						new VisionIOPhotonVisionSim(
								REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM, driveSystem::getPose),
						new VisionIOPhotonVisionSim(
								STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM, driveSystem::getPose));
			}
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveSystem.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
		if (Features.MAPLE_SIM_ENABLED) {
			SimulatedArena.getInstance().resetFieldForAuto();
		}
	}

	@Override
	public void simulationPeriodic() {
		if (!Features.MAPLE_SIM_ENABLED) {
			return;
		}

		driveSystem.getSimDrivetrain().update();
		Logger.recordOutput(
			"Field Simulation/Game Pieces/Coral",
			SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
		);

		Logger.recordOutput(
			"Field Simulation/Game Pieces/Algae",
			SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
		);
	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() {
		vision.periodic();

		Logger.recordOutput("Field Bounding Box",
		new Pose2d[] {
			new Pose2d(0, 0, new Rotation2d()),
			new Pose2d(0, TAG_LAYOUT.getFieldLength(), new Rotation2d()),
			new Pose2d(TAG_LAYOUT.getFieldWidth(), TAG_LAYOUT.getFieldLength(), new Rotation2d()),
			new Pose2d(TAG_LAYOUT.getFieldWidth(), 0, new Rotation2d()),
		} 
		);
	}
}
