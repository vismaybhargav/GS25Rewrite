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

package frc.robot.vision;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.ANGULAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.CAMERA_STD_DEV_FACTORS;
import static frc.robot.Constants.VisionConstants.LINEAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.MAX_Z_ERROR;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private final VisionConsumer visionConsumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	/**
	 * Creates a new Vision subsystem.
	 * @param consumer The consumer to accept vision observations.
	 * @param iO The IO objects to use for the cameras.
	 */
	public Vision(VisionConsumer consumer, VisionIO... iO) {
		this.visionConsumer = consumer;
		this.io = iO;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert(
					"Vision camera "
					+ Integer.toString(i) + " is disconnected.", AlertType.kWarning);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 * @return the yaw of the target
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].getLatestTargetObservation().tx();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].getTagIds()) {
				var tagPose = TAG_LAYOUT.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].getPoseObservations()) {
				// Check whether to reject pose
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
						|| (observation.tagCount() == 1
								// Cannot be high ambiguity
								&& observation.ambiguity() > MAX_AMBIGUITY)
						// Must have realistic Z coordinate
						|| Math.abs(observation.pose().getZ()) > MAX_Z_ERROR

						// Must be within the field boundaries
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX() > TAG_LAYOUT.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > TAG_LAYOUT.getFieldWidth();

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double stdDevFactor =
					Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
				double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
				if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
					linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
					angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
				}

				// Send vision observation
				visionConsumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log camera datadata
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput(
				"Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPoses",
				allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
	}

	@FunctionalInterface
	public interface VisionConsumer {
		/**
		 * Accepts a vision observation.
		 * @param visionRobotPoseMeters The robot pose in meters.
		 * @param timestampSeconds The timestamp in seconds.
		 * @param visionMeasurementStdDevs The standard deviations of the vision
		 */
		void accept(
				Pose2d visionRobotPoseMeters,
				double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
