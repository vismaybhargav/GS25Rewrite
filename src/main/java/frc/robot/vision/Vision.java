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
import frc.robot.Constants.VisionConstants;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.VisionConstants.ANGULAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.CAMERA_STD_DEV_FACTORS;
import static frc.robot.Constants.VisionConstants.LINEAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.MAX_Z_ERROR;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;
import static frc.robot.Constants.VisionConstants.FIELD_BORDER_MARGIN;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private final VisionConsumer visionConsumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;
	private final Supplier<Rotation2d> rotatonSupplier;

	/**
	 * Creates a new Vision subsystem.
	 * @param consumer The consumer to accept vision observations.
	 * @param rotationSupp The supplier for the robot's rotation.
	 * @param iO The IO objects to use for the cameras.
	 */
	public Vision(VisionConsumer consumer, Supplier<Rotation2d> rotationSupp, VisionIO... iO) {
		this.visionConsumer = consumer;
		this.rotatonSupplier = rotationSupp;
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
		return inputs[cameraIndex].latestTargetObservation.tx();
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

		List<Pose3d> tagPoses = new LinkedList<>();
		List<Pose3d> robotPoses = new LinkedList<>();
		List<Pose3d> robotPosesAccepted = new LinkedList<>();
		List<Pose3d> robotPosesRejected = new LinkedList<>();


		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			tagPoses.clear();
			robotPoses.clear();
			robotPosesAccepted.clear();
			robotPosesRejected.clear();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = TAG_LAYOUT.getTagPose(tagId);
				tagPose.ifPresent(tagPoses::add);
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
						|| (observation.tagCount() == 1
								// Cannot be high ambiguity
								&& observation.ambiguity() > MAX_AMBIGUITY
								&& Math.abs(
									rotatonSupplier
										.get()
										.minus(observation.pose().toPose2d().getRotation())
										.getRadians()) > VisionConstants.MAX_POSE_ROT_OFFSET.in(Radians))
						// Must have realistic Z coordinate
						|| Math.abs(observation.pose().getZ()) > MAX_Z_ERROR

						// Must be within the field boundaries
						|| observation.pose().getX() < -FIELD_BORDER_MARGIN
						|| observation.pose().getX() > TAG_LAYOUT.getFieldLength() + FIELD_BORDER_MARGIN
						|| observation.pose().getY() < -FIELD_BORDER_MARGIN
						|| observation.pose().getY() > TAG_LAYOUT.getFieldWidth() +  FIELD_BORDER_MARGIN;
				
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

				//attempt 1 at dynamic shit
				double totalDistanceX = 0.0;
				double totalDistanceY = 0.0;
				int contributingTags = 0;

				for (int tagId : inputs[cameraIndex].tagIds) {
					var tagPoseOpt = TAG_LAYOUT.getTagPose(tagId);
					if (tagPoseOpt.isPresent()) {
						var tagPose = tagPoseOpt.get();
						totalDistanceX += Math.abs(observation.pose().getX() - tagPose.getX());
						totalDistanceY += Math.abs(observation.pose().getY() - tagPose.getY());
						contributingTags++;
					}
				}

				// If no tag pose found, fall back to average distance
				double distanceX = (contributingTags > 0)
						? totalDistanceX / contributingTags
						: observation.averageTagDistance();
				double distanceY = (contributingTags > 0)
						? totalDistanceY / contributingTags
						: observation.averageTagDistance();

				// Scale uncertainties
				double tagFactor = Math.sqrt(Math.max(1, observation.tagCount()));

				double linearStdDevX = LINEAR_STD_DEV_BASELINE * (distanceX * distanceX) / tagFactor;
				double linearStdDevY = LINEAR_STD_DEV_BASELINE * (distanceY * distanceY) / tagFactor;
				double angularStdDev = ANGULAR_STD_DEV_BASELINE * (observation.averageTagDistance() / tagFactor);

				// Apply per-camera tuning factors
				if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
					linearStdDevX *= CAMERA_STD_DEV_FACTORS[cameraIndex];
					linearStdDevY *= CAMERA_STD_DEV_FACTORS[cameraIndex];
					angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
				}

				// Send vision observation
				visionConsumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDevX, linearStdDevY, angularStdDev));
			
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
