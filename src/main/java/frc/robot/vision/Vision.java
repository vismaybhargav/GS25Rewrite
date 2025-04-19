package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;
import static frc.robot.Constants.VisionConstants.MAX_Z_ERROR;
import static frc.robot.Constants.VisionConstants.CAMERA_STD_DEV_FACTORS;
import static frc.robot.Constants.VisionConstants.LINEAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.ANGULAR_STD_DEV_BASELINE;

import frc.robot.generated.VisionIOInputsAutoLogged;

import java.util.LinkedList;
import java.util.List;

public class Vision extends SubsystemBase {
	private final VisionConsumer visionConsumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;

	/**
	 * Constructs the vision subsystem.
	 * @param visConsumer the vision consumer
	 * @param inputOutput list of all cameras/sources
	 */
	public Vision(VisionConsumer visConsumer, VisionIO... inputOutput) {
		visionConsumer = visConsumer;
		io = inputOutput;

		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < io.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}
	}

	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + i, inputs[i]);
		}

		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();

		for (int camIdx = 0; camIdx < io.length; camIdx++) {
			// Initialize logging values
			List<Pose3d> robotPosesAccepted = new LinkedList<>();

			for (VisionIO.PoseObservation poseObservation
				: inputs[camIdx].getLatestPoseObservations()) {
				boolean rejectPose = poseObservation.tagCount() == 0
						|| (poseObservation.tagCount() == 1
								&& poseObservation.ambiguity() > MAX_AMBIGUITY)
						|| Math.abs(poseObservation.pose().getZ()) > MAX_Z_ERROR
						|| poseObservation.pose().getX() < 0.0
						|| poseObservation.pose().getY() < 0.0
						|| poseObservation.pose().getX() > TAG_LAYOUT.getFieldLength()
						|| poseObservation.pose().getY() > TAG_LAYOUT.getFieldWidth();

				if (!rejectPose) {
					robotPosesAccepted.add(poseObservation.pose());
				} else {
					continue;
				}

				double stdDevFactor =
					Math.pow(poseObservation.avgTagDist(), 2.0) / poseObservation.tagCount();
				double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
				double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

				if (camIdx < CAMERA_STD_DEV_FACTORS.length) {
					linearStdDev *= CAMERA_STD_DEV_FACTORS[camIdx];
					angularStdDev *= CAMERA_STD_DEV_FACTORS[camIdx];
				}

				visionConsumer.accept(
						poseObservation.pose().toPose2d(),
						poseObservation.timestampSeconds(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
				Logger.recordOutput(
						"Vision/Camera" + camIdx + "/RobotPosesAccepted",
						robotPosesAccepted.toArray(new Pose3d[0]));
				allRobotPosesAccepted.addAll(robotPosesAccepted);
			}
		}

		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[0]));
	}

	@FunctionalInterface
	public interface VisionConsumer {
		/**
		 * Accepts an robot pose estimation.
		 * @param pose2d the estimated pose
		 * @param timestampSeconds the timestamp of the pose
		 * @param visionMeasurementStdDevs the standard deviations
		 */
		void accept(
			Pose2d pose2d,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs
		);
	}
}
