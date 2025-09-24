package frc.robot.vision;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.LimelightHelpers;
import frc.robot.vision.VisionIO.PoseObservation;

public class VisionIOLimelight implements VisionIO {
	private String limelightName;
	private final Supplier<Rotation2d> rotationSupplier;

	public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupp) {
		limelightName = name;
		rotationSupplier = rotationSupp;
		LimelightHelpers.SetIMUMode(name, 0); // Use external IMU yaw submitted with SetRobotOrientation()
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		// Consider Limelight disconnected if no valid pipeline data received in last 250ms
		inputs.connected = ((RobotController.getFPGATime() - LimelightHelpers.getLatency_Pipeline(limelightName)) / 1000) < 250;

		// This is required for MT2
		LimelightHelpers.SetRobotOrientation(limelightName, rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0);

		var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
		var est1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

		inputs.latestTargetObservation =
			new TargetObservation(
				Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)),
				Rotation2d.fromDegrees(LimelightHelpers.getTY(limelightName))
			);

		LimelightHelpers.Flush(); // Recommended by Limelight

		Set<Integer> tagIds = new HashSet<>();
		for (var rawFiducial : LimelightHelpers.getRawFiducials(limelightName)) {
			tagIds.add(rawFiducial.id);
		}

		// Convert to array. omg this so bad.
		// TODO: cleanup
		AtomicInteger i = new AtomicInteger(0);
		var ids = new int[tagIds.size()];
		tagIds.forEach(id -> ids[i.getAndIncrement()] = id);

		inputs.tagIds = ids;

		inputs.poseObservations = new PoseObservation[] {
			new PoseObservation(
				est.timestampSeconds, // TODO: Sync with latency?
				new Pose3d(est.pose),
				0,
				est.tagCount,
				est.avgTagDist,
				PoseObservationType.MEGATAG_2
			),
			new PoseObservation(
				est1.timestampSeconds, // TODO: Sync with latency?
				new Pose3d(est1.pose),
				est1.rawFiducials[0].ambiguity, // Using the first tag, should we avg?
				est1.tagCount,
				est1.avgTagDist,
				PoseObservationType.MEGATAG_1
			)
		};
	}
}
