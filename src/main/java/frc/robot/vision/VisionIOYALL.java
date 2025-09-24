package frc.robot.vision;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import limelight.Limelight;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.ImuMode;

public class VisionIOYALL implements VisionIO {
	private Limelight limelight;
	private String limelightName;
	private Supplier<Orientation3d> orientationSupplier;

	public VisionIOYALL(String name, Supplier<Orientation3d> orientationSupp) {
		this.limelightName = name;
		orientationSupplier = orientationSupp;
		limelight = new Limelight(name);

		limelight.getSettings().withImuMode(ImuMode.ExternalImu).save();
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.connected = Limelight.isAvailable(limelightName);

		limelight
			.getSettings()
			.withRobotOrientation(orientationSupplier.get())
			.save();

		limelight.flush(); // reccommended by Limelight

		PoseObservation[] poseObservations = new PoseObservation[2];
		Optional<PoseEstimate> optEst = limelight.getPoseEstimator(true).getPoseEstimate();
		optEst.ifPresent((PoseEstimate estimate) -> {
			poseObservations[0] =
				new PoseObservation(
					estimate.timestampSeconds, // TODO: Sync with latency?
					estimate.pose,
					estimate.getAvgTagAmbiguity(),
					estimate.rawFiducials.length,
					estimate.avgTagDist,
					PoseObservationType.MEGATAG_2
				);
		});

		Optional<PoseEstimate> optEst1 = limelight.getPoseEstimator(false).getPoseEstimate();
		optEst1.ifPresent((PoseEstimate estimate) -> {
			poseObservations[1] =
				new PoseObservation(
					estimate.timestampSeconds, // TODO: Sync with latency?
					estimate.pose,
					estimate.getAvgTagAmbiguity(),
					estimate.rawFiducials.length,
					estimate.avgTagDist,
					PoseObservationType.MEGATAG_1
				);
		});

		inputs.poseObservations = poseObservations;

		Set<Integer> tagIds = new HashSet<>();
		// not sure I should do this or I should get from LimelightData
		limelight.getLatestResults().ifPresent(results -> {
			var targets = results.targets_Fiducials;
			var firstTarget = targets[0];
			inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(firstTarget.tx), Rotation2d.fromDegrees(firstTarget.ty));

			for (var tag : targets) {
				tagIds.add((int) tag.fiducialID); // why on god's flat earth is this a double?
			}
		});

		// convert to array. still bad :(
		// TODO: cleanup
		AtomicInteger i = new AtomicInteger(0);
		var ids = new int[tagIds.size()];
		tagIds.forEach(id -> ids[i.getAndIncrement()] = id);

		inputs.tagIds = ids;
	}
}
