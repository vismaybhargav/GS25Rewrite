package frc.robot.vision;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;
import limelight.networktables.LimelightSettings.ImuMode;

public class VisionIOLimelight implements VisionIO {
	private Limelight limelight;
	private Supplier<Orientation3d> orientationSupplier;

	private LimelightPoseEstimator mt1PoseEstimator;
	private LimelightPoseEstimator mt2PoseEstimator;

	/**
	 * Creates a new VisionIOYALL implementation.
	 * @param name The name of the Limelight (e.g. "limelight" or "limelight-1")
	 * @param orientationSupp A supplier that provides the robot's orientation
	 */
	public VisionIOLimelight(String name, Supplier<Orientation3d> orientationSupp) {
		orientationSupplier = orientationSupp;
		limelight = new Limelight(name);

		mt1PoseEstimator = limelight.getPoseEstimator(false);
		mt2PoseEstimator = limelight.getPoseEstimator(true);

		limelight.getSettings().withImuMode(ImuMode.ExternalImu).save();

		seedLimelight(Optional.of(Rotation2d.kZero));

		limelight.getSettings().withImuMode(ImuMode.InternalImu).save();
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		var limelightResults = limelight.getLatestResults();
		inputs.setConnected(limelightResults.isPresent());

		var poseObservations = new LinkedList<PoseObservation>();

		Optional<PoseEstimate> optEst = mt2PoseEstimator.getPoseEstimate();
		optEst.ifPresent(
				(PoseEstimate estimate) -> {
					poseObservations.add(new PoseObservation(
							estimate.timestampSeconds,
							estimate.pose,
							estimate.getAvgTagAmbiguity(),
							estimate.rawFiducials.length,
							estimate.avgTagDist,
							PoseObservationType.MEGATAG_2));
				});
		inputs.setPoseObservations(poseObservations.toArray(new PoseObservation[0]));

		Set<Integer> tagIds = new HashSet<>();
		RawFiducial[] data = limelight.getData().getRawFiducials();
		// if (optEst.isPresent()){
		// 	RawFiducial[] data = optEst.get().rawFiducials;
		// 	if(data.length > 0) {
		// 		inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(data[0].txnc), Rotation2d.fromDegrees(data[0].tync));
		// 	}
		// 	Arrays.stream(data).forEach(fid -> tagIds.add(fid.id));
		// }
		
		if (data.length > 0) {
			inputs.setLatestTargetObservation(new TargetObservation(Rotation2d.fromDegrees(data[0].txnc), Rotation2d.fromDegrees(data[0].tync)));
		}
		Arrays.stream(data).forEach(fid -> tagIds.add(fid.id));

		// limelightResults.ifPresent(results -> {
		// 	var targets = results.targets_Fiducials;
		// 	System.out.println("Targets " + targets);
		// 	Logger.recordOutput("NumberOfTargets", targets.length);


		// 	if (targets.length == 0) {
		// 		return;
		// 	}
		// 	var firstTarget = targets[0];

		// 	inputs.latestTargetObservation = new TargetObservation(
		// 			Rotation2d.fromDegrees(firstTarget.tx),
		// 			Rotation2d.fromDegrees(firstTarget.ty));

		// 	for (var tag : targets) {
		// 		Logger.recordOutput("tag"+ tag.fiducialID, (int)tag.fiducialID);
		// 		tagIds.add((int) tag.fiducialID);
		// 	}
		// });
		inputs.setTagIds(tagIds.stream().mapToInt(Integer::intValue).toArray());
	}

	public void seedLimelight(Optional<Rotation2d> optionalRotation) {
		var orientation = new Orientation3d(new Rotation3d(
			optionalRotation.orElse(new Rotation2d())),
			new AngularVelocity3d(
				RotationsPerSecond.of(0),
				RotationsPerSecond.of(0),
				RotationsPerSecond.of(0)
			)
		);
		limelight.getSettings().withRobotOrientation(orientation).save();
	}
}
