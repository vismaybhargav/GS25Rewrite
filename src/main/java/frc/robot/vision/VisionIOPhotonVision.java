package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class VisionIOPhotonVision implements VisionIO {
	private final PhotonCamera camera;
	private final Transform3d robotToCamera;

	/**
	 * Creates a new VisionIOPhotonVision object.
	 * @param name name of the source
	 * @param robot2Camera transform from robot to camera
	 */
	public VisionIOPhotonVision(String name, Transform3d robot2Camera) {
		camera = new PhotonCamera(name);
		robotToCamera = robot2Camera;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();

		for (var result : camera.getAllUnreadResults()) {
			if (result.multitagResult.isPresent()) {
				var multiTagResult = result.multitagResult.get();

				Transform3d fieldToCamera = multiTagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
				Pose3d robotPose =
					new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

				double totalTagDist = 0.0;
				for (var target : result.getTargets()) {
					totalTagDist += target.getBestCameraToTarget().getTranslation().getNorm();
				}

				tagIds.addAll(multiTagResult.fiducialIDsUsed);

				poseObservations.add(
						new PoseObservation(
								result.getTimestampSeconds(),
								robotPose,
								multiTagResult.estimatedPose.ambiguity,
								multiTagResult.fiducialIDsUsed.size(),
								totalTagDist / result.getTargets().size(),
								PoseObservationType.PHOTONVISION));
			} else if (!result.getTargets().isEmpty()) {
				var target = result.getBestTarget();

				var tagPose = TAG_LAYOUT.getTagPose(target.getFiducialId());
				if (tagPose.isPresent()) {
					Transform3d fieldToTarget =
							new Transform3d(
								tagPose.get().getTranslation(), tagPose.get().getRotation());
					Transform3d cameraToTarget = target.getBestCameraToTarget();
					Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
					Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
					Pose3d robotPose =
						new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

					tagIds.add((short) target.getFiducialId());

					poseObservations.add(
							new PoseObservation(
									result.getTimestampSeconds(),
									robotPose,
									target.getPoseAmbiguity(),
									1,
									cameraToTarget.getTranslation().getNorm(),
									PoseObservationType.PHOTONVISION
							)
					);
				}
			}
		}

		inputs.setLatestPoseObservations(new PoseObservation[poseObservations.size()]);
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.getLatestPoseObservations()[i] = poseObservations.get(i);
		}

		inputs.setTagIDs(new int[tagIds.size()]);
		int i = 0;
		for (short tagId : tagIds) {
			inputs.getTagIDs()[i++] = tagId;
		}
	}

	/**
	 * Get the camera.
	 * @return the camera
	 */
	public PhotonCamera getCamera() {
		return camera;
	}

	/**
	 * Get the robot to camera transform.
	 * @return robot to camera transform
	 */
	public Transform3d getRobotToCamera() {
		return robotToCamera;
	}
}
