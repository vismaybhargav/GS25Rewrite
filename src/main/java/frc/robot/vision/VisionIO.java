package frc.robot.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
	@AutoLog
	class VisionIOInputs {
		private TargetObservation latestTargetObservation =
				new TargetObservation(new Rotation2d(), new Rotation2d());
		private PoseObservation[] latestPoseObservations = new PoseObservation[0];
		private int[] tagIDs = new int[0];

		/**
		 * Gets the most recent tag observation.
		 * @return the most recent tag observation
		 */
		public TargetObservation getLatestTargetObservation() {
			return latestTargetObservation;
		}

		/**
		 * Gets the most recent pose observations.
		 * @return a list o all the recent pose observations
		 */
		public PoseObservation[] getLatestPoseObservations() {
			return latestPoseObservations;
		}

		/**
		 * Gets the tag ID's used in the pose observation.
		 * @return the tag ID's used
		 */
		public int[] getTagIDs() {
			return tagIDs;
		}

		/**
		 * Set latest target observation.
		 * @param recentTargetObservation the latest target observation
		 */
		public void setLatestTargetObservation(TargetObservation recentTargetObservation) {
			latestTargetObservation = recentTargetObservation;
		}

		/**
		 * Set the latest pose observation.
		 * @param recentPoseObservations the recent pose observations
		 */
		public void setLatestPoseObservations(PoseObservation[] recentPoseObservations) {
			latestPoseObservations = recentPoseObservations;
		}

		/**
		 * Set the tag ID's used.
		 * @param tagIds the tagID's used
		 */
		public void setTagIDs(int[] tagIds) {
			tagIDs = tagIds;
		}
	}

	/**
	 * Represents the angle to a simple target, not used for pose estimation.
	 * @param tx x rotation
	 * @param ty y rotation
	 */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty) { }

	/**
	 * Represents a robot pose sample used for pose estimation.
	 * @param timestampSeconds the timestamp
	 * @param pose the pose
	 * @param ambiguity idk what this is lol
	 * @param tagCount # of tags in the pose obs
	 * @param avgTagDist avg tag distance
	 * @param type the type of PoseObservation
	 */
	public static record PoseObservation(
			// Has to be a double because AdvantageScope doesn't like the Time class
			double timestampSeconds,
			Pose3d pose,
			double ambiguity,
			int tagCount,
			double avgTagDist,
			PoseObservationType type
	) { }

	enum PoseObservationType {
		PHOTONVISION,
		QUESTNAV
	}

	/**
	 * Updates the inputs.
	 * @param inputs the inputs
	 */
	default void updateInputs(VisionIOInputs inputs) { }
}
