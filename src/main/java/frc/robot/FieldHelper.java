package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SimConstants;

public final /* singleton */ class FieldHelper {
	/**
	 * Reef Side A starts at the one closest to the driver station wall,
	 * and then moves counter clockwise around.
	 */
	public enum ReefSide {
		A, B, C, D, E, F
	}

	public enum BranchSide {
		LEFT, RIGHT
	}

	public static Map<ReefSide, AprilTag> reefAprilTags = new HashMap<>();

	static {
		reefAprilTags.put(ReefSide.A, new AprilTag(18, TAG_LAYOUT.getTagPose(18).orElse(null)));
		reefAprilTags.put(ReefSide.B, new AprilTag(17, TAG_LAYOUT.getTagPose(17).orElse(null)));
		reefAprilTags.put(ReefSide.C, new AprilTag(22, TAG_LAYOUT.getTagPose(22).orElse(null)));
		reefAprilTags.put(ReefSide.D, new AprilTag(21, TAG_LAYOUT.getTagPose(21).orElse(null)));
		reefAprilTags.put(ReefSide.E, new AprilTag(20, TAG_LAYOUT.getTagPose(20).orElse(null)));
		reefAprilTags.put(ReefSide.F, new AprilTag(19, TAG_LAYOUT.getTagPose(19).orElse(null)));
	}

	private FieldHelper() {
		// Prevent instantiation
	}

	/**
	 * Gets the desired pose for the robot to be aligned with the reef.
	 * @param reefSide the side of the reef
	 * @param branchSide the side of the branch
	 * @return the desired pose for the robot to be aligned with the reef
	 */
	public static Pose2d getAlignedDesiredPoseForReef(ReefSide reefSide, BranchSide branchSide) {
		Pose2d atPose = reefAprilTags.get(reefSide).pose.toPose2d();

		Transform2d offsetTransform = new Transform2d(
			SimConstants.ROBOT_WIDTH.in(Meters) / 2, // Back to Front (Don't change this one)
			branchSide == BranchSide.LEFT ?
				AutoConstants.REEF_Y_LEFT_OFFSET.in(Meters) :
				AutoConstants.REEF_Y_RIGHT_OFFSET.in(Meters), // Side to Side
			Rotation2d.k180deg
		);

		return atPose.transformBy(offsetTransform);
	}
}
