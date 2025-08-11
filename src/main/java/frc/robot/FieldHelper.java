package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
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

    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4
    }

	private static Map<ReefSide, AprilTag> reefAprilTags = new HashMap<>();

	public static final int TAG_ID_REEF_SIDE_A = 18;
	public static final int TAG_ID_REEF_SIDE_B = 17;
	public static final int TAG_ID_REEF_SIDE_C = 22;
	public static final int TAG_ID_REEF_SIDE_D = 21;
	public static final int TAG_ID_REEF_SIDE_E = 20;
	public static final int TAG_ID_REEF_SIDE_F = 19;

	public static final int TAG_ID_TEST_REEF_LEFT = 2;
	public static final int TAG_ID_TEST_REEF_RIGHT = 3;

	static {

		if (Features.USE_TEST_FIELD) {
			reefAprilTags.put(
				ReefSide.A,
				new AprilTag(
					TAG_ID_TEST_REEF_LEFT,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_REEF_LEFT).orElse(null)));
			reefAprilTags.put(
				ReefSide.B,
				new AprilTag(
					TAG_ID_TEST_REEF_RIGHT,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_REEF_RIGHT).orElse(null)));
			reefAprilTags.put(
				ReefSide.C,
				new AprilTag(
					TAG_ID_TEST_REEF_LEFT,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_REEF_LEFT).orElse(null)));
			reefAprilTags.put(
				ReefSide.D,
				new AprilTag(
					TAG_ID_TEST_REEF_RIGHT,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_REEF_RIGHT).orElse(null)));
			reefAprilTags.put(
				ReefSide.E,
				new AprilTag(
					TAG_ID_TEST_REEF_LEFT,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_REEF_LEFT).orElse(null)));
			reefAprilTags.put(
				ReefSide.F,
				new AprilTag(
					TAG_ID_TEST_REEF_RIGHT,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_REEF_RIGHT).orElse(null)));
		} else {
			reefAprilTags.put(
				ReefSide.A,
				new AprilTag(
					TAG_ID_REEF_SIDE_A,
					TAG_LAYOUT.getTagPose(TAG_ID_REEF_SIDE_A).orElse(null)));
			reefAprilTags.put(
				ReefSide.B,
				new AprilTag(
					TAG_ID_REEF_SIDE_B,
					TAG_LAYOUT.getTagPose(TAG_ID_REEF_SIDE_B).orElse(null)));
			reefAprilTags.put(
				ReefSide.C,
				new AprilTag(
					TAG_ID_REEF_SIDE_C,
					TAG_LAYOUT.getTagPose(TAG_ID_REEF_SIDE_C).orElse(null)));
			reefAprilTags.put(
				ReefSide.D,
				new AprilTag(
					TAG_ID_REEF_SIDE_D,
					TAG_LAYOUT.getTagPose(TAG_ID_REEF_SIDE_D).orElse(null)));
			reefAprilTags.put(
				ReefSide.E,
				new AprilTag(
					TAG_ID_REEF_SIDE_E,
					TAG_LAYOUT.getTagPose(TAG_ID_REEF_SIDE_E).orElse(null)));
			reefAprilTags.put(
				ReefSide.F,
				new AprilTag(
					TAG_ID_REEF_SIDE_F,
					TAG_LAYOUT.getTagPose(TAG_ID_REEF_SIDE_F).orElse(null)));
		}
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
			branchSide == BranchSide.LEFT
				? AutoConstants.REEF_Y_LEFT_OFFSET.in(Meters)
				: AutoConstants.REEF_Y_RIGHT_OFFSET.in(Meters), // Side to Side
			Rotation2d.k180deg
		);

		return atPose.transformBy(offsetTransform);
	}

	/**
	 * Get a map of the reef april tags.
	 * @return a map of the reef april tags
	 */
	public static Map<ReefSide, AprilTag> getReefAprilTags() {
		return reefAprilTags;
	}

	public enum StationSide {
		LEFT, RIGHT
	}

	public enum StationPosition {
		FAR_LEFT, LEFT, CENTER, RIGHT, FAR_RIGHT
	}

	private static Map<StationSide, AprilTag> stationAprilTags = new HashMap<>();

	public static final int TAG_ID_STATION_LEFT = 1;
	public static final int TAG_ID_STATION_RIGHT = 2;

	public static final int TAG_ID_TEST_STATION = 1;

	static {

		if (Features.USE_TEST_FIELD) {
			stationAprilTags.put(
				StationSide.LEFT,
				new AprilTag(
					TAG_ID_TEST_STATION,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_STATION).orElse(null)));

			stationAprilTags.put(
				StationSide.RIGHT,
				new AprilTag(
					TAG_ID_TEST_STATION,
					TAG_LAYOUT.getTagPose(TAG_ID_TEST_STATION).orElse(null)));
		} else {

			stationAprilTags.put(
				StationSide.LEFT,
				new AprilTag(
					TAG_ID_STATION_LEFT,
					TAG_LAYOUT.getTagPose(TAG_ID_STATION_LEFT).orElse(null)));

			stationAprilTags.put(
				StationSide.RIGHT,
				new AprilTag(
					TAG_ID_STATION_RIGHT,
					TAG_LAYOUT.getTagPose(TAG_ID_STATION_RIGHT).orElse(null)));

		}
	}

	/**
	 * Get a map of the station april tags.
	 * @return a map of the station april tags
	 */
	public static Map<StationSide, AprilTag> getStationAprilTags() {
		return stationAprilTags;
	}

	/**
	 * Gets the desired pose for the robot to be aligned with the station.
	 * @param stationSide the side of the station
	 * @param stationPosition The position on the station
	 * @return the desired pose for the robot to be aligned with the station
	 */
	public static Pose2d getAlignedDesiredPoseForStation(StationSide stationSide,
		StationPosition stationPosition) {

		Pose2d atPose = stationAprilTags.get(stationSide).pose.toPose2d();
		Distance yOffset;

		switch (stationPosition) {
			case FAR_LEFT:
				yOffset = AutoConstants.STATION_FAR_LEFT_OFFSET;
				break;
			case LEFT:
				yOffset = AutoConstants.STATION_LEFT_OFFSET;
				break;
			case FAR_RIGHT:
				yOffset = AutoConstants.STATION_FAR_RIGHT_OFFSET;
				break;
			case RIGHT:
				yOffset = AutoConstants.STATION_RIGHT_OFFSET;
				break;
			default:
				yOffset = AutoConstants.STATION_CENTER_OFFSET;
		}

		Transform2d offsetTransform = new Transform2d(
			SimConstants.ROBOT_WIDTH.in(Meters) / 2, // Back to Front (Don't change this one)
			yOffset.in(Meters), // Side to Side
			Rotation2d.k180deg
		);

		return atPose.transformBy(offsetTransform);
	}
}
