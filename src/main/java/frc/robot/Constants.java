package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
	public static final class DriveConstants {
		public static final double SYS_ID_VOLT_DAMP = 6;

		public static final double TRANSLATION_DEADBAND = 0.5;
		public static final double ROTATION_DEADBAND = 0.5;
		public static final AngularVelocity MAX_ANGULAR_VELO_RPS = RotationsPerSecond.of(0.75);
	}

	public static final class ModuleConstants {
		public static final double DRIVE_P = 0.1;
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 0;
		public static final double DRIVE_V = 0.124;

		public static final double DRIVE_CURRENT_LIMIT = 60;
		public static final double STEER_CURRENT_LIMIT = 60;

		public static final double STEER_P = 100;
		public static final double STEER_I = 0;
		public static final double STEER_D = 0.5;
		public static final double STEER_V = 0.1;
		public static final double STEER_S = 0;
	}

	public static final class AutoConstants {
		public static final double TRANSLATION_P = 10;
		public static final double TRANSLATION_I = 0;
		public static final double TRANSLATION_D = 0;

		public static final double ROTATION_P = 7;
		public static final double ROTATION_I = 0;
		public static final double ROTATION_D = 0;
	}

	public static final class SimConstants {
		public static final Mass MASS_WITH_BUMPER = Pounds.of(115);
		public static final Distance ROBOT_LENGTH = Inches.of(35.5);
		public static final Distance ROBOT_WIDTH = Inches.of(35.5);
		public static final double WHEEL_COF = 1.2;

		public static final double STEER_P = 70;
		public static final double STEER_I = 0;
		public static final double STEER_D = 4.5;
		public static final double STEER_S = 0;
		public static final double STEER_V = 1.91;
		public static final double STEER_A = 0;

		public static final double STEER_MOTOR_GEAR_RATIO = 16.0;
		public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.1);
		public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.51);
		public static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.05);
	}

	public static final class VisionConstants {
		public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
				.loadField(AprilTagFields.k2025ReefscapeWelded);

		public static final String REEF_CAMERA_NAME = "Reef Camera";
		public static final String STATION_CAMERA_NAME = "Station Camera";

		public static final Transform3d ROBOT_TO_REEF_CAM =
			new Transform3d(Units.inchesToMeters(7.129),
				-Units.inchesToMeters(4.306),
				Units.inchesToMeters(14.56), new Rotation3d(0.0, 0.0, 0.0));
		public static final Transform3d ROBOT_TO_STATION_CAM = new Transform3d(
				-Units.inchesToMeters(8.875),
				-Units.inchesToMeters(9.5),
				Units.inchesToMeters(37.596), new Rotation3d(0.0, -Math.toRadians(19), Math.PI));

		public static final double MAX_AMBIGUITY = 0.5;
		public static final double MAX_Z_ERROR = 5.0; // meters

		public static final double LINEAR_STD_DEV_BASELINE = 0.02;
		public static final double ANGULAR_STD_DEV_BASELINE = 0.06;

		public static final double[] CAMERA_STD_DEV_FACTORS = new double[] {
			1.0, // Reef Camera
			1.0 // Station Camera
		};
	}
}
