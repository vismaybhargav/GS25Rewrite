package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class Constants {
	public static final class DriveConstants {
		public static final double SYS_ID_VOLT_DAMP = 6;
	}

	public static final class ModuleConstants {
		public static final double DRIVE_P = 0;
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 0;
		public static final double DRIVE_V = 0;

		public static final double DRIVE_CURRENT_LIMIT = 0;
		public static final double STEER_CURRENT_LIMIT = 0;

		public static final double STEER_P = 0;
		public static final double STEER_I = 0;
		public static final double STEER_D = 0;
		public static final double STEER_V = 0;
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
	}

	public static final class VisionConstants {

	}
}
