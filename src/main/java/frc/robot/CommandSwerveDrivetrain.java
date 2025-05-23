package frc.robot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
	private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
	private Notifier simNotifier = null;
	private double lastSimTime;

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean hasAppliedOperatorPerspective = false;

	/** Swerve request to apply during robot-centric path following. */
	private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
		new SwerveRequest.ApplyRobotSpeeds();

	/* Swerve requests to apply during SysId characterization */
	private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
		new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
		new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
		new SwerveRequest.SysIdSwerveRotation();

	/* SysId routine for characterizing translation.
		This is used to find PID gains for the drive motors. */
	private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,        // Use default ramp rate (1 V/s)
			Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
			null,        // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> setControl(translationCharacterization.withVolts(output)),
			null,
			this
		)
	);

	/* SysId routine for characterizing steer.
		This is used to find PID gains for the steer motors.
	*/
	private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,        // Use default ramp rate (1 V/s)
			Volts.of(7), // Use dynamic voltage of 7 V
			null,        // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			volts -> setControl(steerCharacterization.withVolts(volts)),
			null,
			this
		)
	);

	/*
	 * SysId routine for characterizing rotation.
	 * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
	 * See the documentation of SwerveRequest.SysIdSwerveRotation
	 * for info on importing the log to SysId.
	 */
	private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
		new SysIdRoutine.Config(
			/* This is in radians per second², but SysId only supports "volts per second" */
			Volts.of(Math.PI / DriveConstants.SYS_ID_VOLT_DAMP).per(Second),
			/* This is in radians per second, but SysId only supports "volts" */
			Volts.of(Math.PI),
			null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> {
				/* output is actually radians per second, but SysId only supports "volts" */
				setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
				/* also log the requested output for SysId */
				SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
			},
			null,
			this
		)
	);

	/* The SysId routine to test */
	private SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslation;


	private PPHolonomicDriveController ppHolonomicDriveController = new PPHolonomicDriveController(
					// PID constants for translation
					new PIDConstants(
						AutoConstants.TRANSLATION_P,
						AutoConstants.TRANSLATION_I,
						AutoConstants.TRANSLATION_D
					),
					// PID constants for rotation
					new PIDConstants(
						AutoConstants.ROTATION_P,
						AutoConstants.ROTATION_I,
						AutoConstants.ROTATION_D
					)
				);

	private BiConsumer<ChassisSpeeds, DriveFeedforwards> ppPathfindConsumer =
		(speeds, feedforwards) -> {
			pathApplyRobotSpeeds
				.withSpeeds(speeds)
				.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
				.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
		};
	private RobotConfig config = null;

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 * <p>
	 * This constructs the underlying hardware devices, so users should not construct
	 * the devices themselves. If they need the devices, they can access them through
	 * getters in the classes.
	 *
	 * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
	 * @param modules             Constants for each specific module
	 */
	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
		configureAutoBuilder();
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 * <p>
	 * This constructs the underlying hardware devices, so users should not construct
	 * the devices themselves. If they need the devices, they can access them through
	 * getters in the classes.
	 *
	 * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
	 *                                   unspecified or set to 0 Hz, this is 250 Hz on
	 *                                   CAN FD, and 100 Hz on CAN 2.0.
	 * @param modules                    Constants for each specific module
	 */
	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		double odometryUpdateFrequency,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, odometryUpdateFrequency, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
		configureAutoBuilder();
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 * <p>
	 * This constructs the underlying hardware devices, so users should not construct
	 * the devices themselves. If they need the devices, they can access them through
	 * getters in the classes.
	 *
	 * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
	 *                                   unspecified or set to 0 Hz, this is 250 Hz on
	 *                                   CAN FD, and 100 Hz on CAN 2.0.
	 * @param odometryStandardDeviation  The standard deviation for odometry calculation
	 *                                  in the form [x, y, theta]ᵀ, with units in meters
	 *                                  and radians
	 * @param visionStandardDeviation   The standard deviation for vision calculation
	 *                                  in the form [x, y, theta]ᵀ, with units in meters
	 *                                  and radians
	 * @param modules                    Constants for each specific module
	 */
	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		double odometryUpdateFrequency,
		Matrix<N3, N1> odometryStandardDeviation,
		Matrix<N3, N1> visionStandardDeviation,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(
			drivetrainConstants,
			odometryUpdateFrequency,
			odometryStandardDeviation,
			visionStandardDeviation,
			modules
		);

		if (Utils.isSimulation()) {
			startSimThread();
		}
		configureAutoBuilder();
	}

	private void configureAutoBuilder() {
		try {
			config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
				() -> getState().Pose,   // Supplier of current robot pose
				this::resetPose,         // Consumer for seeding pose against auto
				() -> getState().Speeds, // Supplier of current robot speeds
				// Consumer of ChassisSpeeds and feedforwards to drive the robot
				ppPathfindConsumer,
				ppHolonomicDriveController,
				config,
				// Assume the path needs to be flipped for Red vs Blue, this is normally the case
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this // Subsystem for requirements
			);
		} catch (Exception ex) {
			DriverStation.reportError(
				"Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
		}
	}

	/**
	 * Returns a command that applies the specified control request to this swerve drivetrain.
	 *
	 * @param requestSupplier Function returning the request to apply
	 * @return Command to run
	 */
	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine
	 * specified by {@link #sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutineToApply.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine
	 * specified by {@link #sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutineToApply.dynamic(direction);
	}

	@Override
	public void periodic() {
		/*
		 * Periodically try to apply the operator perspective.
		 * If we haven't applied the operator perspective before,
		 * then we should apply it regardless of DS state.
		 * This allows us to correct the perspective in case the robot code restarts mid-match.
		 * Otherwise, only check and apply the operator perspective if the DS is disabled.
		 * This ensures driving behavior doesn't change until an explicit
		 * disable event occurs during testing.
		 */
		if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(
					allianceColor == Alliance.Red
						? RED_ALLIANCE_PERSPECTIVE_ROTATION
						: BLUE_ALLIANCE_PERSPECTIVE_ROTATION
				);
				hasAppliedOperatorPerspective = true;
			});
		}
	}

	private void startSimThread() {
		lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - lastSimTime;
			lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		simNotifier.startPeriodic(SIM_LOOP_PERIOD);
	}

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
	 * while still accounting for measurement noise.
	 *
	 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds The timestamp of the vision measurement in seconds.
	 */
	@Override
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		super.addVisionMeasurement(
			visionRobotPoseMeters,
			Utils.fpgaToCurrentTime(timestampSeconds)
		);
	}

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
	 * while still accounting for measurement noise.
	 * <p>
	 * Note that the vision measurement standard deviations passed into this method
	 * will continue to apply to future measurements until a subsequent call to
	 * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
	 *
	 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds The timestamp of the vision measurement in seconds.
	 * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
	 *     in the form [x, y, theta]ᵀ, with units in meters and radians.
	 */
	@Override
	public void addVisionMeasurement(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {
		super.addVisionMeasurement(
			visionRobotPoseMeters,
			Utils.fpgaToCurrentTime(timestampSeconds),
			visionMeasurementStdDevs);
	}

	/**
	 * Get the pathplanner holonomic drive controller.
	 * @return the drive controller
	 */
	public PPHolonomicDriveController getPPHolonomicDriveController() {
		return ppHolonomicDriveController;
	}

	/**
	 * Get the biconsumer to apply the speeds to.
	 * @return the bi consumer
	 */
	public BiConsumer<ChassisSpeeds, DriveFeedforwards> getPpPathfindConsumer() {
		return ppPathfindConsumer;
	}

	/**
	 * Get the PP config.
	 * @return the pp config
	 */
	public RobotConfig getPPConfig() {
		return config;
	}
}
