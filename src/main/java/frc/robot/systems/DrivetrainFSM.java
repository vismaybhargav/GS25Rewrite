package frc.robot.systems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Features;
import frc.robot.FieldHelper;
import frc.robot.Robot;
import frc.robot.FieldHelper.BranchSide;
import frc.robot.FieldHelper.ReefSide;


// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.generated.LocalADStarAK;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.SimSwerveDrivetrainConfig;

public class DrivetrainFSM extends DualSetFSMSystem<DrivetrainFSM.DriveSystemState, DrivetrainFSM.DriveWantedState> {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum DriveSystemState {
		TELEOPING,
		PATHFINDING
	}

	public enum DriveWantedState {
		TELEOP,
		PATHFIND
	}

	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_AT_12_VOLTS;
	// kSpeedAt12Volts desired top speed
	private static final AngularVelocity MAX_ANGULAR_RATE = DriveConstants.MAX_ANGULAR_VELO_RPS;
	// 3/4 rps angle velo

	/* ======================== Private variables ======================== */
	private CommandSwerveDrivetrain drivetrain;

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DriveConstants.TRANSLATION_DEADBAND) // 4% deadband
			.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
					* DriveConstants.ROTATION_DEADBAND) // 4% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors

	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DriveConstants.TRANSLATION_DEADBAND) // 4% deadband
			.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
					* DriveConstants.ROTATION_DEADBAND) // 4% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors

	private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
			.withDeadband(MAX_SPEED.in(MetersPerSecond)
					* DriveConstants.TRANSLATION_DEADBAND) // 4% deadband
			.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
					* DriveConstants.ROTATION_DEADBAND) // 4% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

	private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

	/* ======================== Pathfinding Stuffs ================== */
	private final Timer timer = new Timer();
	private double timeOffset = 0;
	private ReefSide currentReefSide = ReefSide.A;
	private BranchSide currentBranchSide = BranchSide.RIGHT;
	private Pose2d targetPose = FieldHelper.getAlignedDesiredPoseForReef(currentReefSide, currentBranchSide);
	private ReefSide[] reefSides = ReefSide.values();
	private Pose2d originalTargetPose = new Pose2d(
			targetPose.getTranslation(), targetPose.getRotation());
	private GoalEndState goalEndState = new GoalEndState(0, targetPose.getRotation());
	private PathPlannerTrajectory currentTrajectory = null;
	private PathPlannerPath currentPath = null;
	private boolean pathfindingFinished = true;

	private PathConstraints pathConstraints = new PathConstraints(
		MAX_SPEED.in(MetersPerSecond),
		2 + 1,
		MAX_ANGULAR_RATE.in(RadiansPerSecond),
		Math.pow(MAX_ANGULAR_RATE.in(RadiansPerSecond), 2)
	);

	private MapleSimSwerveDrivetrain simDrivetrain;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */

	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DrivetrainFSM() {
		// Perform hardware init
		drivetrain = TunerConstants.createDrivetrain();

		var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
		var location = DriverStation.getLocation().orElse(1);

		Pose2d startingPose = FieldHelper.getStartingPose(alliance, location);

		if (Robot.isSimulation()) {
			simDrivetrain = new MapleSimSwerveDrivetrain(
				SimSwerveDrivetrainConfig
					.getDefault()
					.withPigeon(drivetrain.getPigeon2())
					.withModuleLocations(drivetrain.getModuleLocations())
					.withModules(drivetrain.getModules())
					.withStartingPose(
						startingPose
					)
			);
		}

		drivetrain.resetPose(startingPose);

		Pathfinding.setPathfinder(new LocalADStarAK());

		PathPlannerLogging.setLogActivePathCallback((path) -> {
			Logger.recordOutput("PathPlanner/Trajectory", path.toArray(new Pose2d[path.size()]));
		});

		PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
			Logger.recordOutput("PathPlanner/Current Pose", currentPose);
		});

		PathPlannerLogging.setLogTargetPoseCallback((targPose) -> {
			Logger.recordOutput("PathPlanner/Target Pose", targPose);
		});

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	@Override
	public void reset() {
		setSystemState(DriveSystemState.TELEOPING);
		requestWantedState(DriveWantedState.TELEOP);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	@Override
	public void update(TeleopInput input) {
		Logger.recordOutput("Drivetrain/Wanted State", getWantedState());
		Logger.recordOutput("Drivetrain/Current State", getSystemState());

		drivetrain.periodic();

		if (input != null && input.isCCWReefSelectionChangeButtonPressed()) {
			handleCCWReefSelect();
		} else if (input != null && input.isCWReefSelectionChangeButtonPressed()) {
			handleCWReefSelect();
		}

		TAG_LAYOUT.getTagPose(Robot.isSimulation() ? 18 : 2).ifPresent(pose -> {
			Pose2d robotPose = getPose();
			Rotation2d robotRot = robotPose.getRotation();
			Pose2d frontOfRobot = new Pose2d(
				robotPose.getX() + Math.cos(robotRot.getRadians()) * SimConstants.ROBOT_WIDTH.div(2).in(Meters),
				robotPose.getY() + Math.sin(robotRot.getRadians()) * SimConstants.ROBOT_WIDTH.div(2).in(Meters),
				new Rotation2d()
			);

			Pose2d error = pose.toPose2d().relativeTo(frontOfRobot);
			Logger.recordOutput("Vision/Front of bot", frontOfRobot);
			Logger.recordOutput("Vision/Error", error);
		});

		switch (getSystemState()) {
			case TELEOPING:
				handleTeleopState(input);
				break;
			case PATHFINDING:
				if (isPathfindingFinished()) {
					initalizePathfinding();
				}
				handlePathfindState();
				break;
			default:
				throw new IllegalStateException("Invalid state: " + getSystemState().toString());
		}
		setSystemState(nextState(input));
	}

	/* ======================== Private methods ======================== */
	@Override
	public DriveSystemState nextState(TeleopInput input) {
		if (input == null) {
			return DriveSystemState.TELEOPING;
		}

		switch (getWantedState()) {
			case TELEOP:
				return DriveSystemState.TELEOPING;
			case PATHFIND:
				return DriveSystemState.PATHFINDING;
			default:
				throw new IllegalStateException("Invalid State: " + getWantedState().toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handles the TELEOP_STATE, when the robot is controlled by the driver.
	 *
	 * @param input the input of the driver controller
	 */
	public void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}

		pathfindingFinished = true; // Resets the pathfinding, so that we
						// re-initialize when going into the pathfinding state.

		if (input.isSeedButtonPressed()) {
			drivetrain.seedFieldCentric();
		}

		double xSpeed = MathUtil.applyDeadband(
				-input.getDriveLeftJoystickY(),
				DriveConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
				-input.getDriveLeftJoystickX(),
				DriveConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
				-input.getDriveRightJoystickX(),
				DriveConstants.ROTATION_DEADBAND) * MAX_ANGULAR_RATE.in(RadiansPerSecond);

		drivetrain.setControl(
				drive
						.withVelocityX(xSpeed * DriveConstants.TRANSLATIONAL_DAMP)
						.withVelocityY(ySpeed * DriveConstants.TRANSLATIONAL_DAMP)
						.withRotationalRate(thetaSpeed * DriveConstants.ROTATIONAL_DAMP));
	}

	/**
	 * Handles the PATHFIND_STATE, when the robot is pathfinding to it's target
	 * pose.
	 */
	public void handlePathfindState() {
		if (pathfindingFinished) {
			return;
		}

		Pose2d currPose = getPose();
		ChassisSpeeds currSpeeds = getChassisSpeeds();

		PathPlannerLogging.logCurrentPose(currPose);
		PPLibTelemetry.setCurrentPose(currPose);

		// Skip updates if we are very close to the goal
		boolean skipUpdates = currentTrajectory != null
				&& currPose
						.getTranslation()
						.getDistance(currentTrajectory.getEndState().pose
								.getTranslation()) < VisionConstants.STOP_PATHFINDING_UPDATES.in(Meters);

		if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
			currentPath = Pathfinding.getCurrentPath(pathConstraints, goalEndState);

			if (currentPath != null) {
				currentTrajectory = new PathPlannerTrajectory(
						currentPath, currSpeeds, currPose.getRotation(), drivetrain.getPPConfig());
				if (!Double.isFinite(currentTrajectory.getTotalTimeSeconds())) {
					pathfindingFinished = true;
					return;
				}

				// Find the two closest states in front of and behind robot
				int closestState1Idx = 0;
				int closestState2Idx = 1;
				while (closestState2Idx < currentTrajectory.getStates().size() - 1) {
					double closest2Dist = currentTrajectory
							.getState(closestState2Idx).pose
							.getTranslation()
							.getDistance(currPose.getTranslation());
					double nextDist = currentTrajectory
							.getState(closestState2Idx + 1).pose
							.getTranslation()
							.getDistance(currPose.getTranslation());
					if (nextDist < closest2Dist) {
						closestState1Idx++;
						closestState2Idx++;
					} else {
						break;
					}
				}

				// Use the closest 2 states to interpolate what the time offset should be
				// This will account for the delay in pathfinding
				var closestState1 = currentTrajectory.getState(closestState1Idx);
				var closestState2 = currentTrajectory.getState(closestState2Idx);

				double d = closestState1.pose.getTranslation().getDistance(closestState2.pose.getTranslation());
				double t = (currPose
						.getTranslation().getDistance(closestState1.pose.getTranslation())) / d;
				t = MathUtil.clamp(t, 0.0, 1.0);

				timeOffset = MathUtil.interpolate(closestState1.timeSeconds, closestState2.timeSeconds, t);

				// If the robot is stationary and at the start of the path, set the time offset
				// to the next loop
				// This can prevent an issue where the robot will remain stationary if new paths
				// come in every loop
				if (timeOffset <= AutoConstants.TIME_STEP
						&& Math.hypot(
								currSpeeds.vxMetersPerSecond,
								currSpeeds.vyMetersPerSecond) < AutoConstants.MIN_VELOCITY_VEC) {
					timeOffset = AutoConstants.TIME_STEP;
				}

				PathPlannerLogging.logActivePath(currentPath);
				PPLibTelemetry.setCurrentPath(currentPath);
			}

			timer.reset();
			timer.start();
		}

		if (currentTrajectory != null) {
			var targetState = currentTrajectory.sample(timer.get() + timeOffset);

			ChassisSpeeds targSpeeds = drivetrain.getPPHolonomicDriveController()
					.calculateRobotRelativeSpeeds(currPose, targetState);

			double currentVel = Math.hypot(currSpeeds.vxMetersPerSecond, currSpeeds.vyMetersPerSecond);

			PPLibTelemetry.setCurrentPose(currPose);
			PathPlannerLogging.logCurrentPose(currPose);

			PPLibTelemetry.setTargetPose(targetState.pose);
			PathPlannerLogging.logTargetPose(targetState.pose);

			PPLibTelemetry.setVelocities(
					currentVel,
					targetState.linearVelocity,
					currSpeeds.omegaRadiansPerSecond,
					targSpeeds.omegaRadiansPerSecond);

			drivetrain.setControl(
					pathApplyRobotSpeeds
							.withSpeeds(targSpeeds)
							.withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesX())
							.withWheelForceFeedforwardsY(targetState.feedforwards.robotRelativeForcesY()));
		}
	}

	/**
	 * Does all the setup before pathfinding starts.
	 */
	public void initalizePathfinding() {
		currentTrajectory = null;
		pathfindingFinished = false;
		timeOffset = 0;

		Pose2d currentPose = getPose();
		drivetrain.getPPHolonomicDriveController().reset(currentPose, getChassisSpeeds());

		if (currentPose
				.getTranslation()
				.getDistance(
						targetPose.getTranslation()) < AutoConstants.POSE_TOLERANCE) {
			var ff = DriveFeedforwards.zeros(DriveConstants.NUM_MODULES);

			drivetrain.setControl(
					pathApplyRobotSpeeds
							.withSpeeds(getChassisSpeeds())
							.withWheelForceFeedforwardsX(ff.robotRelativeForcesX())
							.withWheelForceFeedforwardsY(ff.robotRelativeForcesY()));

			pathfindingFinished = true;
		} else {
			Pathfinding.setStartPosition(currentPose.getTranslation());
			Pathfinding.setGoalPosition(targetPose.getTranslation());
		}
	}

	/**
	 * Determine if the pathfinding is done.
	 *
	 * @return true if the pathfinding is done
	 */
	@AutoLogOutput(key = "Drivetrain/Pathfinding Finished")
	public boolean isPathfindingFinished() {
		if (pathfindingFinished) {
			return true;
		}

		if (currentTrajectory != null) {
			return timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() - timeOffset);
		}

		return false;
	}

	/**
	 * Handles when the CCW Reef selector button is pressed.
	 */
	public void handleCCWReefSelect() {

		if (currentBranchSide == BranchSide.LEFT) {
			currentReefSide = reefSides[(currentReefSide.ordinal() - 1 + reefSides.length) % reefSides.length];
			currentBranchSide = BranchSide.RIGHT;
		} else {
			currentBranchSide = BranchSide.LEFT;
		}

		targetPose = FieldHelper.getAlignedDesiredPoseForReef(currentReefSide, currentBranchSide);
		goalEndState = new GoalEndState(0, targetPose.getRotation());
	}

	/**
	 * Handles when the CW reef selector is pressed.
	 */
	public void handleCWReefSelect() {
		if (currentBranchSide == BranchSide.RIGHT) {
			currentReefSide = reefSides[(currentReefSide.ordinal() + 1) % reefSides.length];
			currentBranchSide = BranchSide.LEFT;
		} else {
			currentBranchSide = BranchSide.RIGHT;
		}

		targetPose = FieldHelper.getAlignedDesiredPoseForReef(currentReefSide, currentBranchSide);
		goalEndState = new GoalEndState(0, targetPose.getRotation());
	}

	/**
	 * Get the pose of the drivetrain.
	 *
	 * @return pose of the drivetrain
	 */
	@AutoLogOutput(key = "Drivetrain/Pose")
	public Pose2d getPose() {
		if (Robot.isSimulation() && Features.MAPLE_SIM_ENABLED) {
			return simDrivetrain.getMapleSimDrive().getSimulatedDriveTrainPose();
		}
		return drivetrain.getState().Pose;
	}

	/**
	 * Get the chassis speeds of the drivetrain.
	 *
	 * @return the drivetrain chassis speeds
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		if (Robot.isSimulation() && Features.MAPLE_SIM_ENABLED) {
			return simDrivetrain
				.getMapleSimDrive()
				.getDriveTrainSimulatedChassisSpeedsFieldRelative();
		}
		return drivetrain.getState().Speeds;
	}

	/**
	 * Get the drivetrain states.
	 *
	 * @return the swerve module states
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/States/Measured")
	public SwerveModuleState[] getModuleStates() {
		return drivetrain.getState().ModuleStates;
	}

	/**
	 * Get the drivetrain targets.
	 *
	 * @return drivetrain targets
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/States/Targets")
	public SwerveModuleState[] getModuleTargets() {
		return drivetrain.getState().ModuleTargets;
	}

	/**
	 * Get the drivetrain module positions.
	 *
	 * @return the module positions
	 */
	@AutoLogOutput(key = "Drivetrain/Swerve/Positions")
	public SwerveModulePosition[] getModulePositions() {
		if (Robot.isSimulation() && Features.MAPLE_SIM_ENABLED) {
			SwerveModuleSimulation[] simModules = simDrivetrain.getMapleSimDrive().getModules();
			var positions = new SwerveModulePosition[simModules.length];

			for (int i = 0; i < positions.length; i++) {
				double wheelRotations = simModules[i].getDriveWheelFinalPosition().in(Rotations);
				double distance =
					wheelRotations
					* TunerConstants.WHEEL_RADIUS.times(2 * Math.PI).in(Meters);
				Rotation2d angle = simModules[i].getSteerAbsoluteFacing();

				positions[i] = new SwerveModulePosition(distance, angle);
			}

			return positions;
		}
		return drivetrain.getState().ModulePositions;
	}

	/**
	 * Get the target alignment pose.
	 *
	 * @return Target alignment pose
	 */
	@AutoLogOutput(key = "Drivetrain/ReefSelectorTarget")
	public Pose2d getTargetPose() {
		return targetPose;
	}

	/**
	 * Get the maple sim drivetrain.
	 * @return the maple sim swerve drivetrain
	 */
	public MapleSimSwerveDrivetrain getSimDrivetrain() {
		return simDrivetrain;
	}
	/**
	 * Return ATs for the test field.
	 *
	 * @return ATs for the test field.
	 */
	@AutoLogOutput(key = "Drivetrain/TestFieldATs")
	public Pose3d[] logTestFieldATs() {
		return Features.USE_TEST_FIELD ? new Pose3d[] {
				TAG_LAYOUT.getTagPose(VisionConstants.TAG_ID_TEST_REEF_RIGHT)
						.orElse(new Pose3d()),
				TAG_LAYOUT.getTagPose(VisionConstants.TAG_ID_TEST_REEF_LEFT)
						.orElse(new Pose3d()),
				TAG_LAYOUT.getTagPose(VisionConstants.TAG_ID_TEST_REEF_RIGHT)
						.orElse(new Pose3d()) }
				: new Pose3d[] {};
	}

	/**
	 * Get the drivetrain.
	 *
	 * @return the drivetrain
	 */
	public CommandSwerveDrivetrain getDrivetrain() {
		return drivetrain;
	}

	/**
	 * Adds a new timestamped vision measurement.
	 *
	 * @param visionPoseMeters The pose of the robot in the camera's coordinate
	 *                         frame
	 * @param timestampSeconds The timestamp of the measurement
	 * @param visionStdDevs    The standard deviations of the measurement in the x,
	 *                         y, and theta directions
	 */
	public void addVisionMeasurement(
			Pose2d visionPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionStdDevs) {
		drivetrain.addVisionMeasurement(
				visionPoseMeters,
				timestampSeconds,
				visionStdDevs);
	}
}
