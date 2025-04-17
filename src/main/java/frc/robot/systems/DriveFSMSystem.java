package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CommandSwerveDrivetrain;

// WPILib Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.generated.LocalADStarAK;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DriveConstants;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum DriveFSMState {
		TELEOP,
		PRE_PATHFIND,
		PATHFIND
	}

	private static final LinearVelocity MAX_SPEED = TunerConstants.SPEED_AT_12_VOLTS;
		// kSpeedAt12Volts desired top speed
	private static final AngularVelocity MAX_ANGULAR_RATE = DriveConstants.MAX_ANGULAR_VELO_RPS;
		//3/4 rps angle velo

	/* ======================== Private variables ======================== */
	private DriveFSMState currentState;
	private CommandSwerveDrivetrain drivetrain;
	private SwerveDrivePoseEstimator poseEstimator;

	private final SwerveRequest.FieldCentric drive
		= new SwerveRequest.FieldCentric()
		.withDeadband(MAX_SPEED.in(MetersPerSecond)
		* DriveConstants.TRANSLATION_DEADBAND) // 4% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
		* DriveConstants.ROTATION_DEADBAND) //4% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle
		= new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MAX_SPEED.in(MetersPerSecond)
		* DriveConstants.TRANSLATION_DEADBAND) // 4% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
		* DriveConstants.ROTATION_DEADBAND) //4% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.RobotCentric driveRobotCentric
		= new SwerveRequest.RobotCentric()
		.withDeadband(MAX_SPEED.in(MetersPerSecond)
			* DriveConstants.TRANSLATION_DEADBAND) // 4% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE.in(RadiansPerSecond)
			* DriveConstants.ROTATION_DEADBAND) //4% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
		new SwerveRequest.ApplyFieldSpeeds();
	private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
		new SwerveRequest.ApplyRobotSpeeds();

	/* ======================== Pathfinding Stuffs ================== */
	private final Timer timer = new Timer();
	private double timeOffset = 0;
	private Pose2d targetPose = new Pose2d();
	private Pose2d originalTargetPose = new Pose2d(
		targetPose.getTranslation(), targetPose.getRotation()
	);
	private GoalEndState goalEndState = new GoalEndState(0, targetPose.getRotation());
	private PathPlannerTrajectory currentTrajectory = null;
	private PathPlannerPath currentPath = null;
	private boolean finish = true;

	private PathConstraints pathConstraints = new PathConstraints(
		MAX_SPEED.in(MetersPerSecond),
		3,
		MAX_ANGULAR_RATE.in(RadiansPerSecond),
		Math.pow(MAX_ANGULAR_RATE.in(RadiansPerSecond), 2)
	);

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		drivetrain = TunerConstants.createDrivetrain();

		poseEstimator = new SwerveDrivePoseEstimator(
				drivetrain.getKinematics(),
				getPose().getRotation(),
				getModulePositions(),
				getPose());

		Pathfinding.setPathfinder(new LocalADStarAK());


		PathPlannerLogging.setLogActivePathCallback((path) -> {
			Logger.recordOutput("PathPlanner/Trajectory", path.toArray(new Pose2d[path.size()]));
		});

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public DriveFSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Get the current state's string value.
	 * @return current state string value
	 */
	@AutoLogOutput(key = "DriveFSM/Current State")
	public String getCurrentStateName() {
		return currentState.name();
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = DriveFSMState.TELEOP;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		Logger.recordOutput("Timer", timer.get());
		switch (currentState) {
			case TELEOP:
				handleTeleopState(input);
				break;
			case PATHFIND:
				handlePathfindState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 */
	public void updateAutonomous() { }

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private DriveFSMState nextState(TeleopInput input) {
		if (input == null) {
			return DriveFSMState.TELEOP;
		}

		switch (currentState) {
			case TELEOP:
				if (input.isPathfindButtonPressed()) {
					if (isPathfindingFinished()) {
						initalizePathfinding();
					}
					return DriveFSMState.PATHFIND;
				} else {
					return DriveFSMState.TELEOP;
				}
			case PATHFIND:
				if (input.isPathfindButtonPressed()) {
					return DriveFSMState.PATHFIND;
				} else {
					return DriveFSMState.TELEOP;
				}
			default:
				throw new IllegalStateException("Invalid State: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handles the TELEOP_STATE, when the robot is controlled by the driver.
	 * @param input the input of the driver controller
	 */
	public void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}

		finish = true;  // Resets the pathfinding, so that we
						// re-initialize when going into the pathfinding state.

		double xSpeed = MathUtil.applyDeadband(
			-input.getDriveLeftJoystickY(),
			DriveConstants.TRANSLATION_DEADBAND
		) * MAX_SPEED.in(MetersPerSecond);

		double ySpeed = MathUtil.applyDeadband(
			-input.getDriveLeftJoystickX(),
			DriveConstants.TRANSLATION_DEADBAND
		) * MAX_SPEED.in(MetersPerSecond);

		double thetaSpeed = MathUtil.applyDeadband(
			-input.getDriveRightJoystickX(),
			DriveConstants.ROTATION_DEADBAND
		) * MAX_ANGULAR_RATE.in(RadiansPerSecond);

		drivetrain.setControl(
			drive
				.withVelocityX(xSpeed)
				.withVelocityY(ySpeed)
				.withRotationalRate(thetaSpeed)
		);
	}

	/**
	 * Handles the PATHFIND_STATE, when the robot is pathfinding to it's target pose.
	 * @param input drive controller input
	 */
	public void handlePathfindState(TeleopInput input) {
		if (finish) {
			return;
		}

		Pose2d currPose = getPose();
		ChassisSpeeds currSpeeds = getChassisSpeeds();

		// Skip updates if we are very close to the goal
		boolean skipUpdates =
			currentTrajectory != null
				&& currPose
					.getTranslation()
					.getDistance(currentTrajectory.getEndState().pose.getTranslation()) < 2.0;

		if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
			currentPath = Pathfinding.getCurrentPath(pathConstraints, goalEndState);

			if (currentPath != null) {
				currentTrajectory = new PathPlannerTrajectory(
						currentPath, currSpeeds, currPose.getRotation(), drivetrain.getPPConfig());
				if (!Double.isFinite(currentTrajectory.getTotalTimeSeconds())) {
					finish = true;
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

				double d = closestState1
					.pose.getTranslation().getDistance(closestState2.pose.getTranslation());
				double t =
					(currPose
						.getTranslation().getDistance(closestState1.pose.getTranslation())) / d;
				t = MathUtil.clamp(t, 0.0, 1.0);

				timeOffset =
					MathUtil.interpolate(closestState1.timeSeconds, closestState2.timeSeconds, t);

				// If the robot is stationary and at the start of the path, set the time offset
				// to the next loop
				// This can prevent an issue where the robot will remain stationary if new paths
				// come in every loop
				if (timeOffset <= 0.02
						&& Math.hypot(
							currSpeeds.vxMetersPerSecond,
							currSpeeds.vyMetersPerSecond) < 0.1) {
					timeOffset = 0.02;
				}

				PathPlannerLogging.logActivePath(currentPath);
			}

			timer.reset();
			timer.start();
		}

		if (currentTrajectory != null) {
			var targetState = currentTrajectory.sample(timer.get() + timeOffset);

			ChassisSpeeds targSpeeds =
				drivetrain.getPPHolonomicDriveController()
					.calculateRobotRelativeSpeeds(currPose, targetState);

			drivetrain.setControl(
				pathApplyRobotSpeeds
					.withSpeeds(targSpeeds)
					.withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesX())
					.withWheelForceFeedforwardsY(targetState.feedforwards.robotRelativeForcesY())
			);
		}
	}

	/**
	 * Does all the setup before pathfinding starts.
	 */
	public void initalizePathfinding() {
		currentTrajectory = null;
		finish = false;
		timeOffset = 0;

		Pose2d currentPose = getPose();
		drivetrain.getPPHolonomicDriveController().reset(currentPose, getChassisSpeeds());

		if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.5) {
			var ff = DriveFeedforwards.zeros(4);

			drivetrain.setControl(
				pathApplyRobotSpeeds
				.withSpeeds(getChassisSpeeds())
				.withWheelForceFeedforwardsX(ff.robotRelativeForcesX())
				.withWheelForceFeedforwardsY(ff.robotRelativeForcesY())
			);

			finish = true;
		} else {
			Pathfinding.setStartPosition(currentPose.getTranslation());
			Pathfinding.setGoalPosition(targetPose.getTranslation());
		}
	}

	/**
	 * Determine if the pathfinding is done.
	 * @return true if the pathfinding is done
	 */
	@AutoLogOutput(key = "Pathfinding finished")
	public boolean isPathfindingFinished() {
		if (finish) {
			return true;
		}

		if (currentTrajectory != null) {
			return timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() - timeOffset);
		}

		return false;
	}

	/**
	 * Get the pose of the drivetrain.
	 * @return pose of the drivetrain
	 */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return drivetrain.getState().Pose;
	}

	/**
	 * Get the chassis speeds of the drivetrain.
	 * @return the drivetrain chassis speeds
	 */
	@AutoLogOutput(key = "Swerve/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		return drivetrain.getState().Speeds;
	}

	/**
	 * Get the drivetrain states.
	 * @return the swerve module states
	 */
	@AutoLogOutput(key = "Swerve/States/Measured")
	public SwerveModuleState[] getModuleStates() {
		return drivetrain.getState().ModuleStates;
	}

	/**
	 * Get the drivetrain targets.
	 * @return drivetrain targets
	 */
	@AutoLogOutput(key = "Swerve/States/Targets")
	public SwerveModuleState[] getModuleTargets() {
		return drivetrain.getState().ModuleTargets;
	}

	/**
	 * Get the drivetrain module positions.
	 * @return the module positions
	 */
	@AutoLogOutput(key = "Swerve/Positions")
	public SwerveModulePosition[] getModulePositions() {
		return drivetrain.getState().ModulePositions;
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
		poseEstimator.addVisionMeasurement(
				visionPoseMeters,
				timestampSeconds,
				visionStdDevs);
	}
}
