package frc.robot.simulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 *
 *
 * <h2>Injects Maple-Sim simulation data into a CTRE swerve drivetrain.</h2>
 *
 * <p>This class retrieves simulation data from Maple-Sim and injects it into the CTRE
 * {@link com.ctre.phoenix6.swerve.SwerveDrivetrain} instance.
 *
 * <p>It replaces the {@link com.ctre.phoenix6.swerve.SimSwerveDrivetrain} class.
 */
public class MapleSimSwerveDrivetrain {
	private final Pigeon2SimState pigeonSim;
	private final SimSwerveModule[] simModules;
	public final SwerveDriveSimulation mapleSimDrive;

	/**
	 * Constructs a drivetrain simulation using the specified parameters.
	 *
	 * @param config the config
	 */
	public MapleSimSwerveDrivetrain(
			SimSwerveDrivetrainConfig config) {
		this.pigeonSim = config.getPigeon().getSimState();
		simModules = new SimSwerveModule[config.getModuleConstants().length];

		var moduleConstants = config.getModuleConstants();
		DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
				.withRobotMass(config.getRobotMass())
				.withBumperSize(config.getBumperLength(), config.getBumperWidth())
				.withGyro(COTS.ofPigeon2())
				.withCustomModuleTranslations(config.getModuleLocations())
				.withSwerveModule(new SwerveModuleSimulationConfig(
						config.getModuleDriveMotor(),
						config.getModuleSteerMotor(),
						moduleConstants[0].DriveMotorGearRatio,
						moduleConstants[0].SteerMotorGearRatio,
						Volts.of(moduleConstants[0].DriveFrictionVoltage),
						Volts.of(moduleConstants[0].SteerFrictionVoltage),
						Meters.of(moduleConstants[0].WheelRadius),
						KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
						config.getWheelCOF()));
		mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

		SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
		for (int i = 0; i < this.simModules.length; i++) {
			simModules[i] = new SimSwerveModule(
					moduleConstants[0], moduleSimulations[i], config.getModules()[i]);
		}

		SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
	}

	/**
	 * Update the simulation.
	 *
	 * Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices,
	 * including motors and
	 * the IMU.
	 */
	public void update() {
		SimulatedArena.getInstance().simulationPeriodic();
		pigeonSim.setRawYaw(
				mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
		pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
				mapleSimDrive
					.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
	}

	/**
	 *
	 *
	 * <h1>Represents the simulation of a single {@link SwerveModule}.</h1>
	 */
	protected static class SimSwerveModule {
		public final SwerveModuleConstants<
			TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			moduleConstant;
		public final SwerveModuleSimulation moduleSimulation;

		public SimSwerveModule(
				SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
					moduleConstant,
				SwerveModuleSimulation moduleSimulation,
				SwerveModule<TalonFX, TalonFX, CANcoder> module) {
			this.moduleConstant = moduleConstant;
			this.moduleSimulation = moduleSimulation;

			moduleSimulation
				.useDriveMotorController(new TalonFXMotorControllerSim(module.getDriveMotor()));
			moduleSimulation.useSteerMotorController(
					new TalonFXMotorControllerWithRemoteCanCoderSim(
						module.getSteerMotor(), module.getEncoder()));
		}
	}

	// Static utils classes
	public static class TalonFXMotorControllerSim implements SimulatedMotorController {
		public final int id;

		private final TalonFXSimState talonFXSimState;

		/**
		 * Constructs a new TalonFXMotorControllerSim.
		 * @param talonFX the motor
		 */
		public TalonFXMotorControllerSim(TalonFX talonFX) {
			this.id = talonFX.getDeviceID();
			this.talonFXSimState = talonFX.getSimState();
		}

		@Override
		public Voltage updateControlSignal(
				Angle mechanismAngle,
				AngularVelocity mechanismVelocity,
				Angle encoderAngle,
				AngularVelocity encoderVelocity) {
			talonFXSimState.setRawRotorPosition(encoderAngle);
			talonFXSimState.setRotorVelocity(encoderVelocity);
			talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

			return talonFXSimState.getMotorVoltageMeasure();
		}
	}

	public static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
		private final int encoderId;
		private final CANcoderSimState remoteCancoderSimState;

		public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
			super(talonFX);
			this.remoteCancoderSimState = cancoder.getSimState();

			this.encoderId = cancoder.getDeviceID();
		}

		@Override
		public Voltage updateControlSignal(
				Angle mechanismAngle,
				AngularVelocity mechanismVelocity,
				Angle encoderAngle,
				AngularVelocity encoderVelocity) {
			remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
			remoteCancoderSimState.setRawPosition(mechanismAngle);
			remoteCancoderSimState.setVelocity(mechanismVelocity);

			return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
		}
	}

	/**
	 *
	 *
	 * Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.
	 *
	 * This method processes an array of {@link SwerveModuleConstants}
	 * to apply necessary adjustments for simulation
	 * purposes, ensuring compatibility and avoiding known bugs.
	 *
	 * @param moduleConstants the module cosntants
	 *
	 * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
	 * @return the swerve module constants
	 */
	public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
			SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
		for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
			regulateModuleConstantForSimulation(moduleConstant);
		}

		return moduleConstants;
	}

	/**
	 *
	 *
	 * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
	 *
	 * This method applies specific adjustments to the {@link SwerveModuleConstants}
	 * for simulation purposes. These
	 * changes have no effect on real robot operations and address known simulation bugs:
	 *
	 *   Inverted Drive Motors: Prevents drive PID issues caused by inverted configurations
	 *   Non-zero CanCoder Offsets: Fixes potential module state optimization issues.
	 *   Steer Motor PID: Adjusts PID values tuned for real robots to improve simulation
	 *       performance.
	 *
	 * Note:This function is skipped when running on a real robot,
	 * ensuring no impact on constants used on real
	 * robot hardware.
	 *
	 * @param moduleConstants the module constants
	 */
	private static void regulateModuleConstantForSimulation(
		SwerveModuleConstants<?, ?, ?> moduleConstants) {
		// Skip regulation if running on a real robot
		if (RobotBase.isReal()) {
			return;
		}

		// Apply simulation-specific adjustments to module constants
		moduleConstants
				// Disable encoder offsets
				.withEncoderOffset(0)
				// Disable motor inversions for drive and steer motors
				.withDriveMotorInverted(false)
				.withSteerMotorInverted(false)
				// Disable CanCoder inversion
				.withEncoderInverted(false)
				// Adjust steer motor PID gains for simulation
				.withSteerMotorGains(new Slot0Configs()
						.withKP(70)
						.withKI(0)
						.withKD(4.5)
						.withKS(0)
						.withKV(1.91)
						.withKA(0)
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
				.withSteerMotorGearRatio(16.0)
				// Adjust friction voltages
				.withDriveFrictionVoltage(Volts.of(0.1))
				.withSteerFrictionVoltage(Volts.of(0.05))
				// Adjust steer inertia
				.withSteerInertia(KilogramSquareMeters.of(0.05));
	}
}
