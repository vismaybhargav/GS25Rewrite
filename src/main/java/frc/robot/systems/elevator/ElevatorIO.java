package frc.robot.systems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
	@AutoLog
	class ElevatorIOInputs {
		private double positionTicks = 0;
		private double positionInches = 0;
		private double velocityInPerSec = 0;
		private double accelerationInPerSec2 = 0;
		private double appliedVoltage = 0;
		private double currentAmps = 0;

		/**
		 * Gets the value of positionTicks.
		 * @return The value of positionTicks.
		 */
		public double getPositionTicks() {
			return positionTicks;
		}

		/**
		 * Gets the value of positionInches.
		 * @return The value of positionInches.
		 */
		public double getPositionInches() {
			return positionInches;
		}

		/**
		 * Gets the value of velocityInPerSec.
		 * @return The value of velocityInPerSec.
		 */
		public double getVelocityInPerSec() {
			return velocityInPerSec;
		}

		/**
		 * Gets the value of accelerationInPerSec2.
		 * @return The value of accelerationInPerSec2.
		 */
		public double getAccelerationInPerSec2() {
			return accelerationInPerSec2;
		}

		/**
		 * Gets the value of appliedVoltage.
		 * @return The value of appliedVoltage.
		 */
		public double getAppliedVoltage() {
			return appliedVoltage;
		}

		/**
		 * Gets the value of currentAmps.
		 * @return The value of currentAmps.
		 */
		public double getCurrentAmps() {
			return currentAmps;
		}

		/**
		 * Sets the value of positionTicks.
		 * @param newPositionTicks The value to set positionTicks to.
		 */
		public void setPositionTicks(double newPositionTicks) {
			positionTicks = newPositionTicks;
		}

		/**
		 * Sets the value of positionInches.
		 * @param newPositionInches The value to set positionInches to.
		 */
		public void setPositionInches(double newPositionInches) {
			positionInches = newPositionInches;
		}

		/**
		 * Sets the value of velocityInPerSec.
		 * @param newVelocityInPerSec The value to set velocityInPerSec to.
		 */
		public void setVelocityInPerSec(double newVelocityInPerSec) {
			velocityInPerSec = newVelocityInPerSec;
		}

		/**
		 * Sets the value of accelerationInPerSec2.
		 * @param newAccelerationInPerSec2 The value to set accelerationInPerSec2 to.
		 */
		public void setAccelerationInPerSec2(double newAccelerationInPerSec2) {
			accelerationInPerSec2 = newAccelerationInPerSec2;
		}

		/**
		 * Sets the value of appliedVoltage.
		 * @param newAppliedVoltage The value to set appliedVoltage to.
		 */
		public void setAppliedVoltage(double newAppliedVoltage) {
			appliedVoltage = newAppliedVoltage;
		}

		/**
		 * Sets the value of currentAmps.
		 * @param newCurrentAmps The value to set currentAmps to.
		 */
		public void setCurrentAmps(double newCurrentAmps) {
			currentAmps = newCurrentAmps;
		}
	}

	/**
	 * Sets the elevator voltage.
	 * @param voltage Amount of voltage to apply to the motor, in volts.
	 */
	default void setVoltage(double voltage) { }

	/**
	 * Updates set of loggable inputs.
	 * @param inputs The set of logged inputs to write to.
	 */
	default void updateLogging(ElevatorIOInputs inputs) { }
}
