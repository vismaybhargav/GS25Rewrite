package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
// WPILib Imports

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private XboxController driveController;
	private XboxController mechController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new XboxController(DRIVE_CONTROLLER_PORT);
		mechController = new XboxController(MECH_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getDriveLeftJoystickX() {
		return driveController.getLeftX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getDriveLeftJoystickY() {
		return driveController.getLeftY();
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getDriveRightJoystickX() {
		return driveController.getRightX();
	}

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getDriveRightJoystickY() {
		return driveController.getRightY();
	}

	/* ------------------------ Buttons ------------------------ */
	/**
	 * is the pathfinding button pressed.
	 * @return the pathfinding button pressed
	 */
	public boolean isPathfindButtonPressed() {
		return driveController.getXButton();
	}

	/**
	 * Checks if the button to shift the reef selection
	 * counter clockwise is pressed.
	 * @return true if the button is pressed, false otherwise
	 */
	public boolean isCCWReefSelectionChangeButtonPressed() {
		return driveController.getLeftBumperButtonPressed();
	}

	/**
	 * Checks if the button to shift the reef selection
	 * clockwise is pressed.
	 * @return true if the button is pressed, false otherwise
	 */
	public boolean isCWReefSelectionChangeButtonPressed() {
		return driveController.getRightBumperButtonPressed();
	}

	public boolean isGroundButtonPressed() {
		return mechController.getAButton();
	}

	public boolean isL2ButtonPressed() {
		return mechController.getXButton();
	}

	public boolean isL3ButtonPressed() {
		return mechController.getBButton();
	}

	public boolean isL4ButtonPressed() {
		return mechController.getYButton();
	}

	public double getManualElevatorMovementInput() {
		return mechController.getLeftY();
	}

	/* ======================== Private methods ======================== */

}
