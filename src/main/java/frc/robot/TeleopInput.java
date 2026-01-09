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

	public enum ControllerDPad {
		UP(0),
		DOWN(180),
		LEFT(270),
		RIGHT(90),
		TOP_LEFT(315),
		TOP_RIGHT(45),
		BOTTOM_LEFT(225),
		BOTTOM_RIGHT(135),
		NONE(-1);

		private final int code;

		/**
		 * Constructor for DPad direction with no code (e.g., BOTTOM_LEFT, BOTTOM_RIGHT).
		 * @param theCode
		 */
		ControllerDPad(int theCode) {
			code = theCode;
		}

		/**
		 * Get the code associated with the DPad direction.
		 * @return The code
		 */
		public int getCode() {
			return code;
		}
	}

	/* ======================== Private variables ======================== */
	// Input objects
	private XboxController driveController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new XboxController(DRIVE_CONTROLLER_PORT);
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
		return driveController.getBButton();
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

	/**
	 * Checks if the seed button is pressed.
	 * @return true if the seed button is pressed, false otherwise
	 */
	public boolean isSeedButtonPressed() {
		return driveController.getAButtonPressed();
	}

	/**
	 * Checks if the elevator up button is pressed.
	 * @return true if the elevator up button is pressed, false otherwise
	 */
	public boolean isElevatorUpPressed() {
		return driveController.getPOV() == ControllerDPad.UP.getCode();
	}

	/**
	 * Checks if the elevator down button is pressed.
	 * @return true if the elevator down button is pressed, false otherwise
	 */
	public boolean isElevatorDownPressed() {
		return driveController.getPOV() == ControllerDPad.DOWN.getCode();
	}

	/**
	 * Checks if the L4 button is pressed.
	 * @return true if the L4 button is pressed, false otherwise
	 */
	public boolean isL4ButtonPressed() {
		return driveController.getYButtonPressed();
	}

	/**
	 * Checks if the L3 button is pressed.
	 * @return true if the L3 button is pressed, false otherwise
	 */
	public boolean isL3ButtonPressed() {
		return driveController.getXButtonPressed();
	}

	/**
	 * Checks if the L2 button is pressed.
	 * @return true if the L2 button is pressed, false otherwise
	 */
	public boolean isL2ButtonPressed() {
		return driveController.getBButtonPressed();
	}

	/**
	 * Checks if the ground button is pressed.
	 * @return true if the ground button is pressed, false otherwise
	 */
	public boolean isGroundButtonPressed() {
		return driveController.getAButtonPressed();
	}

	/* ======================== Private methods ======================== */

}
