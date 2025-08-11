package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
    // Elevator Maps
    public static final int ELEVATOR_CAN_ID = 14;

    // Elevator Limit Switch DIO Ports
    public static final int ELEVATOR_GROUND_LIMIT_SWITCH_DIO_PORT = 0; // FINAL
    public static final int ELEVATOR_TOP_LIMIT_SWITCH_DIO_PORT = 1; // FINAL

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);
	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}
}
