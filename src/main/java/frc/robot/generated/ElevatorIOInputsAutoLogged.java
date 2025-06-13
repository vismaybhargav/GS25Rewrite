package frc.robot.generated;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.systems.elevator.ElevatorIO;

public class ElevatorIOInputsAutoLogged
	extends ElevatorIO.ElevatorIOInputs
	implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("PositionTicks", getPositionTicks());
		table.put("PositionInches", getPositionInches());
		table.put("VelocityInPerSec", getVelocityInPerSec());
		table.put("AccelerationInPerSec2", getAccelerationInPerSec2());
		table.put("AppliedVoltage", getAppliedVoltage());
		table.put("CurrentAmps", getCurrentAmps());
	}

	@Override
	public void fromLog(LogTable table) {
		setPositionTicks(table.get("PositionTicks", getPositionTicks()));
		setPositionInches(table.get("PositionInches", getPositionInches()));
		setVelocityInPerSec(table.get("VelocityInPerSec", getVelocityInPerSec()));
		setAccelerationInPerSec2(table.get("AccelerationInPerSec2", getAccelerationInPerSec2()));
		setAppliedVoltage(table.get("AppliedVoltage", getAppliedVoltage()));
		setCurrentAmps(table.get("CurrentAmps", getCurrentAmps()));
	}

	/**
	 * Makes a clone object.
	 * @return A clone of the ElevatorIOInputsAutoLogged object.
	 */
	public ElevatorIOInputsAutoLogged clone() {
		ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
		copy.setPositionTicks(this.getPositionTicks());
		copy.setPositionInches(this.getPositionInches());
		copy.setVelocityInPerSec(this.getVelocityInPerSec());
		copy.setAccelerationInPerSec2(this.getAccelerationInPerSec2());
		copy.setAppliedVoltage(this.getAppliedVoltage());
		copy.setCurrentAmps(this.getCurrentAmps());
		return copy;
	}
}
