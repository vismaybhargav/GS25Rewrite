package frc.robot.generated;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.vision.VisionIO;

public class VisionIOInputsAutoLogged
	extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("LatestPoseObservations", getLatestPoseObservations());
		table.put("TagIDs", getTagIDs());
	}

	@Override
	public void fromLog(LogTable table) {
		setLatestPoseObservations(
			table.get("LatestPoseObservations", getLatestPoseObservations())
		);
		setTagIDs(
			table.get("TagIDs", getTagIDs())
		);
	}

	/**
	 * Create a deep copy.
	 * @return the deep copy
	 */
	public VisionIOInputsAutoLogged clone() {
		VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
		copy.setLatestPoseObservations(getLatestPoseObservations().clone());
		copy.setTagIDs(getTagIDs().clone());
		return copy;
	}
}
