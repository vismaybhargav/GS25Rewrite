package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
	private static VisionSystemSim visionSystemSim;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	/**
	 * Creates a new VisionIOPhotonVision object.
	 *
	 * @param name name of the source
	 * @param robotToCamera robot 2 cam transform
	 * @param thePoseSupplier the pose supplier
	 *
	 */
	public VisionIOPhotonVisionSim(
			String name,
			Transform3d robotToCamera,
			Supplier<Pose2d> thePoseSupplier
	) {
		super(name, robotToCamera);
		poseSupplier = thePoseSupplier;

		if (visionSystemSim == null) {
			visionSystemSim = new VisionSystemSim("main");
			visionSystemSim.addAprilTags(TAG_LAYOUT);
		}

		var cameraProps = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(getCamera(), cameraProps, TAG_LAYOUT);
		visionSystemSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSystemSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}
