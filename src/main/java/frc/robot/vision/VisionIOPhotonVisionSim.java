// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
	private static VisionSystemSim visionSystemSim;
	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	/**
	 * Creates a new VisionIOPhotonVisionSim.
	 *
	 * @param name         The name of the camera.
	 * @param robotToCamera The transform from the robot to the camera.
	 * @param pose2dSupplier Supplier for the robot pose to use in simulation.
	 */
	public VisionIOPhotonVisionSim(
			String name,
			Transform3d robotToCamera,
			Supplier<Pose2d> pose2dSupplier
	) {
		super(name, robotToCamera);
		this.poseSupplier = pose2dSupplier;

		if (visionSystemSim == null) {
			visionSystemSim = new VisionSystemSim("main");
			visionSystemSim.addAprilTags(TAG_LAYOUT);
			SmartDashboard.putData(visionSystemSim.getDebugField());
		}

		var cameraProps = new SimCameraProperties();
		cameraProps.setFPS(100);

		// if ("Reef_Camera".equals(name)) {
		// 	cameraProps.setCalibration(
		// 			640,
		// 			480,
		// 			new Matrix<N3, N3>(N3.instance, N3.instance,
		// 					new double[] {
		// 							554.8363329613238,
		// 							0.0,
		// 							319.771006175582,
		// 							0.0,
		// 							555.7640379607542,
		// 							210.90231168898111,
		// 							0.0,
		// 							0.0,
		// 							1.0
		// 					}),
		// 			new Matrix<N8, N1>(N8.instance, N1.instance, new double[] {
		// 					0.032904169887820925,
		// 					0.024981667114235325,
		// 					-0.0024512685439365967,
		// 					9.347928373666906E-4,
		// 					-0.15993971100687385,
		// 					-2.8908154357146817E-4,
		// 					1.516375932970693E-4,
		// 					0.006735034604041476
		// 			}));
		// } else if ("Source_Camera".equals(name)) {
		// 	cameraProps.setCalibration(
		// 			640,
		// 			480,
		// 			new Matrix<N3, N3>(N3.instance, N3.instance,
		// 					new double[] {
		// 							548.8107781815636,
		// 							0.0,
		// 							335.98845208944647,
		// 							0.0,
		// 							549.91022315822,
		// 							261.5076314193876,
		// 							0.0, 0.0, 1.0
		// 					}),
		// 			new Matrix<N8, N1>(N8.instance, N1.instance,
		// 					new double[] {
		// 							0.046882076180144325,
		// 							-0.08739491623632688,
		// 							-7.369602850193537E-4,
		// 							9.49279422750342E-4,
		// 							0.015437967521711683,
		// 							-0.0018478126980591776,
		// 							0.004435053264404992,
		// 							-1.8178696218760975E-4
		// 					}));
		// }

		cameraSim = new PhotonCameraSim(getCamera(), cameraProps, TAG_LAYOUT);
		visionSystemSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSystemSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}
