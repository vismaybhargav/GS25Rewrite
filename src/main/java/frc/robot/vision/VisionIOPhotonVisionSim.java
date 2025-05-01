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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import static frc.robot.Constants.VisionConstants.REEF_CAMERA_DIST_COEFFS;
import static frc.robot.Constants.VisionConstants.REEF_CAMERA_INTRINSICS;
import static frc.robot.Constants.VisionConstants.REEF_CAMERA_RES_HEIGHT;
import static frc.robot.Constants.VisionConstants.REEF_CAMERA_RES_WIDTH;
import static frc.robot.Constants.VisionConstants.STATION_CAMERA_DIST_COEFFS;
import static frc.robot.Constants.VisionConstants.STATION_CAMERA_INTRINSICS;
import static frc.robot.Constants.VisionConstants.STATION_CAMERA_RES_HEIGHT;
import static frc.robot.Constants.VisionConstants.STATION_CAMERA_RES_WIDTH;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

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

		if ("Reef_Camera".equals(name)) {
			cameraProps.setCalibration(
				REEF_CAMERA_RES_WIDTH,
				REEF_CAMERA_RES_HEIGHT,
				REEF_CAMERA_INTRINSICS,
				REEF_CAMERA_DIST_COEFFS
			);
		} else if ("Source_Camera".equals(name)) {
			cameraProps.setCalibration(
				STATION_CAMERA_RES_WIDTH,
				STATION_CAMERA_RES_HEIGHT,
				STATION_CAMERA_INTRINSICS,
				STATION_CAMERA_DIST_COEFFS
			);
		}

		cameraSim = new PhotonCameraSim(getCamera(), cameraProps, TAG_LAYOUT);
		visionSystemSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSystemSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}
