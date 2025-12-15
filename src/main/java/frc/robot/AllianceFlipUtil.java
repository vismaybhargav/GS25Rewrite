// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
	/**
	 * Applies alliance-based flipping to an X coordinate.
	 * @param x The original X coordinate
	 * @return The flipped X coordinate if on Red alliance, otherwise the original X coordinate
	 */
	public static Distance applyX(Distance x) {
		return shouldFlip() ? Meters.of(TAG_LAYOUT.getFieldLength()).minus(x) : x;
	}

	/**
	 * Applies alliance-based flipping to a Y coordinate.
	 * @param y The original Y coordinate
	 * @return The flipped Y coordinate if on Red alliance, otherwise the original Y coordinate
	 */
	public static Distance applyY(Distance y) {
		return shouldFlip() ? Meters.of(TAG_LAYOUT.getFieldWidth()).minus(y) : y;
	}

	/**
	 * Applies alliance-based flipping to a Translation2d.
	 * @param translation The original Translation2d
	 * @return The flipped Translation2d if on Red alliance, otherwise the original Translation2d
	 */
	public static Translation2d apply(Translation2d translation) {
		return new Translation2d(
			applyX(translation.getMeasureX()), applyY(translation.getMeasureY()));
	}

	/**
	 * Applies alliance-based flipping to a Rotation2d.
	 * @param rotation The original Rotation2d
	 * @return The flipped Rotation2d if on Red alliance, otherwise the original Rotation2d
	 */
	public static Rotation2d apply(Rotation2d rotation) {
		return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
	}

	/**
	 * Applies alliance-based flipping to a Pose2d.
	 * @param pose The original Pose2d
	 * @return The flipped Pose2d if on Red alliance, otherwise the original Pose2d
	 */
	public static Pose2d apply(Pose2d pose) {
		return shouldFlip()
				? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
				: pose;
	}

	/**
	 * Determines whether to flip coordinates based on alliance color.
	 * @return true if the alliance is Red, false otherwise
	 */
	public static boolean shouldFlip() {
		return DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
	}
}
