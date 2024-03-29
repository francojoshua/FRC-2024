// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class SwerveConstants {
		public static final double kPAngle = 0.1;

		public static final double kWheelDiameter = Units.inchesToMeters(4.0);
		public static final double kDriveMotorGearRatio = 1 / 6.75;
		public static final double kAngleMotorGearRatio = 1 / (150.0 / 7.0);

		// Calculate circumference (PI * Diameter) and divide it by the gear ratio.
		public static final double kDrivePositionConversionFactor =
				kDriveMotorGearRatio * Math.PI * kWheelDiameter;
		public static final double kDriveVelocityConversionFactor =
				kDrivePositionConversionFactor / 60;

		public static final double kAnglePositionConversionFactor =
				kAngleMotorGearRatio * Math.PI * 2;
		public static final double kAngleVelocityConversionFactor =
				kAnglePositionConversionFactor / 60;


	}

	public static class DriveConstants {

		public static final double kTrackWidth = Units.inchesToMeters(24);
		// Distance between right and left wheels
		public static final double kWheelBase = Units.inchesToMeters(24);
		// Distance between front and back wheels
		public static final SwerveDriveKinematics kDriveKinematics =
				new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front
																								// right
						new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
						new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // back right
						new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // back left

		// POSITIVE Y MEANS LEFT. POSITIVE X MEANS FRONT

		public static final int kFrontRightDriveMotorPort = 2;
		public static final int kBackRightDriveMotorPort = 4;
		public static final int kFrontLeftDriveMotorPort = 6;
		public static final int kBackLeftDriveMotorPort = 8;

		public static final int kFrontRightAngleMotorPort = 1;
		public static final int kBackRightAngleMotorPort = 3;
		public static final int kFrontLeftAngleMotorPort = 5;
		public static final int kBackLeftAngleMotorPort = 7;

		public static final boolean kInverseFrontLeftAngleEncoder = true;
		public static final boolean kInverseBackLeftAngleEncoder = true;
		public static final boolean kInverseFrontRightAngleEncoder = true;
		public static final boolean kInverseBackRightAngleEncoder = true;

		public static final boolean kInverseFrontLeftDriveEncoder = false; // GOOD
		public static final boolean kInverseBackLeftDriveEncoder = false; // GOOD
		public static final boolean kInverseFrontRightDriveEncoder = true; // GOOD
		public static final boolean kInverseBackRightDriveEncoder = true; // GOOD

		public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
		public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
		public static final int kBackRightDriveAbsoluteEncoderPort = 11;
		public static final int kBackLeftDriveAbsoluteEncoderPort = 12;

		public static final boolean kInverseFrontLeftDriveAbsoluteEncoder = false;
		public static final boolean kInverseBackLeftDriveAbsoluteEncoder = false;
		public static final boolean kInverseFrontRightDriveAbsoluteEncoder = false;
		public static final boolean kInverseBackRightDriveAbsoluteEncoder = false;

		public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  0.769775 * Math.PI * 2;
		public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.533447 * Math.PI * 2;
		public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.255371 * Math.PI * 2;
		public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.017822 * Math.PI * 2;

		public static final double kMaxSpeedMetersPerSecond = 5;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

		public static final boolean kIsFieldCentric = true;

	}

	public static class ControllerConstants {
		public static final double kDeadband = 0.05;

		public static final int Controller_ID = 0;
		public static final int LX_ID = 0, LY_ID = 1, RX_ID = 4, RY_ID = 5;
		public static final int UP = 0, RIGHT = 90, DOWN = 180, LEFT = 270;
		public static final int CONTROLLER1_ID = 0, CONTROLLER2_ID = 1;
		public static final int PCM_ID = 0;
	}
}
