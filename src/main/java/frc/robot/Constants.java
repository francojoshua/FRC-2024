// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.SwerveModuleConstants;

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
		// POSITIVE Y MEANS LEFT. POSITIVE X MEANS FRONT
		public static final SwerveDriveKinematics kDriveKinematics =
				new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front
																								// right
						new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
						new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // back right
						new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // back left


		// Front Right Module (0)
		public static class Mod0Constants {
			public static final int driveMotorId = 2;
			public static final int angleMotorId = 1;
			public static final int canCoderId = 10;

			public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.257568);

			public static final boolean invertDriveEncoder = true;
			public static final boolean invertAngleEncoder = true;
			public static final boolean invertCanCoder = false;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorId, angleMotorId, canCoderId, angleOffset,
							invertDriveEncoder, invertAngleEncoder, invertCanCoder);
		}

		// Front Left Module (1)
		public static class Mod1Constants {
			public static final int driveMotorId = 6;
			public static final int angleMotorId = 5;
			public static final int canCoderId = 9;

			public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.275391);

			public static final boolean invertDriveEncoder = true;
			public static final boolean invertAngleEncoder = true;
			public static final boolean invertCanCoder = false;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorId, angleMotorId, canCoderId, angleOffset,
							invertDriveEncoder, invertAngleEncoder, invertCanCoder);
		}

		// Back Right Module (2)
		public static class Mod2Constants {
			public static final int driveMotorId = 4;
			public static final int angleMotorId = 3;
			public static final int canCoderId = 11;

			public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.0197745);

			public static final boolean invertDriveEncoder = true;
			public static final boolean invertAngleEncoder = true;
			public static final boolean invertCanCoder = false;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorId, angleMotorId, canCoderId, angleOffset,
							invertDriveEncoder, invertAngleEncoder, invertCanCoder);
		}

		// Back Left Module (3)
		public static class Mod3Constants {
			public static final int driveMotorId = 8;
			public static final int angleMotorId = 7;
			public static final int canCoderId = 11;

			public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.406982);

			public static final boolean invertDriveEncoder = false;
			public static final boolean invertAngleEncoder = true;
			public static final boolean invertCanCoder = false;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorId, angleMotorId, canCoderId, angleOffset,
							invertDriveEncoder, invertAngleEncoder, invertCanCoder);
		}

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
