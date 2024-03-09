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
		public static final double kPAngle = 0.50;
		public static final double kIAngle = 0.0;
		public static final double kDAngle = 0.0;
		public static final double kFFAngle = 0.0;

		public static final double kPDrive = 0.2;
		public static final double kIDrive = 0.0;
		public static final double kDDrive = 0.0;
		public static final double kFFDrive = 1.0 / 5676.0;

		public static final boolean kIsPositionPIDWrappingEnabled = true;
		public static final double kPositionPIDWrappingMaxInput = 2 * Math.PI;
		public static final double kPositionPIDWrappingMinInput = 0.0;

		public static final double kOutputRangeMin = -1.0;
		public static final double kOutputRangeMax = 1.0;

		public static final int kDriveSmartCurrentLimit = 50;
		public static final int kAngleSmartCurrentLimit = 20;

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
		public static final int kFrontRightDriveMotorPort = 8;
		public static final int kFrontLeftDriveMotorPort = 4;
		public static final int kBackRightDriveMotorPort = 6;
		public static final int kBackLeftDriveMotorPort = 2;

		public static final int kFrontRightAngleMotorPort = 7;
		public static final int kFrontLeftAngleMotorPort = 3;
		public static final int kBackRightAngleMotorPort = 5;
		public static final int kBackLeftAngleMotorPort = 1;

		public static final boolean kInverseFrontRightAngleEncoder = true;
		public static final boolean kInverseFrontLeftAngleEncoder = true;
		public static final boolean kInverseBackRightAngleEncoder = true;
		public static final boolean kInverseBackLeftAngleEncoder = true;

		public static final boolean kInverseFrontRightDriveEncoder = true; // GOOD
		public static final boolean kInverseFrontLeftDriveEncoder = false; // GOOD
		public static final boolean kInverseBackRightDriveEncoder = true; // GOOD
		public static final boolean kInverseBackLeftDriveEncoder = false; // GOOD

		public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
		public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;
		public static final int kBackRightDriveAbsoluteEncoderPort = 9;
		public static final int kBackLeftDriveAbsoluteEncoderPort = 10;

		public static final boolean kInverseFrontRightDriveAbsoluteEncoder = false;
		public static final boolean kInverseFrontLeftDriveAbsoluteEncoder = false;
		public static final boolean kInverseBackRightDriveAbsoluteEncoder = false;
		public static final boolean kInverseBackLeftDriveAbsoluteEncoder = false;

		public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.722412 * Math.PI * 2;
		public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.792480 * Math.PI * 2;
		public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.769287 * Math.PI * 2;
		public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.252441 * Math.PI * 2;

		public static final double kMaxSpeedMetersPerSecond = 3.5;
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

	public static class intake{
		public static final int intake_front = 15;
		public static final int intake_back = 16;
		public static final double speed_front = -.25;
		public static final double speed_back = -.25;
		public static final int stator_current_limit = 80;
		public static final int supply_current_limit = 40;
		public static final boolean stator_limit_enable = true;
		public static final boolean supply_limit_enable = true;
		public static final double note_current_threshold = 27;
		public static final double note_time_threshold = .25;
	}

	public static class ArmConstants {
		public static final int kArmRightMotorPort = 17;
		public static final int kArmLeftMotorPort = 18;

		public static final boolean kInverseArmRightMotor = true;
		public static final boolean kInverseArmLeftMotor = true;

		public static final int kMotorSmartCurrentLimit = 40;

		public static final double kEncoderConversionFactor = 360;
		public static final double kEncoderZeroOffest = 244.4518089;

		public static final double kPArm = 0.014;
		public static final double kIArm = 0.000000001;
		public static final double kDArm = 0.001;

		public static final double kFFArm = 0.000015;

		public static final NumberRange kPercentOutputRange = new NumberRange(-0.35, 0.35);

		public static final boolean kIsPIDWrappingEnabled = true;
		public static final NumberRange kPIDWrappingRange = new NumberRange(0.0, 360.0);

		public static final double kArmUpPosition = 160.0;
		public static final double kArmDownPosition = 6.5;
	}

	public record NumberRange(double min, double max) {};
}