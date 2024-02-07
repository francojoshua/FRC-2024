package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public record SwerveModuleConstants(int driveMotorId, int angleMotorId, int canCoderId,
		double angleOffset, boolean invertDriveMotor, boolean invertAngleMotor,
		boolean invertCanCoder) {
}