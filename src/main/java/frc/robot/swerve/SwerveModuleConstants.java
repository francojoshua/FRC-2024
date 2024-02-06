package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public record SwerveModuleConstants(int driveMotorId, int angleMotorId, int canCoderId,
		Rotation2d angleOffset, boolean invertDriveEncoder, boolean invertAngleEncoder,
		boolean invertCanCoder) {
}