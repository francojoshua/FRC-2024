package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.swerve.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

	private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
			DriveConstants.kFrontLeftAngleMotorPort,
			DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
			DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
			DriveConstants.kInverseFrontLeftDriveEncoder,
			DriveConstants.kInverseFrontLeftAngleEncoder,
			DriveConstants.kInverseFrontLeftDriveAbsoluteEncoder);

	private final SwerveModule frontRight = new SwerveModule(
			DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightAngleMotorPort,
			DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
			DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
			DriveConstants.kInverseFrontRightDriveEncoder,
			DriveConstants.kInverseFrontRightAngleEncoder,
			DriveConstants.kInverseFrontRightDriveAbsoluteEncoder);

	private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
			DriveConstants.kBackLeftAngleMotorPort,
			DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
			DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
			DriveConstants.kInverseBackLeftDriveEncoder,
			DriveConstants.kInverseBackLeftAngleEncoder,
			DriveConstants.kInverseBackLeftDriveAbsoluteEncoder);

	private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
			DriveConstants.kBackRightAngleMotorPort,
			DriveConstants.kBackRightDriveAbsoluteEncoderPort,
			DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
			DriveConstants.kInverseBackRightDriveEncoder,
			DriveConstants.kInverseBackRightAngleEncoder,
			DriveConstants.kInverseBackRightDriveAbsoluteEncoder);

	private AHRS gyro = new AHRS(SPI.Port.kMXP);
	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
			DriveConstants.kDriveKinematics, new Rotation2d(0),
			new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
					new SwerveModulePosition(), new SwerveModulePosition(),},
			new Pose2d(0, 0, new Rotation2d()));

	public void zeroHeading() {
		gyro.reset(); // sets yaw to 0
	}

	public double getHeading() {
		return Math.IEEEremainder(gyro.getAngle(), 360);
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
				DriveConstants.kMaxSpeedMetersPerSecond);
		frontRight.setDesiredState(desiredStates[0]);
		frontLeft.setDesiredState(desiredStates[1]);
		backRight.setDesiredState(desiredStates[2]);
		backLeft.setDesiredState(desiredStates[3]);

		Logger.recordOutput("MyStates", desiredStates);
	}

	public void stopModules() {
		frontRight.stop();
		frontLeft.stop();
		backRight.stop();
		backLeft.stop();
	}

	public void resetEncoders() {
		frontRight.resetEncoders();
		frontLeft.resetEncoders();
		backRight.resetEncoders();
		backLeft.resetEncoders();
	}

	@Override
	public void periodic() {
		odometer.update(getRotation2d(), new SwerveModulePosition[] {frontLeft.getPosition(),
				frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});

		SmartDashboard.putNumber("Robot Heading", getHeading());
		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
	}
}
