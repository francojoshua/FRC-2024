package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Mod0Constants;
import frc.robot.Constants.DriveConstants.Mod1Constants;
import frc.robot.Constants.DriveConstants.Mod2Constants;
import frc.robot.Constants.DriveConstants.Mod3Constants;
import frc.robot.swerve.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

	private final SwerveModule frontRight = new SwerveModule(0, Mod0Constants.constants);

	private final SwerveModule frontLeft = new SwerveModule(1, Mod1Constants.constants);

	private final SwerveModule backRight = new SwerveModule(2, Mod2Constants.constants);

	private final SwerveModule backLeft = new SwerveModule(3, Mod3Constants.constants);

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
