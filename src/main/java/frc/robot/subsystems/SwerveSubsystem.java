package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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

	private double m_headingTargetAngle;

	public SwerveSubsystem() {
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a
									// starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
											// ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
													// live in your Constants class
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
						DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						DriveConstants.kTrackWidth, // Drive base radius in meters. Distance from
													// robot center to furthest
						// module.
						new ReplanningConfig() // Default path replanning config. See the API for
												// the options here
				), () -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				}, this // Reference to this subsystem to set requirements
		);

		m_headingTargetAngle = getHeading();
	}

	private boolean isSlowModeEnabled = false;

	public void zeroHeading() {
		gyro.reset(); // sets yaw to 0
	}

	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	public Rotation2d getRotation2d() {
		return gyro.getRotation2d();
	}

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public Rotation2d getOdometer2d() {
		return odometer.getPoseMeters().getRotation().rotateBy(Rotation2d.fromDegrees(180));
	}

	public void resetPose(Pose2d pose) {
		odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
	}


	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
				DriveConstants.kMaxSpeedMetersPerSecond);
		frontRight.setDesiredState(desiredStates[0], isSlowModeEnabled);
		frontLeft.setDesiredState(desiredStates[1], isSlowModeEnabled);
		backRight.setDesiredState(desiredStates[2], isSlowModeEnabled);
		backLeft.setDesiredState(desiredStates[3], isSlowModeEnabled);

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

	public void toggleSlowMode() {
		this.isSlowModeEnabled = !isSlowModeEnabled;
	}

	public void disableSlowMode() {
		this.isSlowModeEnabled = false;
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		SwerveModuleState[] states = new SwerveModuleState[] {frontRight.getState(),
				frontLeft.getState(), backRight.getState(), backLeft.getState()};


		return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
	}

	public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] moduleStates =
				DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		setModuleStates(moduleStates);
	}


	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {frontRight.getPosition(), frontLeft.getPosition(),
				backRight.getPosition(), backLeft.getPosition()};
	}


	@Override
	public void periodic() {
		odometer.update(getRotation2d(), new SwerveModulePosition[] {frontRight.getPosition(),
				frontLeft.getPosition(), backRight.getPosition(), backLeft.getPosition()});

		SmartDashboard.putNumber("Robot Heading", getHeading());
		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
	
	}
}
