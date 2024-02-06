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
import edu.wpi.first.wpilibj.DriverStation;
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
	private final SwerveDriveOdometry odometry;

	public SwerveSubsystem() {

		// ODOMETRY
		odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(),
				getModulePositions());

		// PATHPLANNER
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
						4.5, // Max module speed, in m/s
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
	}

	public void zeroHeading() {
		gyro.reset(); // sets yaw to 0
	}

	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	public Rotation2d getRotation2d() {
		return gyro.getRotation2d();
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetPose(Pose2d pose) {
		odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
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

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
				backLeft.getPosition(), backRight.getPosition()};
	}

	@Override
	public void periodic() {
		odometry.update(getRotation2d(), getModulePositions());

		SmartDashboard.putString("Robot Heading", getHeading().toString());
		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
	}
}
