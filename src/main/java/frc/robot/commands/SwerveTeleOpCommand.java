
package frc.robot.commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveTeleOpCommand extends Command {

	private final SwerveSubsystem swerveSubsystem;
	private final Supplier<Double> leftAxisX, leftAxisY, rightAxisX;

	private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

	public SwerveTeleOpCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> leftAxisX,
			Supplier<Double> leftAxisY, Supplier<Double> rightAxisX) {
		this.swerveSubsystem = swerveSubsystem;
		this.leftAxisX = leftAxisX;
		this.leftAxisY = leftAxisY;
		this.rightAxisX = rightAxisX;

		this.xLimiter = new SlewRateLimiter(3);
		this.yLimiter = new SlewRateLimiter(3);
		this.rotLimiter = new SlewRateLimiter(3);

		addRequirements(swerveSubsystem);
	}

	@Override
	public void initialize() {
		swerveSubsystem.zeroHeading();
	}

	@Override
	public void execute() {

		// X FORWARD, Y SIDEAWAYS
		double vxSpeed = -leftAxisX.get();
		double vySpeed = -leftAxisY.get();
		double rot = -rightAxisX.get();

		// Deadband
		vxSpeed = Math.abs(vxSpeed) > ControllerConstants.kDeadband ? vxSpeed : 0.0;
		vySpeed = Math.abs(vySpeed) > ControllerConstants.kDeadband ? vySpeed : 0.0;
		rot = Math.abs(rot) > ControllerConstants.kDeadband ? rot : 0.0;

		// use slew limiter here
		vxSpeed = xLimiter.calculate(vxSpeed) * (DriveConstants.kMaxSpeedMetersPerSecond / 4);
		vySpeed = yLimiter.calculate(vySpeed) * (DriveConstants.kMaxSpeedMetersPerSecond / 4);
		rot = rotLimiter.calculate(rot)
				* (DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4);

		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxSpeed, vySpeed, rot,
				swerveSubsystem.getRotation2d());

		SwerveModuleState[] moduleStates =
				DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		swerveSubsystem.setModuleStates(moduleStates);
	}

	@Override
	public void end(boolean interrupted) {
		swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}
