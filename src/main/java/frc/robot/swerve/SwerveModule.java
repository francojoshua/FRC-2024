package frc.robot.swerve;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

	private final CANSparkMax driveMotor;
	private final CANSparkMax angleMotor;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder angleEncoder;

	private final PIDController anglePIDController;
	private final CANcoder absoluteEncoder;

	private final boolean inverseAbsoluteEncoder;
	private final double absoluteEncoderOffset;

	public SwerveModule(int driveMotorId, int angleMotorId, int absoluteEncoderId,
			double absoluteEncoderOffset, boolean inverseDriveMotor, boolean inverseAngleMotor,
			boolean inverseAbsoluteEncoder) {
		driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
		angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);

		driveMotor.setInverted(inverseDriveMotor);
		angleMotor.setInverted(inverseAngleMotor);

		driveEncoder = driveMotor.getEncoder();
		angleEncoder = angleMotor.getEncoder();
		initializeEncoders();

		absoluteEncoder = new CANcoder(absoluteEncoderId);

		this.inverseAbsoluteEncoder = inverseAbsoluteEncoder;
		this.absoluteEncoderOffset = absoluteEncoderOffset;

		anglePIDController = new PIDController(SwerveConstants.kPAngle, 0.0, 0.0);
		anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

		resetEncoders();
	}

	/*
	 * Converts rotations to meters and radians for the internal encoders.
	 */
	private void initializeEncoders() {
		driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivePositionConversionFactor);
		driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor);

		angleEncoder.setPositionConversionFactor(SwerveConstants.kAnglePositionConversionFactor);
		angleEncoder.setVelocityConversionFactor(SwerveConstants.kAngleVelocityConversionFactor);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
	}

	public void setDesiredState(SwerveModuleState state) {
		if (Math.abs(state.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}

		// Optimizes the angle of the motor. If it can turn -45 instead of 135 degrees.
		// It will.
		state = SwerveModuleState.optimize(state, new Rotation2d(getAnglePosition()));

		driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
		angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));

		SmartDashboard.putString("Swerve " + absoluteEncoder.getDeviceID() + " state",
				state.toString());
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition();
	}

	public double getDriveVelocity() {
		return driveEncoder.getVelocity();
	}

	public double getAnglePosition() {
		return angleEncoder.getPosition();
	}

	public double getAngleVelocity() {
		return angleEncoder.getVelocity();
	}

	public double getAbsolutePosition() {
		StatusSignal<Double> angleSupplier = absoluteEncoder.getAbsolutePosition();

		double angle = Units.rotationsToRadians(angleSupplier.getValue());

		if (Robot.isReal()) {
			angle = angle - absoluteEncoderOffset;
		}

		angle = angle * (inverseAbsoluteEncoder ? -1.0 : 1.0);

		return angle;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAnglePosition()));
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0.0);
		angleEncoder.setPosition(getAbsolutePosition());
	}

	public void stop() {
		driveMotor.set(0.0);
		angleMotor.set(0.0);
	}
}
