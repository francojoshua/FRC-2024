package frc.robot.swerve;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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

	// private final PIDController anglePIDController;

	private final SparkPIDController anglePIDController;
	private final SparkPIDController drivePIDController;

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

		driveMotor.setIdleMode(IdleMode.kBrake);
		angleMotor.setIdleMode(IdleMode.kCoast);

		driveEncoder = driveMotor.getEncoder();
		angleEncoder = angleMotor.getEncoder();

		anglePIDController = angleMotor.getPIDController();
		anglePIDController.setFeedbackDevice(angleEncoder);

		drivePIDController = driveMotor.getPIDController();
		drivePIDController.setFeedbackDevice(driveEncoder);


		initializeEncoders();

		absoluteEncoder = new CANcoder(absoluteEncoderId);

		this.inverseAbsoluteEncoder = inverseAbsoluteEncoder;
		this.absoluteEncoderOffset = absoluteEncoderOffset;
		
		anglePIDController.setP(SwerveConstants.kPAngle);
		anglePIDController.setI(0.0);
		anglePIDController.setD(0.0);

		anglePIDController.setPositionPIDWrappingEnabled(true);
		anglePIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
		anglePIDController.setPositionPIDWrappingMinInput(0.0);

		anglePIDController.setOutputRange(-1, 1);
		anglePIDController.setFF(0.0);

		drivePIDController.setP(0.3);
		drivePIDController.setI(0.0);
		drivePIDController.setD(0.0);

		drivePIDController.setOutputRange(-1, 1);
		drivePIDController.setFF(1 / 5676);

		driveMotor.setSmartCurrentLimit(50);
		angleMotor.setSmartCurrentLimit(20);

		driveMotor.burnFlash();
		angleMotor.burnFlash();


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

	public void setDesiredState(SwerveModuleState state, boolean isSlowModeEnabled) {
		if (Math.abs(state.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}

		// Optimizes the angle of the motor. If it can turn -45 instead of 135 degrees.
		// It will.
		state = SwerveModuleState.optimize(state, new Rotation2d(getAnglePosition()));

		//driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
		//angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));

		drivePIDController.setReference(state.speedMetersPerSecond * (isSlowModeEnabled ? 0.2 : 1), ControlType.kVelocity);
		anglePIDController.setReference(state.angle.getRadians(), ControlType.kPosition);

		SmartDashboard.putString("Swerve " + absoluteEncoder.getDeviceID() + " state",
				state.toString());
		
		SmartDashboard.putNumber("Absolute Position " + absoluteEncoder.getDeviceID(), Units.radiansToDegrees(getAbsolutePosition()));
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition();
	}

	public double getDriveVelocity() {
		return driveEncoder.getVelocity();
	}

	public double getAnglePosition() {
		return getAbsolutePosition();
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
