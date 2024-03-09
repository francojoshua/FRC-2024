package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;


public class ArmSubsystem extends SubsystemBase {
	private final CANSparkMax armLeft;
	private final CANSparkMax armRight;

	private final SparkPIDController pidController;
	private final SparkAbsoluteEncoder absoluteEncoder;

	private double currentPosition;

	public ArmSubsystem() {
		armRight = new CANSparkMax(ArmConstants.kArmRightMotorPort, MotorType.kBrushed); // Main Motor
		armLeft = new CANSparkMax(ArmConstants.kArmLeftMotorPort, MotorType.kBrushed); // Follower Motor

		armRight.restoreFactoryDefaults();
		armLeft.restoreFactoryDefaults();

		armRight.setInverted(ArmConstants.kInverseArmRightMotor);
		armLeft.follow(armRight, ArmConstants.kInverseArmLeftMotor);

		armLeft.setIdleMode(IdleMode.kBrake);
		armRight.setIdleMode(IdleMode.kBrake);

		this.pidController = armRight.getPIDController();
		this.absoluteEncoder = armRight.getAbsoluteEncoder(Type.kDutyCycle);
		
		absoluteEncoder.setPositionConversionFactor(ArmConstants.kEncoderConversionFactor);
		absoluteEncoder.setZeroOffset(ArmConstants.kEncoderZeroOffest);

		currentPosition = getPosition();

		armRight.setSmartCurrentLimit(ArmConstants.kMotorSmartCurrentLimit);
		armLeft.setSmartCurrentLimit(ArmConstants.kMotorSmartCurrentLimit);

		pidController.setFeedbackDevice(absoluteEncoder);
		pidController.setP(ArmConstants.kPArm);
		pidController.setI(ArmConstants.kIArm);
		pidController.setD(ArmConstants.kDArm);

		SmartDashboard.putNumber("ArmP", ArmConstants.kPArm);
		SmartDashboard.putNumber("ArmI", ArmConstants.kIArm);
		SmartDashboard.putNumber("ArmD", ArmConstants.kDArm);
		SmartDashboard.putNumber("ArmFF", ArmConstants.kFFArm);

		pidController.setFF(ArmConstants.kFFArm);

		pidController.setOutputRange(ArmConstants.kPercentOutputRange.min(), ArmConstants.kPercentOutputRange.max());

		pidController.setPositionPIDWrappingEnabled(ArmConstants.kIsPIDWrappingEnabled);
		pidController.setPositionPIDWrappingMinInput(ArmConstants.kPIDWrappingRange.min());
		pidController.setPositionPIDWrappingMaxInput(ArmConstants.kPIDWrappingRange.max());

		armLeft.burnFlash();
		armRight.burnFlash();
		
		stopMotors();
	}

	public void resetPosition() {
		this.currentPosition = getPosition();
	}


	public void setPosition(double position) {
		this.currentPosition = position;
	}

	public double getPosition() {
		return absoluteEncoder.getPosition();
	}

	public void stopMotors() {
		armLeft.stopMotor();
		armRight.stopMotor();
	}

	/*
	 *  ArmSubsystem::setArm and ArmSubsystem::holdArm are not currently used and may not work properly.
	 */
	@Deprecated
	public void setArm(boolean reverse) {
		armLeft.set(-0.35 * (reverse ? -1 : 1));
		armRight.set(0.35 * (reverse ? -1 : 1));
	}

	@Deprecated
	public void holdArm() {
		armLeft.set(0.1);
		armRight.set(-0.1);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Arm Position", getPosition());
		SmartDashboard.putNumber("Arm Position Up", ArmConstants.kArmUpPosition);
		SmartDashboard.putNumber("Arm Position Down", ArmConstants.kArmDownPosition);
		pidController.setReference(currentPosition, ControlType.kPosition);


		pidController.setP(SmartDashboard.getNumber("ArmP", ArmConstants.kPArm));
		pidController.setI(SmartDashboard.getNumber("ArmI", ArmConstants.kIArm));
		pidController.setD(SmartDashboard.getNumber("ArmD", ArmConstants.kDArm));
		pidController.setFF(SmartDashboard.getNumber("ArmFF", ArmConstants.kFFArm));


		Logger.recordOutput("ArmPosition", getPosition());
	}
}
