package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

		armRight.setSmartCurrentLimit(ArmConstants.kMotorSmartCurrentLimit);
		armLeft.setSmartCurrentLimit(ArmConstants.kMotorSmartCurrentLimit);

		pidController.setFeedbackDevice(absoluteEncoder);
		pidController.setP(ArmConstants.kPArm);
		pidController.setI(ArmConstants.kIArm);
		pidController.setD(ArmConstants.kDArm);

		pidController.setFF(ArmConstants.kFFArm);

		pidController.setOutputRange(ArmConstants.kPercentOutputRange.min(), ArmConstants.kPercentOutputRange.max());

		pidController.setPositionPIDWrappingEnabled(ArmConstants.kIsPIDWrappingEnabled);
		pidController.setPositionPIDWrappingMinInput(ArmConstants.kPIDWrappingRange.min());
		pidController.setPositionPIDWrappingMaxInput(ArmConstants.kPIDWrappingRange.max());

		armLeft.burnFlash();
		armRight.burnFlash();
		
		stopMotors();
	}

	public void setPosition(double position) {
		pidController.setReference(position, ControlType.kPosition);
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
	}
}
