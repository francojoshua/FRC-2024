package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.robot.Constants.intake;


public class ArmSubsystem extends SubsystemBase {
	private final CANSparkMax armLeft;
	private final CANSparkMax armRight;

	private final SparkPIDController pidController;
	private final SparkAbsoluteEncoder absoluteEncoder;
	

	public ArmSubsystem() {
		armRight = new CANSparkMax(17, MotorType.kBrushed); // MAIN ONE.
		armLeft = new CANSparkMax(18, MotorType.kBrushed);

		armRight.restoreFactoryDefaults();
		armLeft.restoreFactoryDefaults();

		armRight.setInverted(true);

		armLeft.follow(armRight, true);

		this.pidController = armRight.getPIDController();
		this.absoluteEncoder = armRight.getAbsoluteEncoder(Type.kDutyCycle);

		absoluteEncoder.setPositionConversionFactor(360);
		absoluteEncoder.setZeroOffset(244.4518089);

		armRight.setSmartCurrentLimit(40);
		armLeft.setSmartCurrentLimit(40);

		pidController.setFeedbackDevice(absoluteEncoder);
		pidController.setP(0.012);
		pidController.setI(0.000000001);
		pidController.setD(0.0);

		pidController.setFF(0.000015);

		pidController.setOutputRange(-0.40, 0.40);

		pidController.setPositionPIDWrappingEnabled(true);
		pidController.setPositionPIDWrappingMinInput(0);
		pidController.setPositionPIDWrappingMaxInput(360);

	
		armLeft.burnFlash();
		armRight.burnFlash();
		
		stopmotors();
		//motor_configs();
	}

	public void stopmotors(){
		armLeft.stopMotor();
		armRight.stopMotor();
	}



	public void setarm(boolean reverse){
		armLeft.set(-0.35 * (reverse ? -1 : 1));
		armRight.set(0.35 * (reverse ? -1 : 1));
	}

	public void holdArm() {
		armLeft.set(0.1);
		armRight.set(-0.1);
	}

	public void setPosition(double position) {
		pidController.setReference(position, ControlType.kPosition);
	}

	public double getAbsoluteEncoderPosition() {
		return absoluteEncoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Arm Position", getAbsoluteEncoderPosition());
	}



}
