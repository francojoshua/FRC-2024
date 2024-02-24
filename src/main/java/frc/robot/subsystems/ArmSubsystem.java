package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.intake;


public class ArmSubsystem extends SubsystemBase {
	private final CANSparkMax armleft;
	private final CANSparkMax armright;
	private final CurrentLimitsConfigs current_limits;
	private final MotorOutputConfigs motor_output_configs;
	private final TalonFXConfiguration configs;
	
	

	public ArmSubsystem(){
		armleft = new CANSparkMax(17, MotorType.kBrushed);
		armright = new CANSparkMax(18, MotorType.kBrushed);
		current_limits = new CurrentLimitsConfigs();
		motor_output_configs = new MotorOutputConfigs();
		configs = new TalonFXConfiguration();

		stopmotors();
		//motor_configs();
	}

	public void stopmotors(){
		armleft.stopMotor();
		armright.stopMotor();
	}



	public void setarm(boolean reverse){
		armleft.set(-0.35 * (reverse ? -1 : 1));
		armright.set(0.35 * (reverse ? -1 : 1));
	}

	public void holdArm() {
		armleft.set(0.1);
		armright.set(-0.1);
	}

	public void motor_configs(){
		current_limits.withStatorCurrentLimit(intake.stator_current_limit);
		current_limits.withStatorCurrentLimitEnable(intake.stator_limit_enable);
		current_limits.withSupplyCurrentLimit(intake.supply_current_limit); 
		current_limits.withSupplyCurrentLimitEnable(intake.supply_limit_enable);

		motor_output_configs.withNeutralMode(NeutralModeValue.valueOf(null,"brake"));

		configs.withCurrentLimits(current_limits);
		configs.withMotorOutput(motor_output_configs);
		
		
	}


}
