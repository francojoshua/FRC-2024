package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.intake;


public class IntakeSubsystem extends SubsystemBase {
	private final TalonFX front;
	private final TalonFX back;
	private final CurrentLimitsConfigs current_limits;
	private final MotorOutputConfigs motor_output_configs;
	private final TalonFXConfiguration configs;

	public IntakeSubsystem(){
		front = new TalonFX(intake.intake_front);
		back = new TalonFX(intake.intake_back);
		current_limits = new CurrentLimitsConfigs();
		motor_output_configs = new MotorOutputConfigs();
		configs = new TalonFXConfiguration();

		stopmotors();
		motor_configs();
	}

	public void stopmotors(){
		front.stopMotor();
		back.stopMotor();
	}

	public void setspeed(){
		front.set(intake.speed_front);
		back.set(intake.speed_back);
	}

	public void motor_configs(){
		current_limits.withStatorCurrentLimit(intake.stator_current_limit);
		current_limits.withStatorCurrentLimitEnable(intake.stator_limit_enable);
		current_limits.withSupplyCurrentLimit(intake.supply_current_limit); 
		current_limits.withSupplyCurrentLimitEnable(intake.supply_limit_enable);

		motor_output_configs.withNeutralMode(NeutralModeValue.valueOf(null,"brake"));

		configs.withCurrentLimits(current_limits);
		configs.withMotorOutput(motor_output_configs);
		
		front.getConfigurator().apply(configs);
		back.getConfigurator().apply(configs);
	}

	public double get_current(){
		var current = back.getTorqueCurrent();
		System.out.println(current.getValue());
		return current.getValue();
	}

	public boolean check_note(){
		boolean note = false;
		if(get_current() > intake.note_current_threshold){
			note = true;
		}
		return note;
	}

}
