package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

public class ClimberSubsystem {
    TalonFX climberMotor;
    ClimberSubsystem() {
        Logger.info("Instantiating Climber subsystem...");
        climberMotor = new TalonFX(RobotMap.CLIMBER_MOTOR);
        RobustConfigurator.tryConfigureTalonFX("Climber Motor", climberMotor, ClimberConstants.climberMotorConfiguration);
    }
}
