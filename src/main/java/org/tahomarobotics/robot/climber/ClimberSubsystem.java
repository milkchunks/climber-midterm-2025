package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

public class ClimberSubsystem {
    TalonFX rollerMotor;
    TalonFX leftPivotMotor;
    TalonFX rightPivotMotor;
    PWMVictorSPX solenoid;

    ClimberSubsystem() {
        Logger.info("Instantiating Climber subsystem...");
        rollerMotor = new TalonFX(RobotMap.CLIMBER_ROLLER_MOTOR);
        leftPivotMotor = new TalonFX(RobotMap.CLIMBER_LEFT_MOTOR);
        rightPivotMotor = new TalonFX(RobotMap.CLIMBER_RIGHT_MOTOR);
        solenoid = new PWMVictorSPX(RobotMap.CLIMBER_SOLENOID);
        RobustConfigurator.tryConfigureTalonFX("Climber Roller Motor", rollerMotor, ClimberConstants.climberMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Climber Left Pivot Motor", leftPivotMotor, ClimberConstants.leftMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Climber Right Pivot Motor", rightPivotMotor, ClimberConstants.rightMotorConfiguration);
    }

    enum PivotState {
        STOWED(ClimberConstants.STOW_POSITION),
        DEPLOYED(ClimberConstants.DEPLOY_POSITION),
        ZEROED(ClimberConstants.STOW_POSITION);

        final double position;

        private PivotState(double position) {
            this.position = position;
        }
    }

    enum HoldState {
        EMPTY,
        HOLDING_CAGE,
        INTAKING_CAGE,
        EJECTING_CAGE;
    }
}