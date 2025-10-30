package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

public class ClimberSubsystem {
    TalonFX rollerMotor;

    ClimberSubsystem() {
        Logger.info("Instantiating Climber subsystem...");
        rollerMotor = new TalonFX(RobotMap.COLLECTOR_COLLECT_MOTOR);
        RobustConfigurator.tryConfigureTalonFX("Climber Roller Motor", rollerMotor, ClimberConstants.climberMotorConfiguration);
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