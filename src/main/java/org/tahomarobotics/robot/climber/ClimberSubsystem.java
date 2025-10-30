package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tinylog.Logger;

import static edu.wpi.first.units.BaseUnits.AngleUnit;

class ClimberSubsystem extends AbstractSubsystem {
    final TalonFX rollerMotor;
    final TalonFX leftPivotMotor;
    final TalonFX rightPivotMotor;
    final Solenoid solenoid;

    final MotionMagicVoltage pivotPositionController = new MotionMagicVoltage(0);
    final MotionMagicVelocityVoltage rollerVelocityController = new MotionMagicVelocityVoltage(0);

    final LoggedStatusSignal[] signals;

    PivotState pivotState = PivotState.STOWED;
    HoldState holdState = HoldState.EMPTY;




    ClimberSubsystem() {
        Logger.info("Instantiating Climber subsystem...");
        rollerMotor = new TalonFX(RobotMap.CLIMBER_ROLLER_MOTOR);
        leftPivotMotor = new TalonFX(RobotMap.CLIMBER_LEFT_MOTOR);
        rightPivotMotor = new TalonFX(RobotMap.CLIMBER_RIGHT_MOTOR);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_SOLENOID);

        signals = new LoggedStatusSignal[] {
                new LoggedStatusSignal("Pivot Position", leftPivotMotor.getPosition()),
                new LoggedStatusSignal("Roller Velocity", rollerMotor.getVelocity()),
                new LoggedStatusSignal("Pivot Voltage", leftPivotMotor.getMotorVoltage())
        };

        RobustConfigurator.tryConfigureTalonFX("Climber Roller Motor", rollerMotor, ClimberConstants.climberMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Climber Left Pivot Motor", leftPivotMotor, ClimberConstants.leftMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Climber Right Pivot Motor", rightPivotMotor, ClimberConstants.rightMotorConfiguration);
    }

    void setRollerVelocity(AngularVelocity targetVelocity) {
        rollerMotor.setControl(rollerVelocityController.withVelocity(targetVelocity));
        org.littletonrobotics.junction.Logger.recordOutput("Climber/Roller Target Velocity", targetVelocity);
    }

    void setPivotPosition(Angle targetPosition) {
        leftPivotMotor.setControl(pivotPositionController.withPosition(targetPosition));
        rightPivotMotor.setControl(pivotPositionController);
        org.littletonrobotics.junction.Logger.recordOutput("Climber/Pivot Target Position", targetPosition);

    }

    void setPivotVoltage(double targetVoltage) {
        leftPivotMotor.setVoltage(targetVoltage);
        rightPivotMotor.setVoltage(targetVoltage);
        org.littletonrobotics.junction.Logger.recordOutput("Climber/Pivot Target Voltage", targetVoltage);
    }

    void stopRoller() {
        Logger.info("Stopping climber roller.");
        rollerMotor.stopMotor();
        org.littletonrobotics.junction.Logger.recordOutput("Climber/Roller Target Velocity", 0);
    }

    void stopPivot() {
        Logger.info("Stopping climber pivot.");
        leftPivotMotor.stopMotor();
        rightPivotMotor.stopMotor();
        org.littletonrobotics.junction.Logger.recordOutput("Climber/Pivot Target Voltage", 0);
    }

    //i guess...
    void engageSolenoid() {
        solenoid.set(true);
    }

    void disengageSolenoid() {
        solenoid.set(false);
    }

    void transitionToStowed() {
        disengageSolenoid();
        pivotState = PivotState.STOWED;
        setPivotPosition(PivotState.STOWED.theta);
        engageSolenoid();
    }

    void transitionToDeployed() {
        disengageSolenoid();
        pivotState = PivotState.DEPLOYED;
        setPivotPosition(PivotState.DEPLOYED.theta);
        engageSolenoid();
    }

    void transitionToHoldingCage() {
        holdState = HoldState.HOLDING_CAGE;
        stopRoller();
    }

    void transitionToIntakingCage() {
        holdState = HoldState.INTAKING_CAGE;
        setRollerVelocity(ClimberConstants.MAX_ROLLER_VELOCITY);
    }

    @Override
    public void subsystemPeriodic() {
        LoggedStatusSignal.refreshAll(signals);
        LoggedStatusSignal.log("Climber/", signals);
    }

    enum PivotState {
        STOWED(Angle.ofBaseUnits(ClimberConstants.STOW_POSITION, Units.Degrees)),
        DEPLOYED(Angle.ofBaseUnits(ClimberConstants.STOW_POSITION, Units.Degrees)),
        ZEROED(Angle.ofBaseUnits(ClimberConstants.STOW_POSITION, Units.Degrees));

        final Angle theta;

        private PivotState(Angle theta) {this.theta = theta;}
    }

    enum HoldState {
        EMPTY,
        HOLDING_CAGE,
        INTAKING_CAGE,
        CLIMBED;
    }
}