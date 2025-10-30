package org.tahomarobotics.robot.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tinylog.Logger;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.BaseUnits.AngleUnit;

class ClimberSubsystem extends AbstractSubsystem implements AutoCloseable {
    final TalonFX rollerMotor;
    final TalonFX leftPivotMotor;
    final TalonFX rightPivotMotor;
    final VictorSPX solenoid;
    final DigitalInput limitSwitch;

    final MotionMagicVoltage pivotPositionController = new MotionMagicVoltage(0);
    final MotionMagicVelocityVoltage rollerVelocityController = new MotionMagicVelocityVoltage(0);

    final LoggedStatusSignal[] signals;

    //Tracker variables for logging
    @AutoLogOutput(key = "Climber/Pivot State")
    PivotState pivotState = PivotState.STOWED;
    @AutoLogOutput(key = "Climber/Hold State")
    HoldState holdState = HoldState.EMPTY;
    @AutoLogOutput(key = "Climber/Roller Target Velocity")
    AngularVelocity rollerTargetVelocity = AngularVelocity.ofBaseUnits(0.0, Units.DegreesPerSecond);
    @AutoLogOutput(key = "Climber/Pivot Target Position")
    Angle pivotTargetPosition = pivotState.theta;
    @AutoLogOutput(key = "Climber/Is Solenoid Engaged?")
    boolean isSolenoidEngaged = true;
    @AutoLogOutput(key = "Climber/Using Zero Pivot Voltage?")
    boolean usingZeroPivotVoltage = false;




    ClimberSubsystem() {
        rollerMotor = new TalonFX(RobotMap.CLIMBER_ROLLER_MOTOR);
        leftPivotMotor = new TalonFX(RobotMap.CLIMBER_LEFT_MOTOR);
        rightPivotMotor = new TalonFX(RobotMap.CLIMBER_RIGHT_MOTOR);
        solenoid = new VictorSPX(RobotMap.CLIMBER_SOLENOID);
        limitSwitch = new DigitalInput(RobotMap.CLIMBER_LIMIT_SWITCH);

        signals = new LoggedStatusSignal[] {
                new LoggedStatusSignal("Pivot Actual Position", leftPivotMotor.getPosition()),
                new LoggedStatusSignal("Roller Actual Velocity", rollerMotor.getVelocity()),
                new LoggedStatusSignal("Pivot Voltage", leftPivotMotor.getMotorVoltage())
        };

        RobustConfigurator.tryConfigureTalonFX("Climber Roller Motor", rollerMotor, ClimberConstants.climberMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Climber Left Pivot Motor", leftPivotMotor, ClimberConstants.leftMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Climber Right Pivot Motor", rightPivotMotor, ClimberConstants.rightMotorConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(50, signals[0].signal(), signals[1].signal()); //more important, 50 updates per second
        BaseStatusSignal.setUpdateFrequencyForAll(10, signals[2].signal()); //less important, 10 updates per second
        ParentDevice.optimizeBusUtilizationForAll(rollerMotor, leftPivotMotor, rightPivotMotor);
    }

    void setRollerVelocity(AngularVelocity targetVelocity) {
        rollerTargetVelocity = targetVelocity;
        rollerMotor.setControl(rollerVelocityController.withVelocity(targetVelocity));
    }

    void setPivotPosition(Angle targetPosition) {
        pivotTargetPosition = targetPosition;
        leftPivotMotor.setControl(pivotPositionController.withPosition(targetPosition));
        rightPivotMotor.setControl(pivotPositionController);

    }

    void setZeroingVoltage() {
        usingZeroPivotVoltage = true;
        pivotState = PivotState.ZEROED;
        leftPivotMotor.setVoltage(ClimberConstants.ZERO_VOLTAGE.baseUnitMagnitude());
        rightPivotMotor.setVoltage(ClimberConstants.ZERO_VOLTAGE.baseUnitMagnitude());
        usingZeroPivotVoltage = false;
    }

    void stopRoller() {
        rollerTargetVelocity = AngularVelocity.ofBaseUnits(0.0, Units.DegreesPerSecond);
        rollerMotor.stopMotor();
    }

    boolean pivotIsStopped() {
        return Math.abs(leftPivotMotor.getVelocity().getValueAsDouble()) < ClimberConstants.STOPPED_TOLERANCE;
    }

    //Set zero position of motors, then stow.
    void zero() {
        Logger.info("Climber zeroing.");

        //set the motors' internal measure of position to the zeroed angle.
        leftPivotMotor.setPosition(PivotState.ZEROED.theta);
        rightPivotMotor.setPosition(PivotState.ZEROED.theta);
        pivotState = PivotState.ZEROED;

        //Stow
        transitionToStowed();
    }

    void engageSolenoid() {
        isSolenoidEngaged = true;
        //Continuously press on ratchet with 1% speed to hold it in place.
        solenoid.set(VictorSPXControlMode.PercentOutput, ClimberConstants.SOLENOID_ENGAGED_PERCENT_SPEED / 100.0);
    }

    void disengageSolenoid() {
        isSolenoidEngaged = false;
        solenoid.set(VictorSPXControlMode.PercentOutput, 0);
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

    void transitionToClimbed() {
        Logger.info("Tried climbing!");
        holdState = HoldState.CLIMBED;
    }

    void beginIntakingCage() {
        holdState = HoldState.INTAKING_CAGE;
        setRollerVelocity(ClimberConstants.MAX_ROLLER_VELOCITY);
    }

    @AutoLogOutput(key = "Climber/Is Limit Switch Hit")
    boolean isLimitSwitchHit() {
        return !limitSwitch.get();
    }

    @Override
    public void subsystemPeriodic() {
        LoggedStatusSignal.refreshAll(signals);
        LoggedStatusSignal.log("Climber/", signals);
    }

    @Override
    public void close() throws Exception {
        rollerMotor.close();
        leftPivotMotor.close();
        rightPivotMotor.close();
        limitSwitch.close();
    }

    enum PivotState {
        STOWED(Angle.ofBaseUnits(ClimberConstants.STOW_POSITION, Units.Degrees)),
        DEPLOYED(Angle.ofBaseUnits(ClimberConstants.DEPLOY_POSITION, Units.Degrees)),
        ZEROED(Angle.ofBaseUnits(ClimberConstants.ZERO_POSITION, Units.Degrees));

        final Angle theta;

        PivotState(Angle theta) {this.theta = theta;}
    }

    enum HoldState {
        EMPTY,
        HOLDING_CAGE,
        INTAKING_CAGE,
        CLIMBED;
    }
}