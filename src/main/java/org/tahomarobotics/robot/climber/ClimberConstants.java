package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import org.tahomarobotics.robot.RobotMap;

import static edu.wpi.first.units.Units.*;

class ClimberConstants {
    private static final double ROLLER_GEAR_RATIO = (34.0 / 8.0) * (18.0 / 18.0);
    private static final double PIVOT_GEAR_RATIO = (72.0 / 8.0) * (72.0 / 20.0) * (9.0 / 72.0) * (36.0 / 9.0);

    private static final AngularVelocity MAX_ROLLER_VELOCITY = RotationsPerSecond.of(5.0);
    //wow this is the worst thing ever
    //reach max velocity in 1/4 of a second at the fastest (tentative)
    private static final AngularAcceleration MAX_ROLLER_ACCELERATION = RotationsPerSecondPerSecond.of(MAX_ROLLER_VELOCITY.times(RotationsPerSecond.of(4.0)).magnitude());

    private static final AngularVelocity MAX_PIVOT_VELOCITY = RotationsPerSecond.of(2.0);
    private static final AngularAcceleration MAX_PIVOT_ACCELERATION = RotationsPerSecondPerSecond.of(MAX_PIVOT_VELOCITY.times(RotationsPerSecond.of(4.0)).magnitude());

    static final double STOW_POSITION = 0.0;
    static final double DEPLOY_POSITION = 90.0;

    static final double ZERO_VOLTAGE = 1.0;

    static final double SOLENOID_ENGAGEMENT_PERCENT_SPEED = 1.0;

    static final TalonFXConfiguration climberMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    //positive = intake cage, negative = eject cage
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ROLLER_GEAR_RATIO))
            .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                    .withForwardLimitRemoteSensorID(RobotMap.CLIMBER_LIMIT_SWITCH))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(MAX_ROLLER_VELOCITY)
                    .withMotionMagicAcceleration(MAX_ROLLER_ACCELERATION));

    //positive direction is spinning climber in towards the robot
    static final TalonFXConfiguration leftMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(MAX_PIVOT_VELOCITY)
                    .withMotionMagicAcceleration(MAX_PIVOT_ACCELERATION));

    static final TalonFXConfiguration rightMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(MAX_PIVOT_VELOCITY)
                    .withMotionMagicAcceleration(MAX_PIVOT_ACCELERATION));

}
