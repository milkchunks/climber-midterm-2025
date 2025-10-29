package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import org.tahomarobotics.robot.RobotMap;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

class ClimberConstants {
    private static final double GEAR_RATIO = (8.0 / 34.0) * (18.0 / 18.0);
    private static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(5.0);
    //wow this is the worst thing ever
    //reach max velocity in 1/4 of a second at the fastest (tentative)
    private static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(MAX_VELOCITY.times(RotationsPerSecond.of(4.0)).magnitude());
    static final TalonFXConfiguration climberMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    //positive = intake cage, negative = eject cage
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(GEAR_RATIO))
            .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                    .withForwardLimitRemoteSensorID(RobotMap.CLIMBER_LIMIT_SWITCH))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                    .withMotionMagicAcceleration(MAX_ACCELERATION));
}
