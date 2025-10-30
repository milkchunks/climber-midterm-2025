package org.tahomarobotics.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tinylog.Logger;

import java.util.function.BooleanSupplier;

public class ClimberCommands {
    public static Command createZeroCommand(ClimberSubsystem climberSubsystem) {
        //todo: somewhere in here the commandscheduler loop gets overrun
        return climberSubsystem
                .runOnce(climberSubsystem::disengageSolenoid)
                .andThen(climberSubsystem.runOnce(climberSubsystem::setZeroingVoltage))
                .andThen(Commands.waitUntil(climberSubsystem::pivotIsStopped).withTimeout(ClimberConstants.ZEROING_TIMEOUT))
                .andThen(climberSubsystem::zero);
    }

    public static Command createClimbCommand(ClimberSubsystem climberSubsystem) {
        if (climberSubsystem.holdState == ClimberSubsystem.HoldState.CLIMBED) {
            return climberSubsystem.runOnce(() -> Logger.error("You can't climb twice :("));
        }
        return climberSubsystem.runOnce(climberSubsystem::transitionToDeployed)
                .andThen(climberSubsystem.runOnce(climberSubsystem::beginIntakingCage))
                .andThen(Commands.waitUntil(climberSubsystem::isLimitSwitchHit).withTimeout(ClimberConstants.CLIMB_TIMEOUT))
                .andThen(climberSubsystem.runOnce(climberSubsystem::transitionToHoldingCage))
                .andThen(climberSubsystem::transitionToStowed)
                .andThen(climberSubsystem::transitionToClimbed);
    }
}
