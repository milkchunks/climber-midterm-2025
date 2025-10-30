package org.tahomarobotics.robot.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.tinylog.Logger;

public class Climber implements AutoCloseable {
    public final ClimberSubsystem climber;
    public Command zeroCommand;
    public Command climbCommand;

    public Climber() {
        this(new ClimberSubsystem());
        Logger.info("Creating new instance of Climber...");
        zeroCommand = ClimberCommands.createZeroCommand(climber);
        climbCommand = ClimberCommands.createClimbCommand(climber);
    }

    Climber(ClimberSubsystem climber) {
        this.climber = climber;
    }
    
    @Override
    public void close() throws Exception {}
}
