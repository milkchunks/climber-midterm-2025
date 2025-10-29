package org.tahomarobotics.robot.climber;

import org.tinylog.Logger;

public class Climber implements AutoCloseable {
    private final ClimberSubsystem climber;
    public Climber() {
        this(new ClimberSubsystem());
        Logger.info("Creating new instance of Climber...");
    }

    Climber(ClimberSubsystem climber) {
        this.climber = climber;
    }
    
    @Override
    public void close() throws Exception {}
}
