package org.tahomarobotics.climber;

import org.littletonrobotics.junction.Logger;

public class Climber implements AutoCloseable {
    private final ClimberSubsystem climber;
    public Climber() {
        this(new ClimberSubsystem());
    }

    Climber(ClimberSubsystem climber) {
        this.climber = climber;
    }
    
    @Override
    public void close() throws Exception {}
}
