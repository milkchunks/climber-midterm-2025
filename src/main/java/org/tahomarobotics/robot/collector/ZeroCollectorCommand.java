package org.tahomarobotics.robot.collector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroCollectorCommand extends Command {
    private final Collector collector = Collector.getInstance();

    private static final double TIMEOUT = 5;

    private static final double INITIAL_MOVE_TIME = 0.1;

    private final Timer timer = new Timer();

    public ZeroCollectorCommand() {
        addRequirements(this.collector);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        collector.setZeroingVoltage();
    }

    @Override
    public boolean isFinished() {
        return hasStopped() || timer.hasElapsed(TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        collector.zero();
        collector.setTargetDeploymentState(CollectorConstants.TargetDeploymentState.STOW);
    }

    private boolean hasStopped() {
        return timer.hasElapsed(INITIAL_MOVE_TIME) && collector.isDeploymentStopped();
    }
}
