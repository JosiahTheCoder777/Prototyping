package org.tahomarobotics.robot.collector;

import edu.wpi.first.wpilibj2.command.Command;


public class CollectorZeroCommand extends Command {

    private final Collector collector = Collector.getInstance();

    public CollectorZeroCommand() {
        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.setDeployVoltage(CollectorConstants.COLLECTOR_ZERO_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        if (collector.getLeftPivotVelocity() > -0.5) {
            return Math.abs(collector.getLeftDeployVelocity()) < CollectorConstants.COLLECTOR_ZERO_VELOCITY_TOLERANCE
                    && Math.abs(collector.getRightDeployVelocity()) < CollectorConstants.COLLECTOR_ZERO_VELOCITY_TOLERANCE;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        collector.zeroDeploys();
        collector.deployStow();
    }

}

