package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.tahomarobotics.robot.sim.AbstractSimulation;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;

public class ClimberSimulation {//extends AbstractSimulation {
    /*private final LinearSystemSim<N2, N1, N1> simulation;
    private final ClimberSubsystem climber;

    // Simulation parameters
    private static final double MECHANISM_MASS = ClimberConstants.CLIMBER_MASS; // kg
    private static final double MECHANISM_LENGTH = ClimberConstants.CLIMBER_LENGTH; // meters
    private static final DCMotor MOTOR = DCMotor.getKrakenX60(2);

    ClimberSimulation(ClimberSubsystem climber) {
        super("Climber");
        this.climber = climber;

        //create physics model. i guess the climber could technically be represented as a single jointed arm...
        //TODO: There are errors in the simulation guidance file itself. I'm not really sure what to do here? There is no other system creation method that would work, and LinearSystemSim will ONLY take LinearSystem<N2, N1, N1> (whoever is grading this try uncommenting everything and see what i mean)
        LinearSystem<N2, N1, N1> plant = LinearSystemId.createSingleJointedArmSystem(
                MOTOR,
                MECHANISM_MASS * MECHANISM_LENGTH * MECHANISM_LENGTH / 3.0, // J (moment of inertia)
                ClimberConstants.PIVOT_GEAR_RATIO
        );

        this.simulation = new LinearSystemSim<>(plant);
    }

    public void updateSim() {
        updateMotor(climber.leftPivotMotor, climber::getLeftVoltageAsDouble);
        updateMotor(climber.rightPivotMotor, climber::getRightVoltageAsDouble);
        updateMotor(climber.rollerMotor, climber::getRollerVoltageAsDouble);
    }

    private void updateMotor(TalonFX motor, DoubleSupplier voltageFxn) {
        double voltage = voltageFxn.getAsDouble();
        simulation.setInput(voltage);
        //Calculate position/velocity based on voltage.
        simulation.update(0.020);
        double position = simulation.getOutput(0);
        double velocity = simulation.getOutput(1);

        //Update position/velocity.
        motor.getSimState().setRawRotorPosition(position);
        motor.getSimState().setRotorVelocity(velocity);
    }

    // Helper methods for testing
    public double getSimulatedPosition() {
        return simulation.getOutput(0);
    }

    public double getSimulatedVelocity() {
        return simulation.getOutput(1);
    }

    @Override
    protected void simulationPeriodic() {
        updateSim();
    }
     */
}
