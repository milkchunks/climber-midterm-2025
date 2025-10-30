/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;

import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemBase {
    private final static Collector INSTANCE = new Collector();

    // -- Member Variables --

    // Hardware

    private final TalonFX leftMotor; // Leader
    private final TalonFX rightMotor; // Follower
    private final TalonFX collectorMotor;

    // Status Signals

    private final StatusSignal<Angle> leftDeploymentPosition;
    private final StatusSignal<AngularVelocity> leftDeploymentVelocity, rightDeploymentVelocity;
    private final StatusSignal<Current> collectorCurrent, leftDeploymentCurrent, rightDeploymentCurrent;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0);

    private final VoltageOut voltageControl = new VoltageOut(0.0);

    // State
    private TargetDeploymentState targetDeploymentState = TargetDeploymentState.ZEROED;
    private TargetCollectorState targetCollectorState = TargetCollectorState.DISABLED;
    private GamePiece collectionMode = GamePiece.CORAL;

    private Collector() {
        // Create hardware

        leftMotor = new TalonFX(RobotMap.COLLECTOR_LEFT_MOTOR);
        rightMotor = new TalonFX(RobotMap.COLLECTOR_RIGHT_MOTOR);
        collectorMotor = new TalonFX(RobotMap.COLLECTOR_COLLECT_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Collector Left Motor", leftMotor, deploymentLeftMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX(
                "Collector Right Motor", rightMotor,
                deploymentLeftMotorConfiguration.withMotorOutput(deploymentRightMotorOutputConfiguration)
        );
        RobustConfigurator.tryConfigureTalonFX("Collector Collect Motor", collectorMotor, collectorMotorConfig);

        // Bind status signals

        leftDeploymentPosition = leftMotor.getPosition();
        leftDeploymentVelocity = leftMotor.getVelocity();
        rightDeploymentVelocity = rightMotor.getVelocity();

        leftDeploymentCurrent = leftMotor.getSupplyCurrent();
        rightDeploymentCurrent = rightMotor.getSupplyCurrent();
        collectorCurrent = collectorMotor.getSupplyCurrent();
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor, collectorMotor);
    }

    public static Collector getInstance() {
        return INSTANCE;
    }

    public void setZeroingVoltage() {
        leftMotor.setVoltage(DEPLOYMENT_ZEROING_VOLTAGE);
        rightMotor.setVoltage(DEPLOYMENT_ZEROING_VOLTAGE);
    }

    public boolean isDeploymentStopped() {
        return leftDeploymentVelocity.getValueAsDouble() < DEPLOYMENT_MOVING_VELOCITY_THRESHOLD &&
                rightDeploymentVelocity.getValueAsDouble() < DEPLOYMENT_MOVING_VELOCITY_THRESHOLD;
    }

    public void zero() {
        if (RobotState.isDisabled()) { return; }

        leftMotor.setPosition(TargetDeploymentState.ZEROED.angle);
        rightMotor.setPosition(TargetDeploymentState.ZEROED.angle);

        targetDeploymentState = TargetDeploymentState.STOW;
        setDeploymentControl(targetDeploymentState);
    }

    public TargetDeploymentState getTargetDeploymentState() {
        return targetDeploymentState;
    }

    private void setDeploymentControl(TargetDeploymentState state) {
        leftMotor.setControl(positionControl.withPosition(state.angle));
        rightMotor.setControl(positionControl);
    }

    /**
     * Sync the applied control request for the deployment motors to the target state.
     */
    private void syncDeploymentControl() {
        // Ensure we transition to the correct mode.
        if (isDeploymentCollecting()) {
            targetDeploymentState = TargetDeploymentState.CORAL_COLLECT;
        }

        setDeploymentControl(targetDeploymentState);
    }

    public void setTargetDeploymentState(TargetDeploymentState state) {
        if (checkZeroingGuard()) { return; }

        targetDeploymentState = state;
        syncDeploymentControl();
    }

    public TargetCollectorState getTargetCollectorState() {
        return targetCollectorState;
    }
}