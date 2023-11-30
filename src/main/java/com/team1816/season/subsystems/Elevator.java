package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.LazyTalonSRX;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import jakarta.inject.Inject;
import jakarta.inject.Singleton;


@Singleton
public class Elevator extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "elevator";

    /**
     * Components
     */
    private final IGreenMotor spoolMotor;

    /**
     * Properties
     */
    private final double ascendPower;
    private final double descendPower;
    private final double taughtPower;

    /**
     * States
     */

    private HEIGHT_STATE desiredElevatorHeightState = HEIGHT_STATE.STOP;

    private boolean outputsChanged;

    /**
     * Logging
     */
    private DoubleLogEntry spoolCurrentDraw;


    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param inf            Infrastructure
     * @param rs             RobotState
     */
    @Inject
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        this.spoolMotor = factory.getMotor(NAME, "spoolMotor");

        ascendPower = factory.getConstant(NAME, "ascendPower");
        descendPower = factory.getConstant(NAME, "descendPower");
        taughtPower = factory.getConstant(NAME, "taughtPower");

        if (Constants.kLoggingRobot) {
            spoolCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/currentDraw");
        }
    }

    /**
     * Sets the desired elevator height in meters
     *
     * @param desiredElevatorExtensionState
     */
    public void setDesiredElevatorHeightState(HEIGHT_STATE desiredElevatorExtensionState){
        this.desiredElevatorHeightState = desiredElevatorExtensionState;
        outputsChanged = true;
    }

    @Override
    public void readFromHardware() {

        if (robotState.actualElevatorHeightState != desiredElevatorHeightState && isHeightAtTarget()) {
            outputsChanged = true;
            robotState.actualElevatorHeightState = desiredElevatorHeightState;
        }

        if (Constants.kLoggingRobot) {
            spoolCurrentDraw.append(spoolMotor.getOutputCurrent());
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            double desiredPower;

            switch (desiredElevatorHeightState) {
                case HUMAN_COLLECT -> desiredPower = descendPower;
                case SILO_DROP -> desiredPower = ascendPower;
                default -> desiredPower = taughtPower; //Stop
            }

            spoolMotor.set(ControlMode.PercentOutput, desiredPower);

        }
    }

    /**
     * Checks if the elevator's actual height is within range of its target height
     *
     * @return boolean - true if elevator is within range of target
     */
    private boolean isHeightAtTarget(){
        switch (desiredElevatorHeightState) {
            case SILO_DROP -> {
                if (((LazyTalonSRX) spoolMotor).isRevLimitSwitchClosed() == 1 && !outputsChanged) {
                    return true;
                }
            }
            case HUMAN_COLLECT -> {
                if (((LazyTalonSRX) spoolMotor).isFwdLimitSwitchClosed() == 1 && !outputsChanged) {
                    return true;
                }
            }
            default -> {
                return true;
            }
        }

        return false;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean testSubsystem() {
        return false;
    }


    public enum HEIGHT_STATE{
        HUMAN_COLLECT,
        SILO_DROP,
        STOP
    }
}