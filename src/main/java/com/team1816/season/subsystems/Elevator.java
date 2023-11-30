package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.LazyTalonSRX;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
    private IControlBoard controlBoard;

    /**
     * Properties
     */
    private final double ascendPower;
    private final double joltPower;
    private final double descendPower;
    private final double tautPower;

    private boolean downReleased;

    /**
     * States
     */

    private HEIGHT_STATE desiredElevatorHeightState = HEIGHT_STATE.STOP;

    private boolean outputsChanged;

    private boolean rumbleJustStopped = false;

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

        controlBoard = Injector.get(IControlBoard.class);
        this.spoolMotor = factory.getMotor(NAME, "spoolMotor");

        ascendPower = factory.getConstant(NAME, "ascendPower");
        joltPower = factory.getConstant(NAME, "joltPower");
        descendPower = factory.getConstant(NAME, "descendPower");
        tautPower = factory.getConstant(NAME, "tautPower");

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
        if (robotState.actualElevatorHeightState != desiredElevatorHeightState) { // Mismatch + height not met => rumble
            controlBoard.setFullRumble(IControlBoard.CONTROLLER.OPERATOR, desiredElevatorHeightState == HEIGHT_STATE.JOLT ? 0.9 : 0.5);
        } else if (rumbleJustStopped) {
            rumbleJustStopped = false;
            controlBoard.setFullRumble(IControlBoard.CONTROLLER.OPERATOR,0);
        }

        if (robotState.actualElevatorHeightState != desiredElevatorHeightState && isHeightAtTarget()) {
            outputsChanged = true;
            rumbleJustStopped = true;
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
                case HUMAN_COLLECT, EMERGENCY_DOWN -> desiredPower = descendPower;
                case JOLT -> desiredPower = joltPower;
                case SILO_DROP -> desiredPower = ascendPower;
                default -> desiredPower = tautPower; //Stop
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
            case SILO_DROP, JOLT -> {
                if (((LazyTalonSRX) spoolMotor).isRevLimitSwitchClosed() == 1 && !outputsChanged) {
                    return true;
                }
            }
            case HUMAN_COLLECT -> {
                if (((LazyTalonSRX) spoolMotor).isFwdLimitSwitchClosed() == 1 && !outputsChanged) {
                    return true;
                }
            }
            case EMERGENCY_DOWN -> {
                return downReleased;
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

    public void setDownReleased(boolean released) {
        downReleased = released;
    }


    public enum HEIGHT_STATE{
        HUMAN_COLLECT,
        SILO_DROP,
        STOP,
        JOLT,
        EMERGENCY_DOWN
    }
}