package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
    private final IGreenMotor heightMotor; // Better name needed, like spoolMotor or something similar- descriptive of what it actually does

    /**
     * Properties
     */
    public static final double elevatorZeroOffset = factory.getConstant(NAME, "elevatorZeroOffset");
    public static final double elevatorTicksPerMeter = factory.getConstant(NAME, "elevatorTicksPerMeter");
    public static final double humanCollectHeight = (factory.getConstant(NAME, "humanCollectHeight") + elevatorZeroOffset) * elevatorTicksPerMeter;
    public static final double siloDropHeight = (factory.getConstant(NAME, "siloDropHeight") + elevatorZeroOffset) * elevatorTicksPerMeter;
    public static final double stowHeight = (factory.getConstant(NAME, "stowHeight") + elevatorZeroOffset) * elevatorTicksPerMeter;

    public static final double allowableHeightError = factory.getConstant(NAME, "allowableHeighterror"); // Capitalize the E in error, it is case sensitive

    /**
     * States
     */
    private double desiredHeightTicks = 0;
    private double actualHeightTicks = 0;
    private double actualHeightVelocity = 0; // specify units in comment

    private HEIGHT_STATE desiredElevatorHeightState = HEIGHT_STATE.STOW;

    private boolean outputsChanged;

    /**
     * Logging
     */
    private DoubleLogEntry desiredHeightLogger;
    private DoubleLogEntry actualHeightLogger;
    private DoubleLogEntry actualHeightVelocityLogger; // More descriptive name needed
    private DoubleLogEntry heightCurrentDraw;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param inf            Infrastructure
     * @param rs             RobotState
     */
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        this.heightMotor = factory.getMotor(NAME, "heightMotor");

        if (Constants.kLoggingRobot) {
            //Don't need /Height, as there's only one motor on this elevator
            desiredHeightLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/desiredHeightPosition");
            actualHeightLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/actualHeightPosition");
            actualHeightVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/actualHeightVelocity");
            heightCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/currentDraw");
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
        actualHeightTicks = heightMotor.getSelectedSensorPosition(0);
        actualHeightVelocity = heightMotor.getSelectedSensorVelocity(0);

        if (robotState.actualElevatorHeightState != desiredElevatorHeightState && isHeightAtTarget()) {
            outputsChanged = true;
            robotState.actualElevatorHeightState = desiredElevatorHeightState;
        }

        robotState.actualElevatorHeightMeters = actualHeightTicks / elevatorTicksPerMeter;

        if (Constants.kLoggingRobot) {
            desiredHeightLogger.append(desiredHeightTicks);
            actualHeightLogger.append(actualHeightTicks);
            actualHeightVelocityLogger.append(actualHeightVelocity);
            heightCurrentDraw.append(heightMotor.getOutputCurrent());
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;

            double height = 0;

            switch (desiredElevatorHeightState) {
                case HUMAN_COLLECT -> height = humanCollectHeight;
                case SILO_DROP -> height = siloDropHeight;
                case STOW -> height = stowHeight;
            }
            desiredHeightTicks = height;

            heightMotor.set(ControlMode.Position, desiredHeightTicks);
        }
    }

    /**
     * Checks if the elevator's actual height is within range of its target height
     *
     * @return boolean - true if elevator is within range of target
     */
    private boolean isHeightAtTarget(){
        return Math.abs(desiredHeightTicks - actualHeightTicks) < (allowableHeightError * elevatorTicksPerMeter) && !outputsChanged;
    }

    /**
     * Checks if the elevator's actual height is within the meter range of its target height
     *
     * @param meterRange
     * @return boolean - true if elevator is within the meter range of target
     */
    private boolean isHeightAtTarget(double meterRange){
        return Math.abs(desiredHeightTicks - actualHeightTicks) < (meterRange * elevatorTicksPerMeter) && !outputsChanged;
    }

    @Override
    public void zeroSensors() {
            // This method sets the height motor's 0 position to its current position no matter where it is.
            // use setSelectedSensorPosition()
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean testSubsystem() {
        return false;
    }


    public enum HEIGHT_STATE{ // Remove the spaces in between values it's weird
        HUMAN_COLLECT,

        SILO_DROP,

        STOW
    }
}