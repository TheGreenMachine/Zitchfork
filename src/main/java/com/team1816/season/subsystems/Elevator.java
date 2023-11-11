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
    private final IGreenMotor heightMotor;

    /**
     * Properties
     */
    public static final double elevatorZitchforkOffset = factory.getConstant(NAME, "elevatorZitchforkOffset");
    public static final double elevatorTicksPerMeter = factory.getConstant(NAME, "elevatorTicksPerMeter");
    public static final double human_collectHeight = (factory.getConstant(NAME, "human_collectHeight") + elevatorZitchforkOffset) * elevatorTicksPerMeter;
    public static final double silo_dropHeight = (factory.getConstant(NAME, "silo_dropHeight") + elevatorZitchforkOffset) * elevatorTicksPerMeter;

    public static final double allowableHeightError = factory.getConstant(NAME, "allowableHeighterror");

    /**
     * States
     */
    private double desiredHeightTicks = 0;
    private double actualHeightTicks = 0;
    private double actualHeightVelocity = 0;
    private HEIGHT_STATE desiredElevatorHeightState = HEIGHT_STATE.HUMAN_COLLECT;

    private boolean outputsChanged;

    /**
     * Logging
     */

    private DoubleLogEntry desiredHeightLogger;
    private DoubleLogEntry actualHeightLogger;
    private DoubleLogEntry actualHeightVelocityLogger;
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
            desiredHeightLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/desiredHeightPosition");
            actualHeightLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/actualHeightPosition");
            actualHeightVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/actualHeightVelocity");
            heightCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Height/currentDraw");
        }
    }

    public void setDesiredElevatorExtensionState(HEIGHT_STATE desiredElevatorExtensionState){
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
                case HUMAN_COLLECT -> height = human_collectHeight;
                case SILO_DROP -> height = silo_dropHeight;
            }
            desiredHeightTicks = height;

            heightMotor.set(ControlMode.Position, desiredHeightTicks);
        }
    }

    private boolean isHeightAtTarget(){
        return Math.abs(desiredHeightTicks - actualHeightTicks) < (allowableHeightError * elevatorTicksPerMeter) && !outputsChanged;
    }
    private boolean isHeightAtTarget(double meterRange){
        return Math.abs(desiredHeightTicks - actualHeightTicks) < (meterRange * elevatorTicksPerMeter) && !outputsChanged;
    }

    @Override
    public void zeroSensors() {
        //maybe zero the height motor, don't know what position to zero it at yet
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

        SILO_DROP
    }
}