package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

@Singleton
public class Collector extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "collector";

    /**
     * Components
     */
    private final IGreenMotor intakeMotor;

    /**
     * Properties
     */
    public final double intakeSpeed;
    public final double outtakeSpeed;

    /**
     * Logging
     */
    private DoubleLogEntry intakeVelocityLogger;
    private DoubleLogEntry intakeCurrentDraw;

    /**
     * States
     */
    private COLLECTOR_STATE desiredState = COLLECTOR_STATE.STOP;
    private double collectorVelocity = 0;
    private boolean outputsChanged = false;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public Collector(String name, Infrastructure inf, RobotState rs) {
        super(name, inf, rs);
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
        intakeSpeed = factory.getConstant(NAME, "intakeSpeed", -0.5);
        outtakeSpeed = factory.getConstant(NAME, "outtakeSpeed", 0.5);
        if (Constants.kLoggingRobot) {
            intakeVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "Collector/intakeVelocity");
            intakeCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Collector/currentDraw");
        }
    }

    /**
     * Sets the desired state of the collector
     *
     * @param desiredState COLLECTOR_STATE
     */
    public void setDesiredState(COLLECTOR_STATE desiredState) {
        this.desiredState = desiredState;
        outputsChanged = true;
    }

    /**
     * Reads actual outputs from intake motor
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        collectorVelocity = intakeMotor.getSelectedSensorVelocity(0);

        if (robotState.actualCollectorState != desiredState) {
            robotState.actualCollectorState = desiredState;
        }

        if (Constants.kLoggingRobot) {
            intakeVelocityLogger.append(collectorVelocity);
            intakeCurrentDraw.append(intakeMotor.getOutputCurrent());
        }
    }

    /**
     * Writes outputs to intake motor
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP -> {
                    intakeMotor.set(ControlMode.Velocity, 0);
                }
                case INTAKE -> {
                    intakeMotor.set(ControlMode.Velocity, intakeSpeed);
                }
                case OUTTAKE -> {
                    intakeMotor.set(ControlMode.Velocity, outtakeSpeed);
                }
            }
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    /**
     * Tests the collector subsystem, returns true if tests passed
     *
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        return false;
    }

    /**
     * Returns the desired collector state
     *
     * @return desired collector state
     */
    public COLLECTOR_STATE getDesiredCollectorState() {
        return desiredState;
    }

    /**
     * Returns the collector velocity
     *
     * @return collector velocity
     */
    public double getCollectorVelocity() {
        return collectorVelocity;
    }

    /**
     * Base enum for collector
     */
    public enum COLLECTOR_STATE {
        STOP,
        INTAKE,
        OUTTAKE
    }
}