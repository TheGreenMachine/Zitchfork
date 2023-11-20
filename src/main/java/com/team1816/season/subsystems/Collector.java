package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

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
    public final double intakePower;
    public final double outtakePower;

    // Log intakeMotor's velocity
    // Log intakeMotor's current draw
    // Both can be modeled after Zero's collector logging

    /**
     * States
     */
    private COLLECTOR_STATE desiredState = COLLECTOR_STATE.STOP;
    private boolean outputsChanged = false;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    public Collector(String name, Infrastructure inf, RobotState rs) {
        super(name, inf, rs);
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
        intakePower = factory.getConstant(NAME, "intakePower", 0.70);
        outtakePower = factory.getConstant(NAME, "outtakePower", 0.45);
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
        if (robotState.actualCollectorState != desiredState) {
            robotState.actualCollectorState = desiredState;
        }

        //Update logged values here
        //Appending to logmanagers also happens here
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
                    intakeMotor.set(ControlMode.Velocity, intakePower); // Think about this. Is this the right value to give in velocity mode? Is velocity mode appropriate for this subsystem?
                }
                case OUTTAKE -> {
                    intakeMotor.set(ControlMode.Velocity, outtakePower);
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

    // You may want getter methods for desiredState and the motor velocity for use in controls
    /**
     * Base enum for collector
     */
    public enum COLLECTOR_STATE {
        STOP,
        INTAKE,
        OUTTAKE
    }
}