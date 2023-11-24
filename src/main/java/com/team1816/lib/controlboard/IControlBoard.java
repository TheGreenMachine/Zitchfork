package com.team1816.lib.controlboard;

/**
 * A basic interface for accessing controller outputs
 *
 * @see ControlBoard
 */
public interface IControlBoard {
    boolean getAsBool(String getName);

    double getAsDouble(String getName);

    //TODO don't bring these to the next season, this is a terrible implementation
    void setFullRumble(CONTROLLER controller, double percent);

    void setLeftRumble(CONTROLLER controller, double percent);

    void setRightRumble(CONTROLLER controller, double percent);

    enum CONTROLLER {
        DRIVER,
        OPERATOR
    }
}
