package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.controlboard.IControlBoard;
import edu.wpi.first.wpilibj.GenericHID;

public class RumbleAction implements AutoAction {
    private IControlBoard controlboard;

    private IControlBoard.CONTROLLER controllerToRumble;
    private double percentToRumble;
    private GenericHID.RumbleType directionToRumble;

    public RumbleAction(IControlBoard.CONTROLLER controller, GenericHID.RumbleType direction, double percent) {
        controlboard = Injector.get(IControlBoard.class);
        controllerToRumble = controller;
        percentToRumble = percent;
        directionToRumble = direction;
    }
    @Override
    public void start() {
        switch(directionToRumble){
            case kBothRumble -> {
                controlboard.setFullRumble(controllerToRumble, percentToRumble);
            }
            case kLeftRumble -> {
                controlboard.setLeftRumble(controllerToRumble, percentToRumble);
            }
            case kRightRumble -> {
                controlboard.setRightRumble(controllerToRumble, percentToRumble);
            }
        }
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
