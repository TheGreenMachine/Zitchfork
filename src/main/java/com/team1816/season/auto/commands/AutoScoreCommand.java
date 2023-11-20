package com.team1816.season.auto.commands;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.states.Orchestrator;

public class AutoScoreCommand extends AutoCommand {
    private int cobsToScore;

    public AutoScoreCommand(int cobs) {
        cobsToScore = cobs;
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Auto Score Command!");
        runAction(new ScoreAction(cobsToScore));
    }

    public void done() {
        super.done();
        Orchestrator.runningAutoScore = false;
        GreenLogger.log("Auto Score Command Completed!");
    }
}
