// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class LegoAssembler {
    void assemble (StartPosition startPosition, int gamepieces, boolean placeLast, boolean balance) {
        // add(null, null);

    }

    public enum StartPosition {
        WALL(TargetPoseOnField.WallTarget),
        MID(TargetPoseOnField.MidTarget),
        HUMAN(TargetPoseOnField.HumanTarget);

        final TargetPoseOnField targetPoseOnField;
        StartPosition(TargetPoseOnField targetPoseOnField) {
            this.targetPoseOnField = targetPoseOnField;
        }

        public TargetPoseOnField getTargetPoseOnField() {
            return this.targetPoseOnField;
        }
    }

    List<CommandAndReason> commandAndReasons;

    LegoAssembler() {
        commandAndReasons = new ArrayList<>();
    }

    void add(String reason, Command command) {
        commandAndReasons.add(new CommandAndReason(command, reason));
    }

    public List<Command> getCommands() {
        var rv = new ArrayList<Command>(commandAndReasons.size());
        for (var cr: commandAndReasons) {
            rv.add(cr.command);
        }
        return rv;
    }

    public List<String> getReasons() {
        var rv = new ArrayList<String>(commandAndReasons.size());
        for (var cr: commandAndReasons) {
            rv.add(cr.reason);
        }
        return rv;
    }

    public List<String> getReasonsWithCommands() {
        var rv = new ArrayList<String>(commandAndReasons.size());
        for (var cr: commandAndReasons) {
            rv.add(cr.reason + ": " + cr.command);
        }
        return rv;
    }

    public static LegoAssembler getLegoAssembler (StartPosition startPosition, int gamepieces, boolean placeLast, boolean balance) {
        var rv = new LegoAssembler();
        rv.assemble(startPosition, gamepieces, placeLast, balance);
        return rv;
    }

    class CommandAndReason {
        final Command command;
        final String reason;
        CommandAndReason (Command c, String r) {
            this.command = c;
            this.reason = r;
        }
    }
}
