package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TargetPoseOnField.TargetPosition;

public class LegoAssembler {
    protected void assemble (TargetPosition startPosition, int gamepieces, boolean placeLast, boolean balance) {
        // add(null, null);
    }

    protected List<CommandAndReason> commandAndReasons;

    protected LegoAssembler() {
        commandAndReasons = new ArrayList<>();
    }

    protected void add(String reason, Command command) {
        commandAndReasons.add(new CommandAndReason(command, reason));
    }

    protected void add(Command command, String reason) {
        commandAndReasons.add(new CommandAndReason(command, reason));
    }

    protected void add(Command command) {
        commandAndReasons.add(new CommandAndReason(command, null));
    }

    protected void add(String reason) {
        commandAndReasons.add(new CommandAndReason(null, reason));
    }

    // TODO use streams on these?
    public List<Command> getCommands() {
        var rv = new ArrayList<Command>(commandAndReasons.size());
        for (var cr: commandAndReasons) {
            if (cr.command != null) {
                rv.add(cr.command);
            }
        }
        return rv;
    }

    public List<String> getReasonsWithCommands() {
        var rv = new ArrayList<String>(commandAndReasons.size());
        for (var cr: commandAndReasons) {
            rv.add(cr.toString());
        }
        return rv;
    }

    public static LegoAssembler getLegoAssembler (TargetPosition startPosition, int gamepieces, boolean placeLast, boolean balance) {
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

        @Override 
        public String toString() {
            StringBuilder sb = new StringBuilder();
            if (reason != null) {
                sb.append(reason);
                if (command != null) sb.append(": ");
            }
            if (command != null) {
                sb.append(command.toString());
            }
            return sb.toString();
        }
    }
}
