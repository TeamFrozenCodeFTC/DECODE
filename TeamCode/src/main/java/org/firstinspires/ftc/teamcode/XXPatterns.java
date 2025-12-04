package org.firstinspires.ftc.teamcode;

import static java.util.Map.entry;

import java.util.HashMap;
import java.util.Map;

public class XXPatterns {
    Map<String, Map<String, String>> actions = new HashMap<>();

    public XXPatterns() {
        actions.put("PPG", Map.ofEntries(
                entry("PPG","LLL"),
                entry("PGP","LRR"),
                entry("GPP","RRR")
        ));
        actions.put("GPP", Map.ofEntries(
                entry("PPG","RRR"),
                entry("PGP","RLL"),
                entry("GPP","LLL")
        ));
    }

    public String get(String spindexPattern, String targetPattern) {
        return actions.get(spindexPattern).get(targetPattern);
    }

    static int START_MOTIF = 1;
    static int LAUNCH_MOTIF = 2;
    static int SPINDEX_TURNING = 3;

    public void doStuff() {

        int state = 0;
        String actionSequence = "";
        int actionSequenceIndex = 0;

        if(state == START_MOTIF) {
            actionSequence = get("PPG", "PGP");
            actionSequenceIndex = 0;
            state = LAUNCH_MOTIF;

        } else if(state == LAUNCH_MOTIF) {
            char actionCode = actionSequence.charAt(actionSequenceIndex);
            if(actionCode == 'L') {
                // spindexer.setSlot(spindexer.getCurrentSlot()-1)
            } else {
                // spindexer.setSlot(spindexer.getCurrentSlot()+1)
            }
            state = SPINDEX_TURNING;

            actionSequenceIndex++;
            if(actionSequenceIndex >= 3) {
                state = 0;
            }
        }
    }
}
