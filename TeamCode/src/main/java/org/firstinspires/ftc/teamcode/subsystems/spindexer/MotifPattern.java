package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import org.firstinspires.ftc.teamcode.Artifact;

public enum MotifPattern {
    GPP(new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE}),
    PGP(new Artifact[]{Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE}),
    PPG(new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN});
    
    private final Artifact[] pattern;
    
    MotifPattern(Artifact[] pattern) {
        this.pattern = pattern;
    }
    
    public Artifact[] getPattern() {
        return pattern;
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        for(Artifact a : pattern) {
            if(a == Artifact.GREEN) {
                buf.append("G");
            } else if (a == Artifact.PURPLE) {
                buf.append("P");
            } else {
                buf.append("N");
            }
        }
        return buf.toString();
    }
}
