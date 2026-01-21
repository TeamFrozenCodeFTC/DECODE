package org.firstinspires.ftc.teamcode;

public enum Artifact {
    NONE,
    PURPLE,
    GREEN;
    
    public boolean isArtifact() {
        return this != NONE;
    }
    
    public Artifact oppositeColor() {
        return this == Artifact.PURPLE ? Artifact.GREEN : Artifact.PURPLE;
    }
    
    public static Artifact[] getHumanPlayerPattern() {
        return new Artifact[]
            {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
    }
    
    public static Artifact[] getEmptyPattern() {
        return new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};
    }
    
    public static boolean patternIsPGP(Artifact[] pattern) {
        return pattern[0] == Artifact.PURPLE
            && pattern[1] == Artifact.GREEN
            && pattern[2] == Artifact.PURPLE;
    }
}
