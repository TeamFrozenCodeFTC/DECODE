package org.firstinspires.ftc.teamcode.blackice.core;

interface Command {
    void start();
    void update();
    boolean isFinished();
    void end();
}
