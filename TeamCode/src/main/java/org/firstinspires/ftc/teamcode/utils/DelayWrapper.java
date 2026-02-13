package org.firstinspires.ftc.teamcode.utils;

import java.util.function.BooleanSupplier;

public class DelayWrapper implements BooleanSupplier {

    long startTime = -1;

    long delayMillis;

    BooleanSupplier eventFunction;

    public DelayWrapper(long delayMillis, BooleanSupplier eventFunction) {

        this.delayMillis = delayMillis;

        this.eventFunction = eventFunction;

    }

 

    public boolean getAsBoolean() {

        if(eventFunction.getAsBoolean() && startTime == -1) {

            startTime = System.currentTimeMillis();

            System.out.println("Event triggered - starting timer");

        }

 

        if(startTime != -1 && System.currentTimeMillis() - startTime > delayMillis) {

            return true;

        }

        return false;

    }

}
