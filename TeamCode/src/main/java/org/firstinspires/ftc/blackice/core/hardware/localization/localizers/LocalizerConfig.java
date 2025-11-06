package org.firstinspires.ftc.blackice.core.hardware.localization.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.hardware.localization.Localizer;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionTracker;

public interface LocalizerConfig {
    Localizer build(HardwareMap hardwareMap) throws InterruptedException;
    
    default MotionTracker createMotionTracker(HardwareMap hardwareMap) {
        return new MotionTracker(hardwareMap, build(hardwareMap));
    }
}
