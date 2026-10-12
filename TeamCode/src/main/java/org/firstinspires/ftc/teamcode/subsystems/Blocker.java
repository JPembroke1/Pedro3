package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Blocker {

    public static double BLOCKER_OPEN_POSITION = 0.0;
    public static double BLOCKER_CLOSED_POSITION = 1.0;
    private final Servo blockerServo;

public Blocker(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, "blockerServo");
    }
    public void openBlocker() {
        blockerServo.setPosition(BLOCKER_OPEN_POSITION);
    }
    public void closeBlocker() {
        blockerServo.setPosition(BLOCKER_CLOSED_POSITION);
    }
}
