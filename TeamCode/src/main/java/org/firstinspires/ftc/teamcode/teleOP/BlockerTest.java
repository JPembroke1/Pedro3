package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Blocker Test")
public class BlockerTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo blocker = hardwareMap.get(Servo.class, "blockerServo");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.cross) {
                blocker.setPosition(0.0);
            }

            if (gamepad1.circle) {
                blocker.setPosition(1.0);
            }

            telemetry.addData("Position", blocker.getPosition());
            telemetry.update();
        }
    }
}