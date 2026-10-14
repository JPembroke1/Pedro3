package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "1. Competition TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {

    private Shooter shooter;
    private Blocker blocker;
    private Intake intake;

    private boolean shooterEnabled = false;

    private boolean blockerClosed = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap);
        blocker = new Blocker(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =============================
            // SHOOTER TOGGLE
            // =============================

            if (gamepad1.cross) {
                shooterEnabled = true;
            }

            if (gamepad1.circle) {
                shooterEnabled = false;
            }

            if (shooterEnabled) {
                shooter.shooterOn();
            } else {
                shooter.shooterOff();
            }

            // =============================
            // BLOCKER
            // =============================

            if (gamepad1.dpad_up && !lastDpadUp) {
                blockerClosed = true;
            }

            if (gamepad1.dpad_down && !lastDpadDown) {
                blockerClosed = false;
            }

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            if (blockerClosed) {
                blocker.closeBlocker();
            } else {
                blocker.openBlocker();
            }

            // =============================
            // FEED / INTAKE
            // =============================

            if (gamepad1.right_trigger > 0.5
                    && shooter.shooterReady(200)) {

                intake.feedOn();

            } else if (gamepad1.right_bumper) {

                intake.intakeOn();

            } else if (gamepad1.left_bumper) {

                intake.intakeReverse();

            } else {

                intake.intakeOff();
            }

            // =============================
            // SHOOTER TELEMETRY
            // =============================

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData(
                    "L Wheel RPM",
                    "%.0f",
                    shooter.getLeftRPM());

            telemetry.addData(
                    "L Motor RPM",
                    "%.0f",
                    shooter.getLeftMotorRPM());

            telemetry.addData(
                    "L Target",
                    "%.0f",
                    Shooter.TARGET_RPM_LEFT);

            telemetry.addData(
                    "L Power",
                    "%.2f",
                    shooter.getLeftPower());

            telemetry.addLine("");

            telemetry.addData(
                    "R Wheel RPM",
                    "%.0f",
                    shooter.getRightRPM());

            telemetry.addData(
                    "R Motor RPM",
                    "%.0f",
                    shooter.getRightMotorRPM());

            telemetry.addData(
                    "R Target",
                    "%.0f",
                    Shooter.TARGET_RPM_RIGHT);

            telemetry.addData(
                    "R Power",
                    "%.2f",
                    shooter.getRightPower());

            telemetry.addLine("");

            telemetry.addData(
                    "Ready",
                    shooter.shooterReady(200) ? "YES" : "NO");

            telemetry.addData(
                    "Shooter",
                    shooterEnabled ? "ON" : "OFF");

            telemetry.addData(
                    "Blocker",
                    blockerClosed ? "CLOSED" : "OPEN");

            telemetry.update();
        }
    }
}