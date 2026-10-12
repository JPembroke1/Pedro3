package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "1. Competition TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {

    private final Shooter shooter = new Shooter(hardwareMap);
    private final Blocker blocker = new Blocker(hardwareMap);

    private final Intake intake = new Intake(hardwareMap);


    private boolean shooterEnabled = false;
    private boolean blockerEnabled = false;

    @Override
    public void runOpMode() {

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake
            if (gamepad1.right_bumper) {
                intake.intakeOn();
            } else {
                intake.intakeOff();
            }
            // Shooting with Check of RPM
            if (gamepad1.right_trigger > 0.5 && shooter.getRPM() > Shooter.TARGET_RPM -100) {
                intake.intakeOn();
                blockerEnabled = false;
            } else if (gamepad1.left_trigger > 0.5) {
                blockerEnabled = true;
            }

            // Shooter ON
            if (gamepad1.cross) {
                shooterEnabled = true;
            }

            // Shooter OFF
            if (gamepad1.circle) {
                shooterEnabled = false;
            }


            if (shooterEnabled) {
                shooter.shooterOn();
            } else {
                shooter.shooterOff();
            }

            if (blockerEnabled) {
                blocker.closeBlocker();
            } else {
                blocker.openBlocker();
            }

            telemetry.addLine("========== SHOOTER ==========");

            telemetry.addData("RPM", "%.0f", shooter.getRPM());
            telemetry.addData("TARGET", "%.0f", Shooter.TARGET_RPM);
            telemetry.addData("ERROR", "%.0f", Shooter.TARGET_RPM - shooter.getRPM());

            telemetry.addLine();

            telemetry.addData("kF", "%.3f", Shooter.kF);
            telemetry.addData("kP", "%.5f", Shooter.kP);

            telemetry.update();

        }
    }
}