package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@TeleOp(name = "1. shooterTestTeleOp", group = "TeleOp")
public class shooterTestTeleOp extends LinearOpMode {

    private Shooter shooter;
    private Blocker blocker;
    private Intake intake;

    private List<LynxModule> hubs;

    private boolean shooterEnabled = false;

    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap);
        blocker = new Blocker(hardwareMap);
        intake = new Intake(hardwareMap);

        // =====================================
        // LYNX BULK CACHING
        // =====================================

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }

        // =====================================
        // SAFETY INITIALISATION
        // =====================================

        shooter.shooterOff();
        intake.intakeOff();
        blocker.closeBlocker();

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =====================================
            // CLEAR BULK CACHE
            // =====================================

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            // =====================================
            // SHOOTER CONTROL
            // =====================================

            if (gamepad1.cross) {
                shooterEnabled = true;
            }

            if (gamepad1.circle) {
                shooterEnabled = false;
            }

            if (shooterEnabled) {
                shooter.update();
            } else {
                shooter.shooterOff();
            }

            // Default blocker state
            blocker.closeBlocker();

            // =====================================
            // FEED / INTAKE
            // =====================================

            if (gamepad1.right_trigger > 0.5) {

                blocker.openBlocker();

                if (shooter.shooterReady()) {
                    intake.feedOn();
                } else {
                    intake.intakeOff();
                }

            } else if (gamepad1.right_bumper) {

                intake.intakeOn();

            } else if (gamepad1.left_bumper) {

                intake.intakeReverse();

            } else {

                intake.intakeOff();
            }

            // =====================================
            // DRIVER HUD
            // =====================================

            telemetry.addData(
                    "Shooter",
                    shooterEnabled ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Ready",
                    shooter.shooterReady() ? "YES" : "NO"
            );

            telemetry.addData(
                    "Left RPM",
                    "%.0f",
                    shooter.getLeftRPM()
            );

            telemetry.addData(
                    "Right RPM",
                    "%.0f",
                    shooter.getRightRPM()
            );

            telemetry.addLine(
                    shooter.shooterReady()
                            ? "SHOOT"
                            : "SPINNING UP"
            );

            telemetry.update();
        }

        // =====================================
        // SAFETY SHUTDOWN
        // =====================================

        shooter.shooterOff();
        intake.intakeOff();
        blocker.closeBlocker();
    }
}