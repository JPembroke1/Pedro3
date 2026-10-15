package org.firstinspires.ftc.teamcode.teleOP;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@TeleOp(name = "1. Competition TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {

    private Follower follower;

    private Shooter shooter;
    private Blocker blocker;
    private Intake intake;

    private List<LynxModule> hubs;


    @Override
    public void runOpMode() {

        boolean shooterEnabled = false;

        // =====================================
        // PEDRO 3 DRIVE AND LOCALIZATION
        // =====================================

        follower = Constants.create(hardwareMap);

        // =====================================
        // ROBOT SUBSYSTEMS
        // =====================================

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

        shooterEnabled = false;

        shooter.shooterOff();
        intake.intakeOff();
        blocker.closeBlocker();

        telemetry.addLine("ROBOT READY");
        telemetry.addLine("Drive: FIELD CENTRIC");
        telemetry.addLine("Shooter: OFF");
        telemetry.addLine("Blocker: CLOSED");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // Clear each hub cache exactly once per loop.
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            // =====================================
            // FIELD-CENTRIC MECANUM DRIVE
            // =====================================

            double forward =
                    -gamepad1.left_stick_y;

            double strafe =
                    gamepad1.left_stick_x;

            double turn =
                    gamepad1.right_stick_x;

            DrivePowers drivePowers =
                    ManualDrive.fieldCentric(
                            forward,
                            strafe,
                            turn,
                            follower.pose().heading()
                    );

            follower.manual(drivePowers);
            follower.update();

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

            // =====================================
            // BLOCKER AND FEED
            // =====================================

            boolean feedRequested =
                    gamepad1.right_trigger > 0.5;

            /*
             * Blocker is always closed unless the
             * right trigger is held.
             */
            if (feedRequested) {
                blocker.openBlocker();
            } else {
                blocker.closeBlocker();
            }

            /*
             * The blocker opens immediately on RT,
             * but the intake only feeds once both
             * shooter wheels are ready.
             */
            if (feedRequested && shooter.shooterReady()) {

                intake.feedOn();

            } else if (gamepad1.right_bumper) {

                intake.intakeOn();

            } else if (gamepad1.left_bumper) {

                intake.intakeReverse();

            } else {

                intake.intakeOff();
            }

            // =====================================
            // COMPACT COMPETITION HUD
            // =====================================

            String shooterStatus;

            if (!shooterEnabled) {
                shooterStatus = "OFF";
            } else if (shooter.shooterReady()) {
                shooterStatus = "READY";
            } else {
                shooterStatus = "SPINNING";
            }

            String feedStatus;

            if (!feedRequested) {
                feedStatus = "CLOSED";
            } else if (shooter.shooterReady()) {
                feedStatus = "FEEDING";
            } else {
                feedStatus = "WAITING";
            }

            telemetry.addData(
                    "SHOOTER",
                    shooterStatus
            );

            telemetry.addData(
                    "FEED",
                    feedStatus
            );

            telemetry.addData(
                    "POSE",
                    "X %.1f  Y %.1f  H %.0f",
                    follower.pose().x(),
                    follower.pose().y(),
                    Math.toDegrees(
                            follower.pose().heading()
                    )
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