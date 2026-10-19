package org.firstinspires.ftc.teamcode.teleOP;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;


@TeleOp(
        name = "1. Competition TeleOp",
        group = "TeleOp"
)
public class CompetitionTeleOp extends LinearOpMode {

    // =====================================
    // SUBSYSTEMS
    // =====================================

    private Follower follower;

    private Shooter shooter;
    private Blocker blocker;
    private Intake intake;

    private List<LynxModule> hubs;


    // =====================================
    // FEED SPIN-UP TIMER
    // =====================================

    private static final long FEED_SPINUP_TIME_MS = 500;

    private long feedRequestStartNanos = 0L;


    // =====================================
    // RUN OPMODE
    // =====================================

    @Override
    public void runOpMode() {

        // =====================================
        // SET UP ROBOT
        // =====================================

        follower = Constants.create(hardwareMap);

        shooter = new Shooter(hardwareMap);
        blocker = new Blocker(hardwareMap);
        intake = new Intake(hardwareMap);


        // =====================================
        // BULK CACHING
        // =====================================

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }


        // =====================================
        // SAFE START
        // =====================================

        shooter.shooterOff();
        intake.intakeOff();
        blocker.closeBlocker();

        telemetry.addLine("ROBOT READY");
        telemetry.addLine("Cross/A: Shooter on");
        telemetry.addLine("Circle/B: Shooter idle");
        telemetry.addLine("Right trigger: Automatic feed");
        telemetry.addLine("Right bumper: Intake");
        telemetry.addLine("Left bumper: Reverse intake");

        telemetry.addData(
                "Left shoot power",
                "%.3f",
                Shooter.LEFT_SHOOT_POWER
        );

        telemetry.addData(
                "Back shoot power",
                "%.3f",
                Shooter.BACK_SHOOT_POWER
        );

        telemetry.update();


        // =====================================
        // WAIT FOR START
        // =====================================

        waitForStart();

        if (isStopRequested()) {
            safeShutdown();
            return;
        }


        // Start with the shooter running slowly.
        shooter.shooterIdle();


        // =====================================
        // MAIN LOOP
        // =====================================

        while (opModeIsActive()) {

            // =====================================
            // CLEAR BULK CACHE
            // =====================================

            /*
             * Clear the cache once at the beginning
             * of every control loop.
             */
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }


            // =====================================
            // DRIVE
            // =====================================

            // Forward remains linear.
            double forward =
                    -gamepad1.left_stick_y;

            /*
             * Cubed input provides finer control
             * near the centre of the joystick.
             */
            double strafe =
                    -Math.pow(
                            gamepad1.left_stick_x,
                            3
                    );

            double turn =
                    -Math.pow(
                            gamepad1.right_stick_x,
                            3
                    );

            follower.manual(
                    new DrivePowers(
                            forward,
                            strafe,
                            turn
                    )
            );

            follower.update();


            // =====================================
            // DRIVER INPUTS
            // =====================================

            boolean feedRequested =
                    gamepad1.right_trigger > 0.5;

            boolean intakeRequested =
                    gamepad1.right_bumper;

            boolean reverseRequested =
                    gamepad1.left_bumper;

            boolean shooterRequested =
                    gamepad1.cross;

            boolean idleRequested =
                    gamepad1.circle;


            // =====================================
            // SHOOTER MODE CONTROL
            // =====================================

            /*
             * Control priority:
             *
             * 1. Feed request
             * 2. Manual shooter request
             * 3. Intake or reverse
             * 4. Manual idle request
             *
             * Pulling the feed trigger automatically
             * commands shooting power.
             *
             * The selected mode remains active until
             * another command changes it.
             */
            if (feedRequested) {

                shooter.shooterOn();

            } else if (shooterRequested) {

                shooter.shooterOn();

            } else if (intakeRequested || reverseRequested) {

                /*
                 * Reduce shooter power while operating
                 * the intake normally.
                 */
                shooter.shooterIdle();

            } else if (idleRequested) {

                shooter.shooterIdle();
            }


            // =====================================
            // APPLY SHOOTER POWER
            // =====================================

            /*
             * This must run once every loop.
             *
             * It applies the currently selected shooting,
             * idle, or off powers. It also allows live
             * configuration changes to take effect.
             */
            shooter.update();


            // =====================================
            // READ SHOOTER VALUES ONCE
            // =====================================

            /*
             * These values are diagnostic only in this
             * open-loop version. They do not regulate
             * motor power.
             */
            double leftRPM =
                    shooter.getLeftRPM();

            double backRPM =
                    shooter.getBackRPM();

            double leftTPS =
                    shooter.getLeftTicksPerSecond();

            double backTPS =
                    shooter.getBackTicksPerSecond();


            // =====================================
            // FEED SPIN-UP TIMING
            // =====================================

            if (feedRequested) {

                /*
                 * Start the timer on the first loop where
                 * the right trigger is held.
                 */
                if (feedRequestStartNanos == 0L) {
                    feedRequestStartNanos =
                            System.nanoTime();
                }

            } else {

                /*
                 * Releasing the trigger resets the timer.
                 * The next feed request must wait through
                 * the spin-up period again.
                 */
                feedRequestStartNanos = 0L;
            }

            double feedRequestTimeMs = 0.0;

            if (feedRequestStartNanos != 0L) {
                feedRequestTimeMs =
                        (System.nanoTime()
                                - feedRequestStartNanos)
                                / 1_000_000.0;
            }

            boolean feedDelayComplete =
                    feedRequested
                            && feedRequestTimeMs
                            >= FEED_SPINUP_TIME_MS;


            // =====================================
            // FEED / INTAKE CONTROL
            // =====================================

            if (feedRequested) {

                /*
                 * During this direct-power test there is
                 * no RPM-ready interlock.
                 *
                 * Instead, feeding begins after the fixed
                 * spin-up delay.
                 */
                if (feedDelayComplete) {

                    blocker.openBlocker();
                    intake.feedOn();

                } else {

                    blocker.closeBlocker();
                    intake.intakeOff();
                }

            } else {

                /*
                 * The blocker remains closed unless the
                 * driver is actively requesting a feed.
                 */
                blocker.closeBlocker();

                if (intakeRequested) {

                    intake.intakeOn();

                } else if (reverseRequested) {

                    intake.intakeReverse();

                } else {

                    intake.intakeOff();
                }
            }


            // =====================================
            // COMPETITION TELEMETRY
            // =====================================

            telemetry.addData(
                    "Shooter Mode",
                    shooter.getModeName()
            );

            telemetry.addLine();


            // -------------------------------------
            // LEFT / FRONT WHEEL
            // -------------------------------------

            telemetry.addData(
                    "Left RPM",
                    "%.0f",
                    leftRPM
            );

            telemetry.addData(
                    "Left TPS",
                    "%.0f",
                    leftTPS
            );

            telemetry.addData(
                    "Left Power",
                    "%.3f",
                    shooter.getLeftCommandedPower()
            );


            // -------------------------------------
            // BACK WHEEL
            // -------------------------------------

            telemetry.addData(
                    "Back RPM",
                    "%.0f",
                    backRPM
            );

            telemetry.addData(
                    "Back TPS",
                    "%.0f",
                    backTPS
            );

            telemetry.addData(
                    "Back Power",
                    "%.3f",
                    shooter.getBackCommandedPower()
            );

            telemetry.addLine();


            // -------------------------------------
            // CONFIGURED VALUES
            // -------------------------------------

            telemetry.addData(
                    "Configured Left Shoot",
                    "%.3f",
                    Shooter.LEFT_SHOOT_POWER
            );

            telemetry.addData(
                    "Configured Back Shoot",
                    "%.3f",
                    Shooter.BACK_SHOOT_POWER
            );

            telemetry.addData(
                    "Configured Left Idle",
                    "%.3f",
                    Shooter.LEFT_IDLE_POWER
            );

            telemetry.addData(
                    "Configured Back Idle",
                    "%.3f",
                    Shooter.BACK_IDLE_POWER
            );


            // -------------------------------------
            // FEED STATE
            // -------------------------------------

            if (feedRequested) {

                if (feedDelayComplete) {

                    telemetry.addData(
                            "Feed",
                            "FEEDING"
                    );

                } else {

                    telemetry.addData(
                            "Feed",
                            "SPINNING UP %.0f / %d ms",
                            feedRequestTimeMs,
                            FEED_SPINUP_TIME_MS
                    );
                }

            } else if (intakeRequested) {

                telemetry.addData(
                        "Feed",
                        "INTAKING"
                );

            } else if (reverseRequested) {

                telemetry.addData(
                        "Feed",
                        "REVERSING"
                );

            } else {

                telemetry.addData(
                        "Feed",
                        "CLOSED"
                );
            }


            // -------------------------------------
            // POWER LIMIT WARNING
            // -------------------------------------

            if (shooter.isShooting()
                    && shooter.getBackCommandedPower()
                    >= 0.999) {

                telemetry.addLine(
                        "BACK MOTOR AT MAXIMUM POWER"
                );
            }

            telemetry.update();
        }


        // =====================================
        // SAFE SHUTDOWN
        // =====================================

        safeShutdown();
    }


    // =====================================
    // SAFE SHUTDOWN
    // =====================================

    private void safeShutdown() {

        if (follower != null) {

            follower.manual(
                    new DrivePowers(
                            0,
                            0,
                            0
                    )
            );

            follower.update();
        }

        if (shooter != null) {
            shooter.shooterOff();
        }

        if (intake != null) {
            intake.intakeOff();
        }

        if (blocker != null) {
            blocker.closeBlocker();
        }
    }
}