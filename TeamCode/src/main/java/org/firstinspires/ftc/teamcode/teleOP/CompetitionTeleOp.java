package org.firstinspires.ftc.teamcode.teleOP;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
@TeleOp(
        name = "1. Competition TeleOp",
        group = "TeleOp"
)
public class CompetitionTeleOp extends LinearOpMode {

    // =========================================================
    // LIVE-TUNABLE FEED SETTINGS
    // =========================================================

    public static long FEED_PULSE_TIME_MS = 10;
    public static long FEED_RECOVERY_TIME_MS = 0;


    // =========================================================
    // SUBSYSTEMS
    // =========================================================

    private Follower follower;

    private Shooter shooter;
    private Blocker blocker;
    private Intake intake;

    private List<LynxModule> hubs;


    // =========================================================
    // FEED STATE MACHINE
    // =========================================================

    private enum FeedState {
        IDLE,
        WAITING_FOR_READY,
        FEEDING,
        RECOVERING
    }

    private FeedState feedState = FeedState.IDLE;
    private long feedStateStartNanos = 0L;


    // =========================================================
    // RUN OPMODE
    // =========================================================

    @Override
    public void runOpMode() {

        try {

            // -------------------------------------------------
            // Initialise robot
            // -------------------------------------------------

            follower = Constants.create(hardwareMap);

            shooter = new Shooter(hardwareMap);
            blocker = new Blocker(hardwareMap);
            intake = new Intake(hardwareMap);


            // -------------------------------------------------
            // Manual bulk caching
            // -------------------------------------------------

            hubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(
                        LynxModule.BulkCachingMode.MANUAL
                );
            }


            // -------------------------------------------------
            // Safe initial state
            // -------------------------------------------------

            shooter.shooterOff();
            intake.intakeOff();
            blocker.closeBlocker();

            resetFeedSequence();


            // -------------------------------------------------
            // Initial telemetry
            // -------------------------------------------------

            telemetry.addLine("ROBOT READY");
            telemetry.addLine("Cross/A: Shooter ON");
            telemetry.addLine("Circle/B: Shooter IDLE");
            telemetry.addLine("Right trigger: Automatic pulse feed");
            telemetry.addLine("Right bumper: Intake");
            telemetry.addLine("Left bumper: Reverse intake");

            telemetry.addData(
                    "Feed pulse",
                    "%d ms",
                    FEED_PULSE_TIME_MS
            );

            telemetry.addData(
                    "Feed recovery",
                    "%d ms",
                    FEED_RECOVERY_TIME_MS
            );

            telemetry.addData(
                    "Shooting target",
                    "%.0f RPM",
                    Shooter.TARGET_RPM
            );

            telemetry.addData(
                    "Idle target",
                    "%.0f RPM",
                    Shooter.IDLE_RPM
            );

            telemetry.addData(
                    "Shooting Base / KP",
                    "%.3f / %.6f",
                    Shooter.BASE_POWER,
                    Shooter.KP
            );

            telemetry.addData(
                    "Idle Base / KP",
                    "%.3f / %.6f",
                    Shooter.IDLE_BASE_POWER,
                    Shooter.IDLE_KP
            );

            telemetry.addData(
                    "Ready tolerance",
                    "+/- %.0f RPM",
                    Shooter.READY_TOLERANCE_RPM
            );

            telemetry.addData(
                    "Ready settle time",
                    "%.0f ms",
                    Shooter.READY_SETTLE_TIME_MS
            );

            telemetry.update();


            // -------------------------------------------------
            // Wait for start
            // -------------------------------------------------

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            // Start at idle RPM.
            shooter.shooterIdle();


            // =================================================
            // MAIN LOOP
            // =================================================

            while (opModeIsActive()) {

                // ---------------------------------------------
                // Clear bulk cache
                // ---------------------------------------------

                for (LynxModule hub : hubs) {
                    hub.clearBulkCache();
                }


                // ---------------------------------------------
                // Driver inputs
                // ---------------------------------------------

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


                // ---------------------------------------------
                // Drive control
                // ---------------------------------------------

                double forward =
                        -gamepad1.left_stick_y;

                double strafe =
                        -Math.pow(gamepad1.left_stick_x, 3);

                double turn =
                        -Math.pow(gamepad1.right_stick_x, 3);

                follower.manual(
                        new DrivePowers(
                                forward,
                                strafe,
                                turn
                        )
                );

                follower.update();


                // ---------------------------------------------
                // Shooter mode control
                // ---------------------------------------------
                //
                // Priority:
                // 1. Feed request
                // 2. Manual shooter command
                // 3. Intake/reverse
                // 4. Manual idle command
                //
                // If no command is entered, the current shooter
                // mode remains latched.
                // ---------------------------------------------

                if (feedRequested) {

                    shooter.shooterOn();

                } else if (shooterRequested) {

                    shooter.shooterOn();

                } else if (intakeRequested || reverseRequested) {

                    shooter.shooterIdle();

                } else if (idleRequested) {

                    shooter.shooterIdle();
                }


                // ---------------------------------------------
                // Run shooter controller exactly once
                // ---------------------------------------------

                shooter.update();


                // ---------------------------------------------
                // Read values stored by Shooter.update()
                // ---------------------------------------------

                double leftRPM =
                        shooter.getLeftRPM();

                double rightRPM =
                        shooter.getRightRPM();

                boolean leftReady =
                        shooter.isLeftReady(leftRPM);

                boolean rightReady =
                        shooter.isRightReady(rightRPM);

                /*
                 * This method also manages READY_SETTLE_TIME_MS.
                 * It should only be called once per active loop.
                 */
                boolean shooterReady =
                        shooter.shooterReady(
                                leftRPM,
                                rightRPM
                        );


                // ---------------------------------------------
                // Update feed controller
                // ---------------------------------------------

                updateFeedState(
                        feedRequested,
                        shooterReady
                );

                applyFeedAndIntakeOutputs(
                        feedRequested,
                        intakeRequested,
                        reverseRequested
                );


                // ---------------------------------------------
                // Telemetry
                // ---------------------------------------------

                telemetry.addData(
                        "Shooter Mode",
                        shooter.getModeName()
                );

                telemetry.addData(
                        "Shooter Ready",
                        shooterReady ? "YES" : "NO"
                );

                telemetry.addData(
                        "Feed State",
                        feedState.name()
                );

                telemetry.addData(
                        "Feed Status",
                        getFeedStatus(
                                feedRequested,
                                intakeRequested,
                                reverseRequested
                        )
                );

                telemetry.addData(
                        "Feed State Timer",
                        "%.0f ms",
                        getFeedStateElapsedMs()
                );

                telemetry.addLine();


                // ---------------------------------------------
                // Left shooter
                // ---------------------------------------------

                telemetry.addData(
                        "Left RPM",
                        "%.0f / %.0f",
                        leftRPM,
                        shooter.getLeftTargetRPM()
                );

                telemetry.addData(
                        "Left Error",
                        "%.0f RPM",
                        shooter.getLeftErrorRPM()
                );

                telemetry.addData(
                        "Left Power",
                        "%.3f",
                        shooter.getLeftCommandedPower()
                );

                telemetry.addData(
                        "Left Ready",
                        leftReady ? "YES" : "NO"
                );

                telemetry.addData(
                        "Left Encoder",
                        "%.0f ticks/s",
                        shooter.getLeftTicksPerSecond()
                );

                telemetry.addLine();


                // ---------------------------------------------
                // Right shooter
                // ---------------------------------------------

                telemetry.addData(
                        "Right RPM",
                        "%.0f / %.0f",
                        rightRPM,
                        shooter.getRightTargetRPM()
                );

                telemetry.addData(
                        "Right Error",
                        "%.0f RPM",
                        shooter.getRightErrorRPM()
                );

                telemetry.addData(
                        "Right Power",
                        "%.3f",
                        shooter.getRightCommandedPower()
                );

                telemetry.addData(
                        "Right Ready",
                        rightReady ? "YES" : "NO"
                );

                telemetry.addData(
                        "Right Encoder",
                        "%.0f ticks/s",
                        shooter.getRightTicksPerSecond()
                );

                telemetry.addLine();


                // ---------------------------------------------
                // Live feed settings
                // ---------------------------------------------

                telemetry.addData(
                        "Feed Pulse Setting",
                        "%d ms",
                        FEED_PULSE_TIME_MS
                );

                telemetry.addData(
                        "Feed Recovery Setting",
                        "%d ms",
                        FEED_RECOVERY_TIME_MS
                );


                // ---------------------------------------------
                // Shooter live-tuning settings
                // ---------------------------------------------

                telemetry.addData(
                        "Shooting Target",
                        "%.0f RPM",
                        Shooter.TARGET_RPM
                );

                telemetry.addData(
                        "Shooting Base / KP",
                        "%.3f / %.6f",
                        Shooter.BASE_POWER,
                        Shooter.KP
                );

                telemetry.addData(
                        "Idle Target",
                        "%.0f RPM",
                        Shooter.IDLE_RPM
                );

                telemetry.addData(
                        "Idle Base / KP",
                        "%.3f / %.6f",
                        Shooter.IDLE_BASE_POWER,
                        Shooter.IDLE_KP
                );

                telemetry.addData(
                        "Ready Tolerance",
                        "+/- %.0f RPM",
                        Shooter.READY_TOLERANCE_RPM
                );

                telemetry.addData(
                        "Ready Settle",
                        "%.0f ms",
                        Shooter.READY_SETTLE_TIME_MS
                );

                telemetry.addData(
                        "Ticks / Rev",
                        "%.1f",
                        Shooter.TICKS_PER_REV
                );

                telemetry.addData(
                        "Gear Ratio",
                        "%.4f",
                        Shooter.GEAR_RATIO
                );


                // ---------------------------------------------
                // Power-limit warnings
                // ---------------------------------------------

                if (shooter.isShooting()
                        && shooter.getLeftCommandedPower() >= 0.999) {

                    telemetry.addLine(
                            "WARNING: LEFT SHOOTER AT MAX POWER"
                    );
                }

                if (shooter.isShooting()
                        && shooter.getRightCommandedPower() >= 0.999) {

                    telemetry.addLine(
                            "WARNING: RIGHT SHOOTER AT MAX POWER"
                    );
                }

                telemetry.update();
            }

        } finally {

            safeShutdown();
        }
    }


    // =========================================================
    // UPDATE FEED STATE
    // =========================================================

    private void updateFeedState(
            boolean feedRequested,
            boolean shooterReady
    ) {

        /*
         * Releasing the trigger immediately cancels the sequence.
         */
        if (!feedRequested) {
            resetFeedSequence();
            return;
        }

        switch (feedState) {

            case IDLE:

                setFeedState(FeedState.WAITING_FOR_READY);
                break;


            case WAITING_FOR_READY:

                /*
                 * shooterReady already includes the configured
                 * READY_SETTLE_TIME_MS requirement.
                 */
                if (shooterReady) {
                    setFeedState(FeedState.FEEDING);
                }

                break;


            case FEEDING:

                /*
                 * Complete the feed pulse even if shooter RPM
                 * falls as the ball reaches the wheels.
                 */
                if (getFeedStateElapsedMs()
                        >= getSafeFeedPulseTimeMs()) {

                    setFeedState(FeedState.RECOVERING);
                }

                break;


            case RECOVERING:

                if (getFeedStateElapsedMs()
                        >= getSafeRecoveryTimeMs()) {

                    setFeedState(FeedState.WAITING_FOR_READY);
                }

                break;


            default:

                resetFeedSequence();
                break;
        }
    }


    // =========================================================
    // APPLY FEEDER AND INTAKE OUTPUTS
    // =========================================================

    private void applyFeedAndIntakeOutputs(
            boolean feedRequested,
            boolean intakeRequested,
            boolean reverseRequested
    ) {

        /*
         * Feeding has priority over normal intake control.
         */
        if (feedRequested) {

            if (feedState == FeedState.FEEDING) {

                blocker.openBlocker();
                intake.feedOn();

            } else {

                blocker.closeBlocker();
                intake.intakeOff();
            }

            return;
        }


        blocker.closeBlocker();

        if (intakeRequested) {

            intake.intakeOn();

        } else if (reverseRequested) {

            intake.intakeReverse();

        } else {

            intake.intakeOff();
        }
    }


    // =========================================================
    // FEED STATE HELPERS
    // =========================================================

    private void setFeedState(FeedState newState) {

        if (feedState == newState) {
            return;
        }

        feedState = newState;
        feedStateStartNanos = System.nanoTime();
    }


    private void resetFeedSequence() {

        feedState = FeedState.IDLE;
        feedStateStartNanos = 0L;
    }


    private double getFeedStateElapsedMs() {

        if (feedStateStartNanos == 0L) {
            return 0.0;
        }

        return (System.nanoTime() - feedStateStartNanos)
                / 1_000_000.0;
    }


    private long getSafeFeedPulseTimeMs() {

        return Math.max(
                1L,
                FEED_PULSE_TIME_MS
        );
    }


    private long getSafeRecoveryTimeMs() {

        return Math.max(
                0L,
                FEED_RECOVERY_TIME_MS
        );
    }


    // =========================================================
    // FEED STATUS
    // =========================================================

    private String getFeedStatus(
            boolean feedRequested,
            boolean intakeRequested,
            boolean reverseRequested
    ) {

        if (feedRequested) {

            switch (feedState) {

                case IDLE:
                    return "STARTING";

                case WAITING_FOR_READY:
                    return "WAITING FOR RPM";

                case FEEDING:
                    return "FEEDING ONE BALL";

                case RECOVERING:
                    return "SHOOTER RECOVERY";

                default:
                    return "UNKNOWN";
            }
        }

        if (intakeRequested) {
            return "INTAKING";
        }

        if (reverseRequested) {
            return "REVERSING";
        }

        return "CLOSED";
    }


    // =========================================================
    // SAFE SHUTDOWN
    // =========================================================

    private void safeShutdown() {

        resetFeedSequence();

        if (intake != null) {
            intake.intakeOff();
        }

        if (blocker != null) {
            blocker.closeBlocker();
        }

        if (shooter != null) {
            shooter.shooterOff();
        }

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
    }
}