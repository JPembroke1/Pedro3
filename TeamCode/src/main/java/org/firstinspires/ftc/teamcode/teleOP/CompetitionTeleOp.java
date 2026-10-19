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

    /*
     * How long the feeder runs for each ball.
     *
     * Increase if a ball does not completely enter the shooter.
     * Decrease if more than one ball is being fed per pulse.
     */
    public static long FEED_PULSE_TIME_MS = 10;

    /*
     * Minimum pause after each feed pulse.
     *
     * The shooter must also become RPM-ready again before the
     * next feed pulse begins.
     */
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

            // =================================================
            // INITIALISE ROBOT
            // =================================================

            follower = Constants.create(hardwareMap);

            shooter = new Shooter(hardwareMap);
            blocker = new Blocker(hardwareMap);
            intake = new Intake(hardwareMap);


            // =================================================
            // MANUAL BULK CACHING
            // =================================================

            hubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(
                        LynxModule.BulkCachingMode.MANUAL
                );
            }


            // =================================================
            // SAFE INITIAL STATE
            // =================================================

            shooter.shooterOff();
            intake.intakeOff();
            blocker.closeBlocker();

            resetFeedSequence();


            // =================================================
            // INITIAL TELEMETRY
            // =================================================

            telemetry.addLine("ROBOT READY");
            telemetry.addLine("Cross/A: Shooter ON");
            telemetry.addLine("Circle/B: Shooter IDLE");
            telemetry.addLine("Right trigger: Pulse feed");
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
                    "Left target",
                    "%.0f RPM",
                    Shooter.LEFT_TARGET_RPM
            );

            telemetry.addData(
                    "Right target",
                    "%.0f RPM",
                    Shooter.RIGHT_TARGET_RPM
            );

            telemetry.addData(
                    "Left base / KP",
                    "%.3f / %.6f",
                    Shooter.LEFT_BASE_POWER,
                    Shooter.LEFT_KP
            );

            telemetry.addData(
                    "Right base / KP",
                    "%.3f / %.6f",
                    Shooter.RIGHT_BASE_POWER,
                    Shooter.RIGHT_KP
            );

            telemetry.update();


            // =================================================
            // WAIT FOR START
            // =================================================

            waitForStart();

            if (isStopRequested()) {
                return;
            }


            // Start with the shooter running at idle speed.
            shooter.shooterIdle();


            // =================================================
            // MAIN LOOP
            // =================================================

            while (opModeIsActive()) {

                // =============================================
                // CLEAR BULK CACHE
                // =============================================

                for (LynxModule hub : hubs) {
                    hub.clearBulkCache();
                }


                // =============================================
                // DRIVER INPUTS
                // =============================================

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


                // =============================================
                // DRIVE CONTROL
                // =============================================

                double forward =
                        -gamepad1.left_stick_y;

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


                // =============================================
                // SHOOTER MODE CONTROL
                // =============================================

                /*
                 * Priority:
                 *
                 * 1. Feed trigger
                 * 2. Manual shooter ON
                 * 3. Intake or reverse
                 * 4. Manual shooter IDLE
                 *
                 * Shooter mode remains latched if no new command
                 * is entered.
                 */
                if (feedRequested) {

                    shooter.shooterOn();

                } else if (shooterRequested) {

                    shooter.shooterOn();

                } else if (intakeRequested
                        || reverseRequested) {

                    shooter.shooterIdle();

                } else if (idleRequested) {

                    shooter.shooterIdle();
                }


                // =============================================
                // RUN SHOOTER P CONTROLLER
                // =============================================

                /*
                 * Must be called exactly once per active loop.
                 *
                 * This reads RPM, calculates P correction, and
                 * applies motor power.
                 */
                shooter.update();


                // =============================================
                // READ STORED SHOOTER VALUES
                // =============================================

                double leftRPM =
                        shooter.getLeftRPM();

                double rightRPM =
                        shooter.getRightRPM();

                boolean leftReady =
                        shooter.isLeftReady(leftRPM);

                boolean rightReady =
                        shooter.isRightReady(rightRPM);

                boolean shooterReady =
                        shooter.shooterReady(
                                leftRPM,
                                rightRPM
                        );


                // =============================================
                // UPDATE FEED STATE MACHINE
                // =============================================

                updateFeedState(
                        feedRequested,
                        shooterReady
                );


                // =============================================
                // APPLY FEEDER / INTAKE OUTPUTS
                // =============================================

                applyFeedAndIntakeOutputs(
                        feedRequested,
                        intakeRequested,
                        reverseRequested
                );


                // =============================================
                // TELEMETRY
                // =============================================

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
                // LEFT SHOOTER
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

                telemetry.addLine();


                // ---------------------------------------------
                // RIGHT / BACK SHOOTER
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

                telemetry.addLine();


                // ---------------------------------------------
                // LIVE FEED SETTINGS
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
                // SHOOTER TUNING SETTINGS
                // ---------------------------------------------

                telemetry.addData(
                        "Left Base / KP",
                        "%.3f / %.6f",
                        Shooter.LEFT_BASE_POWER,
                        Shooter.LEFT_KP
                );

                telemetry.addData(
                        "Right Base / KP",
                        "%.3f / %.6f",
                        Shooter.RIGHT_BASE_POWER,
                        Shooter.RIGHT_KP
                );


                // ---------------------------------------------
                // POWER LIMIT WARNINGS
                // ---------------------------------------------

                if (shooter.isShooting()
                        && shooter.getLeftCommandedPower()
                        >= 0.999) {

                    telemetry.addLine(
                            "LEFT MOTOR AT MAXIMUM POWER"
                    );
                }

                if (shooter.isShooting()
                        && shooter.getRightCommandedPower()
                        >= 0.999) {

                    telemetry.addLine(
                            "RIGHT MOTOR AT MAXIMUM POWER"
                    );
                }

                telemetry.update();
            }

        } finally {

            // Always place the robot into a safe state.
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
         * Releasing the trigger immediately cancels the feed
         * sequence and returns it to IDLE.
         */
        if (!feedRequested) {
            resetFeedSequence();
            return;
        }

        switch (feedState) {

            // -------------------------------------------------
            // NEW FEED REQUEST
            // -------------------------------------------------

            case IDLE:

                setFeedState(
                        FeedState.WAITING_FOR_READY
                );

                break;


            // -------------------------------------------------
            // WAIT FOR SHOOTER RPM
            // -------------------------------------------------

            case WAITING_FOR_READY:

                /*
                 * A new pulse starts only after both wheels have
                 * remained inside their RPM tolerance for the
                 * Shooter READY_SETTLE_TIME_MS.
                 */
                if (shooterReady) {

                    setFeedState(
                            FeedState.FEEDING
                    );
                }

                break;


            // -------------------------------------------------
            // FEED ONE BALL
            // -------------------------------------------------

            case FEEDING:

                /*
                 * Once a feed pulse starts, allow the complete
                 * pulse to run even if RPM drops.
                 *
                 * This prevents leaving a ball partially engaged
                 * with the shooter wheels.
                 */
                if (getFeedStateElapsedMs()
                        >= getSafeFeedPulseTimeMs()) {

                    setFeedState(
                            FeedState.RECOVERING
                    );
                }

                break;


            // -------------------------------------------------
            // SHOOTER RECOVERY
            // -------------------------------------------------

            case RECOVERING:

                /*
                 * Stay in recovery for at least the configured
                 * recovery time.
                 *
                 * After that, return to WAITING_FOR_READY.
                 * The next pulse still requires shooterReady().
                 */
                if (getFeedStateElapsedMs()
                        >= getSafeRecoveryTimeMs()) {

                    setFeedState(
                            FeedState.WAITING_FOR_READY
                    );
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
         * An active feed request overrides normal intake control.
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


        /*
         * Normal intake operation when the feed trigger is not
         * being held.
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


    // =========================================================
    // SET FEED STATE
    // =========================================================

    private void setFeedState(
            FeedState newState
    ) {

        if (feedState == newState) {
            return;
        }

        feedState = newState;

        feedStateStartNanos =
                System.nanoTime();
    }


    // =========================================================
    // RESET FEED SEQUENCE
    // =========================================================

    private void resetFeedSequence() {

        feedState = FeedState.IDLE;

        feedStateStartNanos = 0L;
    }


    // =========================================================
    // FEED STATE TIMER
    // =========================================================

    private double getFeedStateElapsedMs() {

        if (feedStateStartNanos == 0L) {
            return 0.0;
        }

        return (System.nanoTime()
                - feedStateStartNanos)
                / 1_000_000.0;
    }


    // =========================================================
    // SAFE LIVE TIMING VALUES
    // =========================================================

    private long getSafeFeedPulseTimeMs() {

        /*
         * Prevent a zero or negative pulse from causing rapid
         * state switching.
         */
        return Math.max(
                1L,
                FEED_PULSE_TIME_MS
        );
    }


    private long getSafeRecoveryTimeMs() {

        /*
         * Zero recovery is allowed, but negative values are not.
         */
        return Math.max(
                0L,
                FEED_RECOVERY_TIME_MS
        );
    }


    // =========================================================
    // FEED STATUS TEXT
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


        /*
         * Stop the feeder before shutting down the shooter.
         */
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