package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {

    // =========================================================
    // SHOOTING TARGET RPM
    // =========================================================

    // Shooter wheel RPM, not motor RPM.
    public static double LEFT_TARGET_RPM = 2800.0;
    public static double RIGHT_TARGET_RPM = 5100.0;


    // =========================================================
    // IDLE TARGET RPM
    // =========================================================

    public static double LEFT_IDLE_RPM = 1000.0;
    public static double RIGHT_IDLE_RPM = 1800.0;


    // =========================================================
    // SHOOTING BASE POWER
    //
    // This is the approximate power required to maintain the
    // target RPM.
    //
    // P control adds/subtracts power from these values.
    // =========================================================

    public static double LEFT_BASE_POWER = 0.70;
    public static double RIGHT_BASE_POWER = 1.0;


    // =========================================================
    // IDLE BASE POWER
    // =========================================================

    public static double LEFT_IDLE_BASE_POWER = 0.25;
    public static double RIGHT_IDLE_BASE_POWER = 0.30;


    // =========================================================
    // SHOOTING P GAINS
    //
    // IMPORTANT:
    //
    // These gains operate on RPM error:
    //
    // error = targetRPM - measuredRPM
    //
    // power = basePower + (error * KP)
    //
    // These are NOT ticks-per-second gains.
    // =========================================================

    public static double LEFT_KP = 0.0006;
    public static double RIGHT_KP = 0.0009;


    // =========================================================
    // IDLE P GAINS
    // =========================================================

    public static double LEFT_IDLE_KP = 0.00010;
    public static double RIGHT_IDLE_KP = 0.00010;


    // =========================================================
    // ENCODER SETTINGS
    // =========================================================

    // Encoder ticks per MOTOR revolution.
    public static double TICKS_PER_REV = 28.0;


    // =========================================================
    // GEAR RATIOS
    //
    // Ratio = motor speed -> shooter wheel speed
    //
    // Left:
    // 10 tooth motor sprocket -> 14 tooth shooter sprocket
    //
    // wheelRPM = motorRPM * (10 / 14)
    // =========================================================

    public static double LEFT_GEAR_RATIO = 10.0 / 14.0;

    // Right/back shooter is direct drive.
    public static double RIGHT_GEAR_RATIO = 1.0;


    // =========================================================
    // READY SETTINGS
    // =========================================================

    // Absolute RPM tolerance around target.
    public static double LEFT_READY_TOLERANCE_RPM = 500.0;
    public static double RIGHT_READY_TOLERANCE_RPM = 500.0;

    // Both wheels must remain inside tolerance for this long.
    public static double READY_SETTLE_TIME_MS = 200.0;


    // =========================================================
    // MOTORS
    // =========================================================

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;


    // =========================================================
    // SHOOTER MODE
    // =========================================================

    private enum ShooterMode {
        OFF,
        IDLE,
        SHOOTING
    }

    private ShooterMode mode = ShooterMode.OFF;


    // =========================================================
    // CONTROL STATE
    // =========================================================

    private double activeLeftTargetRPM = 0.0;
    private double activeRightTargetRPM = 0.0;

    /*
     * These contain the RPM sampled during update().
     *
     * TeleOp can retrieve these without causing another hardware
     * read during the same bulk-cache cycle.
     */
    private double measuredLeftRPM = 0.0;
    private double measuredRightRPM = 0.0;

    private double leftErrorRPM = 0.0;
    private double rightErrorRPM = 0.0;

    private double leftCommandedPower = 0.0;
    private double rightCommandedPower = 0.0;

    /*
     * Time at which BOTH shooter wheels first entered their
     * ready tolerance.
     *
     * Zero means ready timing has not started.
     */
    private long readySinceNanos = 0L;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Shooter(HardwareMap hardwareMap) {

        // Must match FTC Robot Configuration names.
        leftMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );

        rightMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor1"
        );


        // =====================================================
        // MOTOR DIRECTIONS
        // =====================================================

        leftMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        rightMotor.setDirection(
                DcMotor.Direction.REVERSE
        );


        // =====================================================
        // ZERO POWER BEHAVIOUR
        // =====================================================

        /*
         * FLOAT allows the shooter wheels to coast instead of
         * actively braking when power is removed.
         */
        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );


        // =====================================================
        // ENCODER RESET
        // =====================================================

        leftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );


        // =====================================================
        // MOTOR RUN MODE
        // =====================================================

        /*
         * We are running our OWN external P controller.
         *
         * RUN_WITHOUT_ENCODER means setPower() directly controls
         * the motors rather than using the REV velocity PID.
         *
         * DcMotorEx.getVelocity() still gives encoder velocity.
         */
        leftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );


        // Start safely.
        shooterOff();
    }


    // =========================================================
    // UPDATE
    // =========================================================

    /**
     * Runs the shooter controller.
     *
     * IMPORTANT:
     *
     * Call this ONCE during every CompetitionTeleOp loop after
     * selecting the desired shooter mode.
     */
    public void update() {

        // -----------------------------------------------------
        // Read wheel speed once per control loop
        // -----------------------------------------------------

        measuredLeftRPM = readLeftRPM();
        measuredRightRPM = readRightRPM();


        // -----------------------------------------------------
        // OFF
        // -----------------------------------------------------

        if (mode == ShooterMode.OFF) {

            activeLeftTargetRPM = 0.0;
            activeRightTargetRPM = 0.0;

            leftErrorRPM = 0.0;
            rightErrorRPM = 0.0;

            leftCommandedPower = 0.0;
            rightCommandedPower = 0.0;

            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

            resetReadyTimer();

            return;
        }


        // -----------------------------------------------------
        // Select mode parameters
        // -----------------------------------------------------

        double leftBasePower;
        double rightBasePower;

        double selectedLeftKP;
        double selectedRightKP;


        if (mode == ShooterMode.SHOOTING) {

            activeLeftTargetRPM = LEFT_TARGET_RPM;
            activeRightTargetRPM = RIGHT_TARGET_RPM;

            leftBasePower = LEFT_BASE_POWER;
            rightBasePower = RIGHT_BASE_POWER;

            selectedLeftKP = LEFT_KP;
            selectedRightKP = RIGHT_KP;

        } else {

            // IDLE

            activeLeftTargetRPM = LEFT_IDLE_RPM;
            activeRightTargetRPM = RIGHT_IDLE_RPM;

            leftBasePower = LEFT_IDLE_BASE_POWER;
            rightBasePower = RIGHT_IDLE_BASE_POWER;

            selectedLeftKP = LEFT_IDLE_KP;
            selectedRightKP = RIGHT_IDLE_KP;
        }


        // -----------------------------------------------------
        // Calculate RPM errors
        // -----------------------------------------------------

        leftErrorRPM =
                activeLeftTargetRPM
                        - measuredLeftRPM;

        rightErrorRPM =
                activeRightTargetRPM
                        - measuredRightRPM;


        // -----------------------------------------------------
        // Calculate motor powers
        // -----------------------------------------------------

        leftCommandedPower = calculatePower(
                activeLeftTargetRPM,
                measuredLeftRPM,
                leftBasePower,
                selectedLeftKP
        );

        rightCommandedPower = calculatePower(
                activeRightTargetRPM,
                measuredRightRPM,
                rightBasePower,
                selectedRightKP
        );


        // -----------------------------------------------------
        // Apply power
        // -----------------------------------------------------

        /*
         * No slew-rate limiter.
         *
         * If a ball hits the wheels and RPM drops, we want the
         * P controller to be able to command full power on the
         * next control loop.
         */
        leftMotor.setPower(
                leftCommandedPower
        );

        rightMotor.setPower(
                rightCommandedPower
        );
    }


    // =========================================================
    // P CONTROLLER
    // =========================================================

    /**
     * Calculates motor power using:
     *
     * errorRPM = targetRPM - measuredRPM
     *
     * correction = errorRPM * KP
     *
     * power = basePower + correction
     */
    private double calculatePower(
            double targetRPM,
            double measuredRPM,
            double basePower,
            double kP
    ) {

        if (targetRPM <= 0.0) {
            return 0.0;
        }

        double errorRPM =
                targetRPM - measuredRPM;

        double proportionalCorrection =
                errorRPM * kP;

        return clipPower(
                basePower
                        + proportionalCorrection
        );
    }


    // =========================================================
    // SHOOTER ON
    // =========================================================

    public void shooterOn() {

        /*
         * Only reset readiness when transitioning INTO shooting.
         *
         * Your TeleOp calls shooterOn() repeatedly while the
         * trigger is held, so we MUST NOT reset every call.
         */
        if (mode != ShooterMode.SHOOTING) {
            resetReadyTimer();
        }

        mode = ShooterMode.SHOOTING;
    }


    // =========================================================
    // SHOOTER IDLE
    // =========================================================

    public void shooterIdle() {

        if (mode != ShooterMode.IDLE) {
            resetReadyTimer();
        }

        mode = ShooterMode.IDLE;
    }


    // =========================================================
    // SHOOTER OFF
    // =========================================================

    public void shooterOff() {

        mode = ShooterMode.OFF;

        activeLeftTargetRPM = 0.0;
        activeRightTargetRPM = 0.0;

        leftErrorRPM = 0.0;
        rightErrorRPM = 0.0;

        leftCommandedPower = 0.0;
        rightCommandedPower = 0.0;

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        resetReadyTimer();
    }


    // =========================================================
    // RPM
    // =========================================================

    /**
     * Returns the RPM sampled during the most recent update().
     *
     * It deliberately does NOT perform another hardware read.
     */
    public double getLeftRPM() {
        return measuredLeftRPM;
    }


    /**
     * Returns the RPM sampled during the most recent update().
     */
    public double getRightRPM() {
        return measuredRightRPM;
    }


    // =========================================================
    // INTERNAL RPM READS
    // =========================================================

    private double readLeftRPM() {

        double ticksPerSecond =
                Math.abs(
                        leftMotor.getVelocity()
                );

        return ticksPerSecondToWheelRPM(
                ticksPerSecond,
                LEFT_GEAR_RATIO
        );
    }


    private double readRightRPM() {

        double ticksPerSecond =
                Math.abs(
                        rightMotor.getVelocity()
                );

        return ticksPerSecondToWheelRPM(
                ticksPerSecond,
                RIGHT_GEAR_RATIO
        );
    }


    // =========================================================
    // RAW ENCODER VELOCITY
    // =========================================================

    public double getLeftTicksPerSecond() {

        return Math.abs(
                leftMotor.getVelocity()
        );
    }


    public double getRightTicksPerSecond() {

        return Math.abs(
                rightMotor.getVelocity()
        );
    }


    // =========================================================
    // INDIVIDUAL READY CHECKS
    // =========================================================

    public boolean isLeftReady(
            double leftRPM
    ) {

        if (mode != ShooterMode.SHOOTING) {
            return false;
        }

        double rpmError =
                Math.abs(
                        LEFT_TARGET_RPM
                                - leftRPM
                );

        return rpmError
                <= LEFT_READY_TOLERANCE_RPM;
    }


    public boolean isRightReady(
            double rightRPM
    ) {

        if (mode != ShooterMode.SHOOTING) {
            return false;
        }

        double rpmError =
                Math.abs(
                        RIGHT_TARGET_RPM
                                - rightRPM
                );

        return rpmError
                <= RIGHT_READY_TOLERANCE_RPM;
    }


    // =========================================================
    // SHOOTER READY
    // =========================================================

    /**
     * Convenience version.
     *
     * Uses RPM values captured during the latest update().
     */
    public boolean shooterReady() {

        return shooterReady(
                measuredLeftRPM,
                measuredRightRPM
        );
    }


    /**
     * Competition TeleOp version.
     *
     * Both wheels must remain within tolerance continuously for
     * READY_SETTLE_TIME_MS before this returns true.
     */
    public boolean shooterReady(
            double leftRPM,
            double rightRPM
    ) {

        if (mode != ShooterMode.SHOOTING) {

            resetReadyTimer();
            return false;
        }

        boolean leftReady =
                isLeftReady(leftRPM);

        boolean rightReady =
                isRightReady(rightRPM);


        // -----------------------------------------------------
        // Either wheel dropped out of tolerance
        // -----------------------------------------------------

        if (!leftReady || !rightReady) {

            resetReadyTimer();
            return false;
        }


        // -----------------------------------------------------
        // Both wheels have just become ready
        // -----------------------------------------------------

        if (readySinceNanos == 0L) {

            readySinceNanos =
                    System.nanoTime();

            return false;
        }


        // -----------------------------------------------------
        // Both wheels remain ready
        // -----------------------------------------------------

        double stableTimeMs =
                (System.nanoTime()
                        - readySinceNanos)
                        / 1_000_000.0;

        return stableTimeMs
                >= READY_SETTLE_TIME_MS;
    }


    // =========================================================
    // RESET READY TIMER
    // =========================================================

    private void resetReadyTimer() {
        readySinceNanos = 0L;
    }


    // =========================================================
    // MODE STATE
    // =========================================================

    public boolean isShooting() {
        return mode
                == ShooterMode.SHOOTING;
    }


    public boolean isIdle() {
        return mode
                == ShooterMode.IDLE;
    }


    public boolean isOff() {
        return mode
                == ShooterMode.OFF;
    }


    public String getModeName() {
        return mode.name();
    }


    // =========================================================
    // TARGET RPM GETTERS
    // =========================================================

    public double getLeftTargetRPM() {
        return activeLeftTargetRPM;
    }


    public double getRightTargetRPM() {
        return activeRightTargetRPM;
    }


    // =========================================================
    // ERROR GETTERS
    // =========================================================

    public double getLeftErrorRPM() {
        return leftErrorRPM;
    }


    public double getRightErrorRPM() {
        return rightErrorRPM;
    }


    // =========================================================
    // POWER GETTERS
    // =========================================================

    public double getLeftCommandedPower() {
        return leftCommandedPower;
    }


    public double getRightCommandedPower() {
        return rightCommandedPower;
    }


    // =========================================================
    // MOTOR RPM GETTERS
    //
    // Useful while checking gearing and encoder calculations.
    // =========================================================

    public double getLeftMotorRPM() {

        return Math.abs(
                leftMotor.getVelocity()
        )
                * 60.0
                / TICKS_PER_REV;
    }


    public double getRightMotorRPM() {

        return Math.abs(
                rightMotor.getVelocity()
        )
                * 60.0
                / TICKS_PER_REV;
    }


    // =========================================================
    // TICKS PER SECOND -> SHOOTER WHEEL RPM
    // =========================================================

    private double ticksPerSecondToWheelRPM(
            double ticksPerSecond,
            double gearRatio
    ) {

        if (TICKS_PER_REV <= 0.0
                || gearRatio <= 0.0) {

            return 0.0;
        }

        double motorRPM =
                ticksPerSecond
                        * 60.0
                        / TICKS_PER_REV;

        return motorRPM
                * gearRatio;
    }


    // =========================================================
    // POWER CLIPPING
    // =========================================================

    private double clipPower(
            double power
    ) {

        return Math.max(
                0.0,
                Math.min(
                        1.0,
                        power
                )
        );
    }
}