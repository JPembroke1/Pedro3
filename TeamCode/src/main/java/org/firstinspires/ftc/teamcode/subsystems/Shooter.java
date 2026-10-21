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
    public static double TARGET_RPM = 2800.0;


    // =========================================================
    // IDLE TARGET RPM
    // =========================================================

    public static double IDLE_RPM = 1000.0;


    // =========================================================
    // SHOOTING BASE POWER
    //
    // This is the approximate power required to maintain the
    // target RPM.
    //
    // P control adds or subtracts power from this value.
    // =========================================================

    public static double BASE_POWER = 0.70;


    // =========================================================
    // IDLE BASE POWER
    // =========================================================

    public static double IDLE_BASE_POWER = 0.25;


    // =========================================================
    // SHOOTING P GAIN
    //
    // error = targetRPM - measuredRPM
    //
    // power = basePower + (error * KP)
    //
    // This gain operates on RPM error, not ticks per second.
    // =========================================================

    public static double KP = 0.0006;


    // =========================================================
    // IDLE P GAIN
    // =========================================================

    public static double IDLE_KP = 0.00010;


    // =========================================================
    // ENCODER SETTINGS
    // =========================================================

    // Encoder ticks per motor revolution.
    public static double TICKS_PER_REV = 28.0;


    // =========================================================
    // GEAR RATIO
    //
    // Ratio converts motor RPM to shooter-wheel RPM.
    //
    // wheelRPM = motorRPM * GEAR_RATIO
    //
    // Direct drive:
    // GEAR_RATIO = 1.0
    //
    // 10-tooth driver to 14-tooth driven:
    // GEAR_RATIO = 10.0 / 14.0
    // =========================================================

    public static double GEAR_RATIO = 1.0;


    // =========================================================
    // READY SETTINGS
    // =========================================================

    // Absolute RPM tolerance around TARGET_RPM.
    public static double READY_TOLERANCE_RPM = 500.0;

    // Both motors must remain ready for this duration.
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
     * These contain the RPM values sampled during update().
     *
     * TeleOp can retrieve these without causing additional
     * hardware reads during the same bulk-cache cycle.
     */
    private double measuredLeftRPM = 0.0;
    private double measuredRightRPM = 0.0;

    private double leftErrorRPM = 0.0;
    private double rightErrorRPM = 0.0;

    private double leftCommandedPower = 0.0;
    private double rightCommandedPower = 0.0;

    /*
     * Time at which both shooter wheels first entered their ready
     * tolerance.
     *
     * Zero means ready timing has not started.
     */
    private long readySinceNanos = 0L;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Shooter(HardwareMap hardwareMap) {

        // Must match the FTC Robot Configuration names.
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
         * The subsystem runs its own external P controller.
         *
         * RUN_WITHOUT_ENCODER means setPower() directly controls
         * motor power rather than using the REV velocity PID.
         *
         * DcMotorEx.getVelocity() still provides encoder velocity.
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
     * Call this once during every CompetitionTeleOp loop after
     * selecting the desired shooter mode.
     */
    public void update() {

        // -----------------------------------------------------
        // Read wheel speeds once per control loop
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

        double selectedTargetRPM;
        double selectedBasePower;
        double selectedKP;

        if (mode == ShooterMode.SHOOTING) {

            selectedTargetRPM = TARGET_RPM;
            selectedBasePower = BASE_POWER;
            selectedKP = KP;

        } else {

            // IDLE
            selectedTargetRPM = IDLE_RPM;
            selectedBasePower = IDLE_BASE_POWER;
            selectedKP = IDLE_KP;
        }

        activeLeftTargetRPM = selectedTargetRPM;
        activeRightTargetRPM = selectedTargetRPM;


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
                selectedBasePower,
                selectedKP
        );

        rightCommandedPower = calculatePower(
                activeRightTargetRPM,
                measuredRightRPM,
                selectedBasePower,
                selectedKP
        );


        // -----------------------------------------------------
        // Apply power
        // -----------------------------------------------------

        /*
         * There is no slew-rate limiter.
         *
         * If a ball causes an RPM drop, the P controller can
         * command the required recovery power immediately.
         */
        leftMotor.setPower(leftCommandedPower);
        rightMotor.setPower(rightCommandedPower);
    }


    // =========================================================
    // P CONTROLLER
    // =========================================================

    /**
     * Calculates motor power using:
     *
     * errorRPM = targetRPM - measuredRPM
     *
     * correction = errorRPM * kP
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
                targetRPM
                        - measuredRPM;

        double proportionalCorrection =
                errorRPM
                        * kP;

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
         * Only reset readiness when transitioning into SHOOTING.
         *
         * TeleOp may call shooterOn() repeatedly while a trigger
         * is held, so the timer must not reset on every call.
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

        measuredLeftRPM = 0.0;
        measuredRightRPM = 0.0;

        leftErrorRPM = 0.0;
        rightErrorRPM = 0.0;

        leftCommandedPower = 0.0;
        rightCommandedPower = 0.0;

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        resetReadyTimer();
    }


    // =========================================================
    // RPM GETTERS
    // =========================================================

    /**
     * Returns the left wheel RPM sampled during the latest
     * update().
     */
    public double getLeftRPM() {
        return measuredLeftRPM;
    }


    /**
     * Returns the right wheel RPM sampled during the latest
     * update().
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
                GEAR_RATIO
        );
    }


    private double readRightRPM() {

        double ticksPerSecond =
                Math.abs(
                        rightMotor.getVelocity()
                );

        return ticksPerSecondToWheelRPM(
                ticksPerSecond,
                GEAR_RATIO
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

    public boolean isLeftReady(double leftRPM) {

        if (mode != ShooterMode.SHOOTING) {
            return false;
        }

        double rpmError =
                Math.abs(
                        TARGET_RPM
                                - leftRPM
                );

        return rpmError
                <= READY_TOLERANCE_RPM;
    }


    public boolean isRightReady(double rightRPM) {

        if (mode != ShooterMode.SHOOTING) {
            return false;
        }

        double rpmError =
                Math.abs(
                        TARGET_RPM
                                - rightRPM
                );

        return rpmError
                <= READY_TOLERANCE_RPM;
    }


    // =========================================================
    // SHOOTER READY
    // =========================================================

    /**
     * Uses RPM values captured during the latest update().
     */
    public boolean shooterReady() {

        return shooterReady(
                measuredLeftRPM,
                measuredRightRPM
        );
    }


    /**
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


        // Either wheel has dropped outside tolerance.
        if (!leftReady || !rightReady) {

            resetReadyTimer();
            return false;
        }


        // Both wheels have just entered tolerance.
        if (readySinceNanos == 0L) {

            readySinceNanos =
                    System.nanoTime();

            return false;
        }


        // Both wheels remain inside tolerance.
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
        return mode == ShooterMode.SHOOTING;
    }


    public boolean isIdle() {
        return mode == ShooterMode.IDLE;
    }


    public boolean isOff() {
        return mode == ShooterMode.OFF;
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

        if (TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        return Math.abs(
                leftMotor.getVelocity()
        )
                * 60.0
                / TICKS_PER_REV;
    }


    public double getRightMotorRPM() {

        if (TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        return Math.abs(
                rightMotor.getVelocity()
        )
                * 60.0
                / TICKS_PER_REV;
    }


    // =========================================================
    // TICKS PER SECOND TO SHOOTER-WHEEL RPM
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

    private double clipPower(double power) {

        return Math.max(
                0.0,
                Math.min(
                        1.0,
                        power
                )
        );
    }
}