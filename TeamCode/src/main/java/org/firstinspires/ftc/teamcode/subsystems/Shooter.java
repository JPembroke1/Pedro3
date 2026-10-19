package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {

    // =========================================================
    // TUNABLE MOTOR POWERS
    // =========================================================

    // Competition shooting powers
    public static double LEFT_SHOOT_POWER = 0.75;
    public static double BACK_SHOOT_POWER = 1.00;

    // Lower powers used to keep the wheels spinning
    public static double LEFT_IDLE_POWER = 0.25;
    public static double BACK_IDLE_POWER = 0.30;

    // Encoder ticks per MOTOR revolution
    public static double TICKS_PER_REV = 28.0;

    // 10-tooth motor sprocket driving 14-tooth shooter sprocket
    public static double LEFT_GEAR_RATIO = 10.0 / 14.0;

    // Back/right wheel is direct drive
    public static double BACK_GEAR_RATIO = 1.0;


    // =========================================================
    // HARDWARE
    // =========================================================

    private final DcMotorEx leftMotor;
    private final DcMotorEx backMotor;


    // =========================================================
    // STATE
    // =========================================================

    private enum ShooterMode {
        OFF,
        IDLE,
        SHOOTING
    }

    private ShooterMode mode = ShooterMode.OFF;

    private double leftCommandedPower = 0.0;
    private double backCommandedPower = 0.0;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Shooter(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );

        backMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor1"
        );

        /*
         * Change either direction if that wheel spins backwards.
         */
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        backMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        backMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        /*
         * Direct power control.
         *
         * Encoders can still be read with getVelocity().
         */
        leftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        backMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterOff();
    }


    // =========================================================
    // PERIODIC UPDATE
    // =========================================================

    /**
     * Applies the currently configured motor powers.
     *
     * Call this once during every TeleOp loop. This allows live
     * configuration changes to take effect immediately.
     */
    public void update() {

        switch (mode) {
            case SHOOTING:
                leftCommandedPower = clip(LEFT_SHOOT_POWER);
                backCommandedPower = clip(BACK_SHOOT_POWER);
                break;

            case IDLE:
                leftCommandedPower = clip(LEFT_IDLE_POWER);
                backCommandedPower = clip(BACK_IDLE_POWER);
                break;

            case OFF:
            default:
                leftCommandedPower = 0.0;
                backCommandedPower = 0.0;
                break;
        }

        leftMotor.setPower(leftCommandedPower);
        backMotor.setPower(backCommandedPower);
    }


    // =========================================================
    // MODE COMMANDS
    // =========================================================

    public void shooterOn() {
        mode = ShooterMode.SHOOTING;
    }

    public void shooterIdle() {
        mode = ShooterMode.IDLE;
    }

    public void shooterOff() {
        mode = ShooterMode.OFF;

        leftCommandedPower = 0.0;
        backCommandedPower = 0.0;

        leftMotor.setPower(0.0);
        backMotor.setPower(0.0);
    }


    // =========================================================
    // RPM READINGS
    // =========================================================

    public double getLeftRPM() {
        return ticksPerSecondToWheelRPM(
                Math.abs(leftMotor.getVelocity()),
                LEFT_GEAR_RATIO
        );
    }

    public double getBackRPM() {
        return ticksPerSecondToWheelRPM(
                Math.abs(backMotor.getVelocity()),
                BACK_GEAR_RATIO
        );
    }

    public double getLeftTicksPerSecond() {
        return Math.abs(leftMotor.getVelocity());
    }

    public double getBackTicksPerSecond() {
        return Math.abs(backMotor.getVelocity());
    }


    // =========================================================
    // POWER INFORMATION
    // =========================================================

    public double getLeftCommandedPower() {
        return leftCommandedPower;
    }

    public double getBackCommandedPower() {
        return backCommandedPower;
    }


    // =========================================================
    // MODE INFORMATION
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
    // UTILITIES
    // =========================================================

    private double ticksPerSecondToWheelRPM(
            double ticksPerSecond,
            double gearRatio
    ) {
        if (TICKS_PER_REV <= 0.0 || gearRatio <= 0.0) {
            return 0.0;
        }

        double motorRPM =
                ticksPerSecond
                        * 60.0
                        / TICKS_PER_REV;

        return motorRPM * gearRatio;
    }

    private double clip(double power) {
        return Math.max(
                0.0,
                Math.min(1.0, power)
        );
    }
}