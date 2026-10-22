package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {

    // =========================================================
    // CONFIGURATION
    // =========================================================

    public static double INTAKE_POWER = 1.0;

    public static double FEED_POWER = 0.70;

    public static double RAPID_FIRE_POWER = 1.00;

    public static double REVERSE_POWER = -1.0;


    // =========================================================
    // HARDWARE
    // =========================================================

    private final DcMotor intakeMotor;
    private final DcMotor intakeMotor1;


    // =========================================================
    // MODES
    // =========================================================

    public enum Mode {
        OFF,
        INTAKE,
        FEED,
        RAPID_FIRE,
        REVERSE
    }

    private Mode mode = Mode.OFF;


    // =========================================================
    // CACHED TELEMETRY
    // =========================================================

    private double commandedPower = 0.0;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(
                DcMotor.class,
                "intakeMotor"
        );

        intakeMotor1 = hardwareMap.get(
                DcMotor.class,
                "intakeMotor1"
        );

        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intakeMotor1.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        offMode();
    }


    // =========================================================
    // UPDATE
    // =========================================================

    public void update() {

        switch (mode) {

            case OFF:

                setPower(0.0);
                break;

            case INTAKE:

                setPower(INTAKE_POWER);
                break;

            case FEED:

                setPower(FEED_POWER);
                break;

            case RAPID_FIRE:

                setPower(RAPID_FIRE_POWER);
                break;

            case REVERSE:

                setPower(REVERSE_POWER);
                break;
        }
    }


    // =========================================================
    // MODE COMMANDS
    // =========================================================

    public void intakeMode() {
        mode = Mode.INTAKE;
    }

    public void feedMode() {
        mode = Mode.FEED;
    }

    public void rapidFireMode() {
        mode = Mode.RAPID_FIRE;
    }

    public void reverseMode() {
        mode = Mode.REVERSE;
    }

    public void offMode() {
        mode = Mode.OFF;
    }


    // =========================================================
    // BACKWARDS COMPATIBILITY
    // =========================================================

    public void intakeOn() {
        intakeMode();
        update();
    }

    public void feedOn() {
        feedMode();
        update();
    }

    public void rapidFireOn() {
        rapidFireMode();
        update();
    }

    public void intakeReverse() {
        reverseMode();
        update();
    }

    public void intakeOff() {
        offMode();
        update();
    }


    // =========================================================
    // INTERNAL
    // =========================================================

    private void setPower(double power) {

        commandedPower = power;

        intakeMotor.setPower(power);
        intakeMotor1.setPower(power);
    }


    // =========================================================
    // TELEMETRY
    // =========================================================

    public Mode getMode() {
        return mode;
    }

    public String getModeName() {
        return mode.name();
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public boolean isOff() {
        return mode == Mode.OFF;
    }

    public boolean isIntaking() {
        return mode == Mode.INTAKE;
    }

    public boolean isFeeding() {
        return mode == Mode.FEED;
    }

    public boolean isRapidFire() {
        return mode == Mode.RAPID_FIRE;
    }

    public boolean isReversing() {
        return mode == Mode.REVERSE;
    }
}