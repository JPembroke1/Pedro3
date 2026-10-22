package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
public class Shooter {

    // =========================================================
    // FRONT SHOOTER CONFIGURATION
    // =========================================================

    public static double FRONT_SHOOT_RPM = 2800.0;
    public static double FRONT_IDLE_RPM = 1000.0;

    public static double FRONT_KF = 0.00025;
    public static double FRONT_KP = 0.00015;

    public static double FRONT_MOTOR_TO_ROLLER_RATIO =
            10.0 / 14.0;

    public static double FRONT_MAX_POWER = 1.0;

    // =========================================================
    // REAR COUNTER-ROLLER CONFIGURATION
    // =========================================================

    public static double REAR_SHOOT_RPM = 5000.0;
    public static double REAR_IDLE_RPM = 2000.0;

    public static double REAR_KF = 0.00018;
    public static double REAR_KP = 0.00010;

    public static double REAR_MOTOR_TO_ROLLER_RATIO = 1.0;

    public static double REAR_MAX_POWER = 1.0;

    // =========================================================
    // ENCODER CONFIGURATION
    // =========================================================

    public static double TICKS_PER_REV = 28.0;

    // =========================================================
    // VOLTAGE COMPENSATION CONFIGURATION
    // =========================================================

    public static double NOMINAL_VOLTAGE = 13.0;
    public static double MIN_COMPENSATION_VOLTAGE = 11.0;

    // =========================================================
    // HARDWARE
    // =========================================================

    private final HardwareMap hardwareMap;

    private final DcMotorEx frontMotor;
    private final DcMotorEx rearMotor;

    // =========================================================
    // SHOOTER MODE
    // =========================================================

    public enum ShooterMode {
        OFF,
        IDLE,
        SHOOTING
    }

    private ShooterMode mode = ShooterMode.OFF;

    // =========================================================
    // CACHED TELEMETRY
    // =========================================================

    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double voltageCompensation = 1.0;

    private double measuredFrontRPM = 0.0;
    private double measuredRearRPM = 0.0;

    private double activeFrontTargetRPM = 0.0;
    private double activeRearTargetRPM = 0.0;

    private double frontErrorRPM = 0.0;
    private double rearErrorRPM = 0.0;

    private double frontCommandedPower = 0.0;
    private double rearCommandedPower = 0.0;

    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Shooter(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        frontMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );

        rearMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor1"
        );

        frontMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        rearMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        frontMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        rearMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        frontMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rearMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterOff();
    }

    // =========================================================
    // UPDATE
    // =========================================================

    public void update() {

        batteryVoltage = readBatteryVoltage();

        voltageCompensation =
                NOMINAL_VOLTAGE
                        / Math.max(
                        MIN_COMPENSATION_VOLTAGE,
                        batteryVoltage
                );

        measuredFrontRPM = readRollerRPM(
                frontMotor,
                FRONT_MOTOR_TO_ROLLER_RATIO
        );

        measuredRearRPM = readRollerRPM(
                rearMotor,
                REAR_MOTOR_TO_ROLLER_RATIO
        );

        if (mode == ShooterMode.OFF) {

            activeFrontTargetRPM = 0.0;
            activeRearTargetRPM = 0.0;

            frontErrorRPM = 0.0;
            rearErrorRPM = 0.0;

            frontCommandedPower = 0.0;
            rearCommandedPower = 0.0;

            frontMotor.setPower(0.0);
            rearMotor.setPower(0.0);

            return;
        }

        if (mode == ShooterMode.SHOOTING) {

            activeFrontTargetRPM =
                    Math.max(0.0, FRONT_SHOOT_RPM);

            activeRearTargetRPM =
                    Math.max(0.0, REAR_SHOOT_RPM);

        } else {

            activeFrontTargetRPM =
                    Math.max(0.0, FRONT_IDLE_RPM);

            activeRearTargetRPM =
                    Math.max(0.0, REAR_IDLE_RPM);
        }

        frontErrorRPM =
                activeFrontTargetRPM - measuredFrontRPM;

        rearErrorRPM =
                activeRearTargetRPM - measuredRearRPM;

        frontCommandedPower = calculatePower(
                activeFrontTargetRPM,
                measuredFrontRPM,
                FRONT_KF,
                FRONT_KP,
                FRONT_MAX_POWER
        );

        rearCommandedPower = calculatePower(
                activeRearTargetRPM,
                measuredRearRPM,
                REAR_KF,
                REAR_KP,
                REAR_MAX_POWER
        );

        frontMotor.setPower(frontCommandedPower);
        rearMotor.setPower(rearCommandedPower);
    }

    // =========================================================
    // VELOCITY CONTROLLER
    // =========================================================

    private double calculatePower(
            double targetRPM,
            double measuredRPM,
            double kF,
            double kP,
            double maximumPower
    ) {

        if (targetRPM <= 0.0) {
            return 0.0;
        }

        double errorRPM =
                targetRPM - measuredRPM;

        double feedforwardPower =
                targetRPM * kF;

        double proportionalPower =
                errorRPM * kP;

        double output =
                (feedforwardPower + proportionalPower)
                        * voltageCompensation;

        return clipPower(
                output,
                maximumPower
        );
    }

    // =========================================================
    // BATTERY VOLTAGE
    // =========================================================

    private double readBatteryVoltage() {

        double highestVoltage = 0.0;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {

            double sensorVoltage =
                    sensor.getVoltage();

            if (sensorVoltage > 0.0
                    && !Double.isNaN(sensorVoltage)
                    && !Double.isInfinite(sensorVoltage)) {

                highestVoltage = Math.max(
                        highestVoltage,
                        sensorVoltage
                );
            }
        }

        if (highestVoltage > 0.0) {
            return highestVoltage;
        }

        return NOMINAL_VOLTAGE;
    }

    // =========================================================
    // RPM CALCULATION
    // =========================================================

    private double readRollerRPM(
            DcMotorEx motor,
            double motorToRollerRatio
    ) {

        if (TICKS_PER_REV <= 0.0
                || motorToRollerRatio <= 0.0) {

            return 0.0;
        }

        double ticksPerSecond =
                Math.abs(
                        motor.getVelocity()
                );

        if (Double.isNaN(ticksPerSecond)
                || Double.isInfinite(ticksPerSecond)) {

            return 0.0;
        }

        double motorRPM =
                ticksPerSecond
                        * 60.0
                        / TICKS_PER_REV;

        return motorRPM
                * motorToRollerRatio;
    }

    // =========================================================
    // MODE CONTROL
    // =========================================================

    public void shooterOn() {
        mode = ShooterMode.SHOOTING;
    }

    public void shooterIdle() {
        mode = ShooterMode.IDLE;
    }

    public void shooterOff() {

        mode = ShooterMode.OFF;

        activeFrontTargetRPM = 0.0;
        activeRearTargetRPM = 0.0;

        frontErrorRPM = 0.0;
        rearErrorRPM = 0.0;

        frontCommandedPower = 0.0;
        rearCommandedPower = 0.0;

        frontMotor.setPower(0.0);
        rearMotor.setPower(0.0);
    }

    // =========================================================
    // MODE GETTERS
    // =========================================================

    public ShooterMode getMode() {
        return mode;
    }

    public String getModeName() {
        return mode.name();
    }

    public boolean isShooting() {
        return mode == ShooterMode.SHOOTING;
    }

    public boolean isIdle() {
        return mode == ShooterMode.IDLE;
    }

    public boolean isOff() {
        return mode == ShooterMode.OFF;
    }

    // =========================================================
    // CACHED TELEMETRY GETTERS
    // =========================================================

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getVoltageCompensation() {
        return voltageCompensation;
    }

    public double getFrontRPM() {
        return measuredFrontRPM;
    }

    public double getRearRPM() {
        return measuredRearRPM;
    }

    public double getFrontTargetRPM() {
        return activeFrontTargetRPM;
    }

    public double getRearTargetRPM() {
        return activeRearTargetRPM;
    }

    public double getFrontErrorRPM() {
        return frontErrorRPM;
    }

    public double getRearErrorRPM() {
        return rearErrorRPM;
    }

    public double getFrontCommandedPower() {
        return frontCommandedPower;
    }

    public double getRearCommandedPower() {
        return rearCommandedPower;
    }

    // =========================================================
    // POWER CLIPPING
    // =========================================================

    private double clipPower(
            double power,
            double maximumPower
    ) {

        if (Double.isNaN(power)
                || Double.isInfinite(power)) {

            return 0.0;
        }

        double safeMaximum =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                maximumPower
                        )
                );

        return Math.max(
                0.0,
                Math.min(
                        safeMaximum,
                        power
                )
        );
    }
}