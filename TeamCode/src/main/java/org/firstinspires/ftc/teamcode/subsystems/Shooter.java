package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {

    // =====================================
    // SHOOTING SETTINGS
    // =====================================

    public static double TARGET_RPM_LEFT = 2800.0;
    public static double KF_LEFT = 0.7;
    public static double KP_LEFT = 0.00001;

    public static double TARGET_RPM_RIGHT = 5400.0;
    public static double KF_RIGHT = 1.0;
    public static double KP_RIGHT = 0.00001;

    // =====================================
    // IDLE SETTINGS
    // =====================================
    public static double IDLE_POWER_LEFT = 0.25;
    public static double IDLE_POWER_RIGHT = 0.35;

    // =====================================
    // READY TOLERANCES
    // =====================================

    public static double LEFT_READY_TOLERANCE = 300.0;
    public static double RIGHT_READY_TOLERANCE = 500.0;

    // =====================================
    // MOTOR AND GEAR SETTINGS
    // =====================================

    public static double TICKS_PER_REV = 28.0;

    // 10T motor sprocket driving 14T shooter sprocket
    public static double LEFT_GEAR_RATIO = 10.0 / 14.0;

    public static double RIGHT_GEAR_RATIO = 1.0;

    // =====================================
    // HARDWARE
    // =====================================

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    // =====================================
    // CONTROL
    // =====================================

    /*
     * true  = full shooting PF control
     * false = reduced idle power
     */
    private boolean shooting = false;

    // Used only for initialization and safety shutdown.
    private boolean stopped = true;

    // =====================================
    // CACHED VALUES
    // =====================================

    private double leftRPM;
    private double rightRPM;

    private double leftError;
    private double rightError;

    // =====================================
    // CONSTRUCTOR
    // =====================================

    public Shooter(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );

        rightMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor1"
        );

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        leftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterOff();
    }

    // =====================================
    // MODE COMMANDS
    // =====================================

    public void shooterOn() {
        stopped = false;
        shooting = true;
    }

    public void shooterIdle() {
        stopped = false;
        shooting = false;
    }

    public void shooterOff() {
        stopped = true;
        shooting = false;

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        leftRPM = 0.0;
        rightRPM = 0.0;

        leftError = 0.0;
        rightError = 0.0;
    }

    // =====================================
    // UPDATE
    // =====================================

    public void update() {

        // Read each motor velocity once per loop.
        leftRPM = calculateWheelRPM(
                leftMotor,
                LEFT_GEAR_RATIO
        );

        rightRPM = calculateWheelRPM(
                rightMotor,
                RIGHT_GEAR_RATIO
        );

        if (stopped) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

            leftError = 0.0;
            rightError = 0.0;
            return;
        }

        if (!shooting) {
            // Idle uses reduced fixed power.
            leftMotor.setPower(
                    clamp(IDLE_POWER_LEFT, 0.0, 1.0)
            );

            rightMotor.setPower(
                    clamp(IDLE_POWER_RIGHT, 0.0, 1.0)
            );

            /*
             * Keep the errors relative to shooting speed.
             * This means shooterReady() remains false at idle.
             */
            leftError = TARGET_RPM_LEFT - leftRPM;
            rightError = TARGET_RPM_RIGHT - rightRPM;
            return;
        }

        // Full shooting PF control.
        leftError = TARGET_RPM_LEFT - leftRPM;
        rightError = TARGET_RPM_RIGHT - rightRPM;

        double leftPower =
                KF_LEFT + (leftError * KP_LEFT);

        double rightPower =
                KF_RIGHT + (rightError * KP_RIGHT);

        leftMotor.setPower(
                clamp(leftPower, 0.0, 1.0)
        );

        rightMotor.setPower(
                clamp(rightPower, 0.0, 1.0)
        );
    }

    // =====================================
    // RPM
    // =====================================

    public double getLeftRPM() {
        return leftRPM;
    }

    public double getRightRPM() {
        return rightRPM;
    }

    // =====================================
    // ERROR
    // =====================================

    public double getLeftError() {
        return leftError;
    }

    public double getRightError() {
        return rightError;
    }

    // =====================================
    // POWER
    // =====================================

    public double getLeftPower() {
        return leftMotor.getPower();
    }

    public double getRightPower() {
        return rightMotor.getPower();
    }

    // =====================================
    // STATUS
    // =====================================

    public boolean isShooting() {
        return shooting && !stopped;
    }

    public boolean isIdle() {
        return !shooting && !stopped;
    }

    public boolean isStopped() {
        return stopped;
    }

    public boolean leftReady() {
        return isShooting()
                && Math.abs(leftError)
                <= LEFT_READY_TOLERANCE;
    }

    public boolean rightReady() {
        return isShooting()
                && Math.abs(rightError)
                <= RIGHT_READY_TOLERANCE;
    }

    public boolean shooterReady() {
        return leftReady() && rightReady();
    }

    // =====================================
    // UTILITIES
    // =====================================

    private double calculateWheelRPM(
            DcMotorEx motor,
            double gearRatio
    ) {
        return Math.abs(motor.getVelocity())
                * 60.0
                / TICKS_PER_REV
                * gearRatio;
    }

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}