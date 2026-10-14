package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {

    // Main wheel
    public static double TARGET_RPM_LEFT = 2800;
    public static double kF_LEFT = 0.78;
    public static double kP_LEFT = 0.0001;

    // Counter wheel
    public static double TARGET_RPM_RIGHT = 11200;
    public static double kF_RIGHT = 0.95;
    public static double kP_RIGHT = 0.00005;

    public static final double TICKS_PER_REV = 28.0;

    public static double LEFT_GEAR_RATIO = 10.0 / 14.0;
    public static double RIGHT_GEAR_RATIO = 2.0;

    private final DcMotorEx left;
    private final DcMotorEx right;

    public Shooter(HardwareMap hardwareMap) {

        left = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        right = hardwareMap.get(DcMotorEx.class, "shooterMotor1");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);


        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shooterOn() {

        double leftRPM = getLeftRPM();
        double rightRPM = getRightRPM();

        double leftPower =
                kF_LEFT + ((TARGET_RPM_LEFT - leftRPM) * kP_LEFT);

        double rightPower =
                kF_RIGHT + ((TARGET_RPM_RIGHT - rightRPM) * kP_RIGHT);

        left.setPower(Math.max(0, Math.min(1, leftPower)));
        right.setPower(Math.max(0, Math.min(1, rightPower)));
    }

    public void shooterOff() {
        left.setPower(0);
        right.setPower(0);
    }

    public double getLeftMotorRPM() {
        return (left.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public double getRightMotorRPM() {
        return (right.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public double getLeftRPM() {
        return getLeftMotorRPM() * LEFT_GEAR_RATIO;
    }

    public double getRightRPM() {
        return getRightMotorRPM() * RIGHT_GEAR_RATIO;
    }

    public double getLeftPower() {
        return left.getPower();
    }

    public double getRightPower() {
        return right.getPower();
    }

    public boolean shooterReady(double toleranceRPM) {

        return Math.abs(TARGET_RPM_LEFT - getLeftRPM()) <= toleranceRPM
                && Math.abs(TARGET_RPM_RIGHT - getRightRPM()) <= toleranceRPM;
    }
}