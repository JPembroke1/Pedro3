package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter {

    public static double TARGET_RPM = 3000;

    // Feedforward (base power)
    public static double kF = 0.50;

    // Proportional correction
    public static double kP = 0.0002;

    private final DcMotorEx shooterMotor;
    private final DcMotorEx shooterMotor1;

    private final double ticksPerRev;

    public Shooter(HardwareMap hardwareMap) {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();
    }

    public void shooterOn() {

        double rpm =
                shooterMotor.getVelocity()
                        * 60.0
                        / ticksPerRev;

        double error = TARGET_RPM - rpm;

        double power = kF + (error * kP);

        power = Math.max(0.0, Math.min(1.0, power));

        shooterMotor.setPower(power);
        shooterMotor1.setPower(power);
    }

    public void shooterOff() {
        shooterMotor.setPower(0);
        shooterMotor1.setPower(0);
    }

    public double getRPM() {
        return shooterMotor.getVelocity()
                * 60.0
                / ticksPerRev;
    }

    public double getError() {
        return TARGET_RPM - getRPM();
    }
}