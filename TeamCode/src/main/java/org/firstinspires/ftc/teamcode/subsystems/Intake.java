package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
@Configurable
public class Intake {

    public static double INTAKE_POWER = 1.0;
    private final DcMotor intakeMotor;
    private final DcMotor intakeMotor1;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intakeMotor1");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setPower(0);
        intakeMotor1.setPower(0);

    }
    public void intakeOn(){
        intakeMotor.setPower(INTAKE_POWER);
        intakeMotor1.setPower(INTAKE_POWER);
    }

    public void intakeOff(){
        intakeMotor.setPower(0);
        intakeMotor1.setPower(0);
    }

    public void intakeReverse(){
        intakeMotor.setPower(-1);
        intakeMotor1.setPower(-1);
    }

}