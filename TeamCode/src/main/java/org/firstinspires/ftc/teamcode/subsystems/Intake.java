package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

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
        intakeMotor.setPower(1);
        intakeMotor1.setPower(1);
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