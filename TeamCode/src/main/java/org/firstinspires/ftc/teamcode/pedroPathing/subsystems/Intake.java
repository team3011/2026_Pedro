package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intakeMotor;

    public static double motorPow = 0;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    public void spinIntake(){
        motorPow = 1;
    }

    public void stopIntake(){
        motorPow = 0;
    }

    public void update(){
        intakeMotor.setPower(motorPow);
    }
}
