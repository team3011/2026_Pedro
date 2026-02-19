package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Intake {
    private DcMotor intakeMotor;

    public static double motorPow = 0;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorPow = 0;
    }

    public void spinIntake(){
        motorPow = 1;
    }
    public void reverseIntake(){motorPow = -1;}
    public void stopIntake(){
        motorPow = 0;
    }

    public void update(){
        intakeMotor.setPower(motorPow);
    }
}
