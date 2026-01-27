package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class Shooter {

    public static double shooterPow = 0;

    public static double shooterPos = 0.4;
    DcMotorEx shooterL;
    DcMotorEx shooterR;
    Servo pivotL;
    Servo pivotR;

    public Shooter(HardwareMap hardwareMap){
        shooterL = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooterR = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotL = hardwareMap.get(Servo.class, "leftPivot");
        pivotR = hardwareMap.get(Servo.class, "rightPivot");
        pivotR.setDirection(Servo.Direction.REVERSE);
    }

    public void setShooterPower(double p){
        shooterPow = p;
    }

    public void stopShooter(){
        shooterPow = 0;
    }

    public void setShooterPos(double pos){
        shooterPos = pos;
    }

    public void update(){
        shooterL.setPower(shooterPow);
        shooterR.setPower(shooterPow);

        pivotL.setPosition(shooterPos);
        pivotR.setPosition(shooterPos);
    }
}
