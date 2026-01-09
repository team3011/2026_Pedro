package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ejector {
    public static double ejectorPow = 0;
    CRServo ejector;

    public Ejector(HardwareMap hardwareMap){
        ejector = hardwareMap.get(CRServo.class, "ejector");
    }

    public void ejectorOn(){
        ejectorPow = 1;
    }

    public void ejectorOff(){
        ejectorPow = 0;
    }

    public void update(){
        ejector.setPower(ejectorPow);
    }
}
