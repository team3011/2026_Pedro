package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class Ejector {
    public static double ejectorPos = .57;
    Servo ejector;

    public Ejector(HardwareMap hardwareMap){
        ejector = hardwareMap.get(Servo.class, "ejector");
    }

    public void ejectorTo(double p){
        ejectorPos = p;
    }

    public void update(){
        ejector.setPosition(ejectorPos);
    }
}
