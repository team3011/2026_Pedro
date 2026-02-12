package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class SuperSystem {
    Index index;
    Ejector ejector;
    Intake intake;
    Shooter shooter;
    MyLimeLight limelight;
    Servo blinkin;
    private boolean isAiming = false;
    private boolean shootReady = false;
    private boolean holdPosition = false;
    private boolean isIntaking = false;
    private boolean isScanning = false;
    private int aimDirection;
    public static int toggleState = 0;
    private double xLeftLimit = -0.1;
    private double xRightLimit = 0.1;
    public static double shooterPower = 0.9;
    public SuperSystem(HardwareMap hardwareMap, TelemetryManager dashboard){
        index = new Index(hardwareMap, dashboard);
        ejector = new Ejector(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new MyLimeLight(hardwareMap, dashboard);
        blinkin = hardwareMap.get(Servo.class, "abrahamBlinkin");
    }
    public void startIntaking(){
        isIntaking = true;
    }

    public void aimShooter(){
        shooterOn();
        isAiming = true;
    }

    public void reset(){
        index.reset();
    }

    public void shooterOn(){
        shooter.setShooterPower(shooterPower);
        shooter.defaultShooterPos();
    }

    public void setLED(){
        if(toggleState == 0){
            blinkin.setPosition(1);
        }else if(toggleState == 1){
            blinkin.setPosition(0.5);
        }else if(toggleState == 2){
            blinkin.setPosition(0.722);
        }
    }
    public void toggle(){
        toggleState += 1;
        if (toggleState > 2) {
            toggleState = 0;
        }
        setLED();
    }

    public void update(){
        if(isIntaking){
            intake.spinIntake();
            index.setIsSensing(true);
            if(index.currentSlotStatus()){
                index.toPickupTarget(0);
                if(index.isFull()){
                    isIntaking = false;
                    intake.stopIntake();
                }
            }
        }
        if(isAiming){
            if (limelight.getxLoc() < xLeftLimit){ //robot moves left
                aimDirection = -1;
            } else if (limelight.getxLoc() > xRightLimit){ //robot moves right
                aimDirection = 1;
            }else{
                aimDirection = 0;
                shootReady = true;
                isAiming = false;
                holdPosition = true;
            }
        }
        if(shootReady){

        }
        setLED();
        index.update();
        ejector.update();
        intake.update();
        shooter.update();
        limelight.update();
    }

    public int getAimDirection(){
        return aimDirection;
    }
    public boolean getAimStatus(){
        return isAiming;
    }
    public boolean holdingPosition(){
        return holdPosition;
    }
}
