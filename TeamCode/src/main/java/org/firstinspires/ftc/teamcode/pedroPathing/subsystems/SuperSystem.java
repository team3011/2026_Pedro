package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class SuperSystem {
    Index index;
    Ejector ejector;
    Intake intake;
    Shooter shooter;
    MyLimeLight limelight;
    Servo blinkin;
    TelemetryManager dashboardTelemetry;
    ElapsedTime lockTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean isAiming = false;
    private boolean shootReady = false;
    private boolean holdPosition = false;
    private boolean isIntaking = false;
    private boolean intakeForceStop = false;
    private boolean aimForceStop = false;
    private boolean isScanning = false;
    private boolean orderToShoot = false;
    private boolean indexCheck = false;
    private int aimDirection;
    private int color;
    private double targShooterSpeed;
    private double headingToMaintain;
    public static int toggleState = 0;
    public static int lockTime = 200;
    public static double xLeftLimit = -3;
    public static double xRightLimit = 3;
    public static double shooterPower = 0.9;
    public SuperSystem(HardwareMap hardwareMap, TelemetryManager dashboard){
        index = new Index(hardwareMap, dashboard);
        ejector = new Ejector(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, dashboard);
        limelight = new MyLimeLight(hardwareMap, dashboard);
        blinkin = hardwareMap.get(Servo.class, "abrahamBlinkin");
        dashboardTelemetry = dashboard;
    }
    public void startIntaking(){
        isIntaking = true;
    }

    public void aimShooter(){
        shooterOn();
        aimForceStop = false;
        isAiming = true;
    }
    public void shoot(){
        orderToShoot = true;
    }

    public void reset(){
        ejector.reset();
        index.reset();
    }

    public void shooterOn(){
        shooter.startShooter();
        shooter.defaultShooterPos();
    }
    public void shooterOff(){
        shooter.stopShooter();
    }

    public void setLED(){
        if(toggleState == 0){
            if(color == 1){
                 blinkin.setPosition(1);
            }else if(color == 0){
                blinkin.setPosition(.611);
            }
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
    public void forceStopIntake(){
        intakeForceStop = true;
    }
    public boolean intakeIsBusy(){
        return isIntaking;
    }
    public void reverseIntake(){
        intake.reverseIntake();
        isIntaking = false;
    }

    public void update(){
        if(isIntaking){
            intake.spinIntake();
            index.setIsSensing(true);
            index.toPickupTarget(0);
            if(index.currentSlotStatus()){
                index.toPickupTarget(0);
            }
            if(index.isFull() || intakeForceStop){
                lockTimer.reset();
                while(lockTimer.milliseconds()<lockTime){}
                isIntaking = false;
                intakeForceStop = false;
                index.toShoot(0);
                index.setIsSensing(false);
//                index.reset();
                intake.stopIntake();
            }
        }
        if(isAiming){
            if (limelight.getxLoc() < xLeftLimit){ //robot moves left
                aimDirection = -1;
            }else if (limelight.getxLoc() > xRightLimit){ //robot moves right
                aimDirection = 1;
            }else{
                aimDirection = 0;
                shootReady = true;
//                isAiming = false;
                holdPosition = true;
            }
            if(aimForceStop){
                limelight.stop();
                isAiming = false;
                aimForceStop = false;
                holdPosition = false;
            }
        }
//        if(shootReady && orderToShoot){
        if(orderToShoot){
            limelight.update();
            targShooterSpeed = shooter.calculateSpeed(limelight.getDistance());
            shooter.setShooterSpeed(targShooterSpeed);
            shooter.update();
            if(!index.isEmpty()) {
                if (toggleState == 1 || toggleState == 2) {
                    index.toShootTarget(toggleState);
                    if(index.isAtPosition() && shooter.isSpunUp()){
                        ejector.quickfire();
                        index.emptyCurrentSlot();
                    }
                }else if(toggleState == 0){
                    index.toShootClosest();
                    if(index.isAtPosition() && shooter.isSpunUp() && limelight.getDistance() > 0){
                        ejector.quickfire();
                        index.emptyCurrentSlot();
                    }
                }
            }else{
                shooter.stopShooter();
                shootReady = false;
                holdPosition = false;
                orderToShoot = false;
                isAiming = false;
            }
        }
        dashboardTelemetry.addData("isChecking", indexCheck);
        dashboardTelemetry.addData("isCheckingIndexSubsys",index.getCheck());
        dashboardTelemetry.addData("isIntaking", isIntaking);
        dashboardTelemetry.addData("forcestop", intakeForceStop);
        dashboardTelemetry.addData("shoot ordered?", orderToShoot);
        dashboardTelemetry.addData("desired shooter speed", targShooterSpeed);
        dashboardTelemetry.addData("aimDir", aimDirection);
        dashboardTelemetry.addData("aim state", getAimStatus());
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
    public boolean getIsIntaking(){
        return isIntaking;
    }
    public void forceStopAiming(){
        aimForceStop = true;
    }
    public boolean holdingPosition(){
        return holdPosition;
    }
    public boolean isEmpty(){
        return index.isEmpty();
    }
    public void startLimelight(int pl){
        limelight.start(pl);
    }
    public void setAllianceColor(int c){
        color = c;
    }
    public void setIndex(int q, int w, int e){
        index.setIndexSlots(q,w,e);
    }
    public void checkIndex(){
        indexCheck = true;
    }
}
