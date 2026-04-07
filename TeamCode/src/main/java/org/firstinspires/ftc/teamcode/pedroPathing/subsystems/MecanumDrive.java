package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// referenced https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
@Configurable
public class MecanumDrive {
    public static double correctionMultiplier = 0.2;
    public static double ANGULAR_TOLERANCE_DEGREES = 4;
    public static double rotMulti = 1.1;
    public static double defaultRotSpeed = 0.2;
    public static int maximumCorrectionThreshold = 20;
    public static double rotSpeedMulti = 1;
    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public GoBildaPinpointDriver pinpoint;
    private double headingToMaintain = 0;
    private double headingInDEG;
    //limits how fast human can rotate robot, decrease to slow down rotation
    public double rotation_multi = 0.5;
    //acceptable angle tolerance in radians
    public double ANGULAR_TOLERANCE = Math.PI/90;
    public double flPow;
    public double rotSpeed;
    private double lastHeadingError = 0;
    public static double kP = 0.012;
    public static double kPClose = 0.020;
    public static double kD = 0.03;
    public static double minPowerThreshold = 0.05;
    private double kPTerm = 0;
    private double kDTerm = 0;
    private ElapsedTime correctionTimer = new ElapsedTime();
    private ElapsedTime settleElapsedTime = new ElapsedTime();
    private double filteredDerivative = 0;
    public static double derivativeFilterAlpha = .2;
    private boolean wasRotating = false;
    public static double settleTime = 0.15; // seconds to wait after release
    public MecanumDrive(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    //***************************************************
    //pulled from 2024 drive code to integrate Pinpoint
    //***************************************************

    //returns the current heading of the robot in RAD
    public double calcYaw() {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        return pinpoint.getHeading();
    }

    //determines the shortest path to desired angle in degrees
//    public double figureOutWhatIsShorter(double reading) {
//        double result;
//        double oppositeButEqualReading;
//
//        if (reading > 0) {
//            oppositeButEqualReading = reading - 360;
//        } else {
//            oppositeButEqualReading = reading + 360;
//        }
//
//        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
//        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);
//        boolean isOppositeReadingShorter =
//                normalReadingDifference > oppositeReadingDifference;
//
//        if (isOppositeReadingShorter) {
//            result = this.headingToMaintain - oppositeButEqualReading;
//        } else {
//            result = this.headingToMaintain - reading;
//        }
//        return -result;
//    }
    public double figureOutWhatIsShorter(double reading) {
        double error = this.headingToMaintain - reading;
        // normalize to (-180, 180)
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    //return a value that is below or equal to the limit
    private double limiter(double input, double lim) {
        //this will limit the pid to a range of -1 to 1
        if (input > lim) {
            input = lim;
        } else if (input < -lim) {
            input = -lim;
        }
        return input;
    }

    public void setHeadingToMaintain(double input){
        this.headingToMaintain = input;
    }


    public void drive2(double x, double y, double rx){
        x = -x;
        y = -y;
        rx = rx * rotSpeedMulti;
        double robotHeadingRAD = calcYaw();
        double robotHeadingDEG = Math.toDegrees(robotHeadingRAD);
        headingInDEG = robotHeadingDEG;
        if (rx == 0 && wasRotating) {
            if (settleElapsedTime.seconds() < settleTime) {
                this.headingToMaintain = robotHeadingDEG;
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                return;
            } else {
                wasRotating = false;
                lastHeadingError = 0;
                filteredDerivative = 0;
                correctionTimer.reset();
            }
        }
        if(rx == 0){
            //we're trying to maintain our current heading
            //calc the shortest deviation to target heading in degrees
            double shorter = this.figureOutWhatIsShorter(robotHeadingDEG);
            //check if we are within tolerance
            boolean isWithinAngularTolerance =
                    Math.abs(shorter) < ANGULAR_TOLERANCE_DEGREES;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                //this means we are moving
//                if (Math.abs(y) > 0 || Math.abs(x) > 0) {
//                    double rotSpeed = Math.abs(shorter);
//                    if (rotSpeed > maximumCorrectionThreshold) {
//                        rotSpeed = defaultRotSpeed;
//                    } else {
//                        rotSpeed = correctionMultiplier * rotSpeed * rotSpeed / 800.0;
//                    }
//                    rx = limiter(shorter, rotSpeed);
//                    this.rotSpeed = rx;
//                } else {
//                    //this means we are not moving but not pointing in the right direction
//                    double rotSpeed = Math.abs(shorter);
//                    if (rotSpeed > maximumCorrectionThreshold) {
//                        rotSpeed = defaultRotSpeed;
//                    } else {
//                        rotSpeed = 2 * correctionMultiplier * rotSpeed * rotSpeed / 800.0;
//                    }
//                    rx = limiter(shorter, rotSpeed);
//                    this.rotSpeed = rx;
//                }
                double dt = correctionTimer.seconds();
                correctionTimer.reset();
                if (dt < 0.005) dt = 0.005; // minimum 5ms to prevent division spike
                double rawDerivative = (shorter - lastHeadingError) / dt;
                filteredDerivative = derivativeFilterAlpha * rawDerivative + (1 - derivativeFilterAlpha) * filteredDerivative;
                lastHeadingError = shorter;
                double kPEffective = (Math.abs(shorter) < 5.0) ? kPClose : kP;
                kPTerm = -(kPEffective * shorter);
                kDTerm = -(kD * filteredDerivative);
                double correction = kPTerm + kDTerm;
                // only apply minimum power if we're outside tolerance
                if (Math.abs(shorter) > ANGULAR_TOLERANCE_DEGREES) {
                    if (Math.abs(correction) < minPowerThreshold) {
                        correction = Math.copySign(minPowerThreshold, correction);
                    }
                }
                rx = limiter(correction, 0.6);
                this.rotSpeed = rx;
            }else{
                filteredDerivative = 0; // add this
                lastHeadingError = 0;
            }
        }else{
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
//            this.headingToMaintain = robotHeadingDEG;
//            rotSpeed = rx;
            wasRotating = true;
            lastHeadingError = 0;
            filteredDerivative = 0;
            settleElapsedTime.reset();
            correctionTimer.reset();
            rotSpeed = rx;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(-robotHeadingRAD) - y * Math.sin(-robotHeadingRAD);
        double rotY = x * Math.sin(-robotHeadingRAD) + y * Math.cos(-robotHeadingRAD);
        rotX = rotX * rotMulti;

        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        //double frontLeftPower = (y+x+rx) / denominator;
        //double backLeftPower = (y - x + rx) / denominator;
        //double frontRightPower = (y-x-rx) / denominator;
        //double backRightPower = (y + x - rx) / denominator;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx)  / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx)   / denominator;
        double backRightPower = (rotY + rotX - rx)  / denominator;
//        flPow = backRightPower;

        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
    }
    public void rotateToHeading(double targetDeg, double x, double y) {
        double robotHeadingRAD = calcYaw();
        double robotHeadingDEG = Math.toDegrees(robotHeadingRAD);

        double error = targetDeg - robotHeadingDEG;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        if (Math.abs(error) > 8.0) {
            // fast phase - write motor powers directly, bypass drive2 entirely
            double fastRx = -Math.copySign(0.5, error);

            // still do field centric translation so driver can strafe during snap
            double rotX = (-x) * Math.cos(-robotHeadingRAD) - (-y) * Math.sin(-robotHeadingRAD);
            double rotY = (-x) * Math.sin(-robotHeadingRAD) + (-y) * Math.cos(-robotHeadingRAD);
            rotX = rotX * rotMulti;

            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(fastRx), 1);
            leftFront.setPower((rotY + rotX + fastRx) / denominator);
            rightFront.setPower((rotY - rotX - fastRx) / denominator);
            leftBack.setPower((rotY - rotX + fastRx) / denominator);
            rightBack.setPower((rotY + rotX - fastRx) / denominator);

            // keep PD state clean for handoff
            wasRotating = false;
            lastHeadingError = error;
            filteredDerivative = 0;
            correctionTimer.reset();

        } else {
            // close enough, hand off to PD correction cleanly
            headingToMaintain = targetDeg;
            wasRotating = false;
            lastHeadingError = 0;
            filteredDerivative = 0;
            correctionTimer.reset();
            drive2(x, y, 0);
        }
    }
    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public double getHeadingToMaintain(){
        return headingToMaintain;
    }
    public double getFlPow(){
        return flPow;
    }
    public double getRotSpeed(){
        return rotSpeed;
    }
    public double getHeadingInDEG(){
        return headingInDEG;
    }
    public double getFilteredDerivative() { return filteredDerivative; }
    public double getKPTerm() { return kPTerm; }
    public double getKDTerm() { return kDTerm; }
}
