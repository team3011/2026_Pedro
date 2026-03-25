package org.firstinspires.ftc.teamcode.pedroPathing.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.SuperSystem;

@Configurable       //if you want configurable constants
//@TeleOp       //if this is a teleop
@Autonomous   //if this is an auto
public class BlueAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, opmodeTimer;
    SuperSystem superSystem;
    private static Pose startPose = new Pose(22.680, 120.129, Math.toRadians(180));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower); // Build paths

        superSystem = new SuperSystem(hardwareMap, panelsTelemetry);
        superSystem.setIndex(1,1,1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        superSystem.startLimelight(0);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing=

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        autonomousPathUpdate();
        superSystem.update();
    }

    public static class Paths {
        public PathChain toShoot, toPickup, toShoot2;

        public Paths(Follower follower) {
            toShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    new Pose(61.505, 82.068)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
            toPickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(61.505, 82.068),
                                    new Pose(24, 83.554)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            toShoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.871, 83.554),
                                    new Pose(61.710, 82.143)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();
        }
    }

    public void setPathState(int pState) { // Updates autonomous pathing state
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(paths.toShoot);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    superSystem.shoot();
                    setPathState(2);
                }
                break;
            case 2:
                if(superSystem.isEmpty() && pathTimer.getElapsedTimeSeconds() > 0.25){
                    follower.followPath(paths.toPickup);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(paths.toShoot2);
                    setPathState(-1);
                }
                break;
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}
