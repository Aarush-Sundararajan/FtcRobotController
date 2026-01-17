package org.firstinspires.ftc.teamcode;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierPoint;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous
public class AutoBottomBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        Path1,
        Shoot1,
        Path2,
        Path3,
        Release,
        Path4,
        Shoot2,
        Path5,
        Path6,
        Shoot3,
        Path7,
        Path8,
        Shoot4
    }

    PathState pathState;


    private final Pose startPose = new Pose(57.706,8.853,Math.toRadians(90));
    private final Pose p1e = new Pose(58.6872037914692,15.758293838862556,Math.toRadians(114));
    private final Pose p2e = new Pose(19.51184834123224,84.21327014218008,Math.toRadians(185));
    private final Pose p2c1 = new Pose(78.54976303317535,90.98104265402844);
    private final Pose p3e = new Pose(16.60189573459715,73.25592417061613,Math.toRadians(180));
    private final Pose p3c1 = new Pose(23.075829383886262,70.77725118483411);
    private final Pose p4e = new Pose(59.8957345971564,88.9526066350711,Math.toRadians(135));
    private final Pose p5e = new Pose(18.876777251184826,59.914691943127956,Math.toRadians(185));
    private final Pose p5c1 = new Pose(49.47867298578199,54.767772511848335);
    private final Pose p5c2 = new Pose(59.936018957345965,59.6658767772512);
    private final Pose p6e = new Pose(60.90995260663507,14.672985781990521,Math.toRadians(114));
    private final Pose p6c1 = new Pose(68.43364928909952,57.66824644549762);
    private final Pose p7e = new Pose(16.943127962085313,35.45497630331754,Math.toRadians(190));
    private final Pose p7c1 = new Pose(54.523696682464454,39.17772511848342);
    private final Pose p7c2 = new Pose(68.53791469194313,36.15402843601896);
    private final Pose p8e = new Pose(60.90995260663507,14.672985781990521,Math.toRadians(114));


    private PathChain driveP1,driveP2,driveP3,driveP4,driveP5,driveP6,driveP7,driveP8;

    public void buildPaths(){
        //put in coordinates for start pose --> end pose
        driveP1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,p1e))
                .setLinearHeadingInterpolation(startPose.getHeading(),p1e.getHeading())
                .build();
        driveP2 = follower.pathBuilder()
                .addPath(new BezierCurve(p1e,p2c1,p2e))
                .setLinearHeadingInterpolation(p1e.getHeading(),p2e.getHeading())
                .build();
        // P3: p2e -> p3e (1 control)
        driveP3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2e, p3c1, p3e))
                .setLinearHeadingInterpolation(p2e.getHeading(), p3e.getHeading())
                .build();

        // P4: p3e -> p4e
        driveP4 = follower.pathBuilder()
                .addPath(new BezierLine(p3e, p4e))
                .setLinearHeadingInterpolation(p3e.getHeading(), p4e.getHeading())
                .build();

        // P5: p4e -> p5e (2 controls)
        driveP5 = follower.pathBuilder()
                .addPath(new BezierCurve(p4e, p5c1, p5c2, p5e))
                .setLinearHeadingInterpolation(p4e.getHeading(), p5e.getHeading())
                .build();

        // P6: p5e -> p6e (1 control)
        driveP6 = follower.pathBuilder()
                .addPath(new BezierCurve(p5e, p6c1, p6e))
                .setLinearHeadingInterpolation(p5e.getHeading(), p6e.getHeading())
                .build();

        // P7: p6e -> p7e (2 controls)
        driveP7 = follower.pathBuilder()
                .addPath(new BezierCurve(p6e, p7c1, p7c2, p7e))
                .setLinearHeadingInterpolation(p6e.getHeading(), p7e.getHeading())
                .build();

        // P8: p7e -> p8e
        driveP8 = follower.pathBuilder()
                .addPath(new BezierLine(p7e, p8e))
                .setLinearHeadingInterpolation(p7e.getHeading(), p8e.getHeading())
                .build();


    }



    public void statePathUpdate(){
        switch(pathState){

            case Path1:
                follower.followPath(driveP1, true);
                setPathState(PathState.Shoot1);
                break;

            case Shoot1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(driveP2, true);
                    setPathState(PathState.Path2);
                }
                break;

            case Path2:
                if(!follower.isBusy()){
                    follower.followPath(driveP3, true);
                    setPathState(PathState.Path3);
                }
                break;

            case Path3:
                if(!follower.isBusy()){
                    setPathState(PathState.Release);
                }
                break;

            case Release:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(driveP4, true);
                    setPathState(PathState.Path4);
                }
                break;

            case Path4:
                if(!follower.isBusy()){
                    setPathState(PathState.Shoot2);
                }
                break;

            case Shoot2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(driveP5, true);
                    setPathState(PathState.Path5);
                }
                break;

            case Path5:
                if(!follower.isBusy()){
                    follower.followPath(driveP6, true);
                    setPathState(PathState.Path6);
                }
                break;

            case Path6:
                if(!follower.isBusy()){
                    setPathState(PathState.Shoot3);
                }
                break;

            case Shoot3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(driveP7, true);
                    setPathState(PathState.Path7);
                }
                break;

            case Path7:
                if(!follower.isBusy()){
                    follower.followPath(driveP8, true);
                    setPathState(PathState.Path8);
                }
                break;

            case Path8:
                if(!follower.isBusy()){
                    setPathState(PathState.Shoot4);
                }
                break;

            case Shoot4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    telemetry.addLine("Auto sequence complete");
                }
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.Path1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();;
        follower = Constants.createFollower(hardwareMap);
        //TODO add any other init ( Vihaan lock in )

        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time",pathTimer.getElapsedTimeSeconds());
    }

}
