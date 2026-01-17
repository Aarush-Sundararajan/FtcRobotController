package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTopRed extends OpMode {

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

    // Mirrored over field centerline (x' = 145 - x), y unchanged, heading' = PI - heading (wrapped)
    private final Pose startPose = new Pose(123.29383886255924, 122.65402843601899, Math.toRadians(36));
    private final Pose p1e      = new Pose(84.7345971563981,  83.43127962085308,  Math.toRadians(45));
    private final Pose p2e      = new Pose(125.29383886255925, 83.82464454976302, Math.toRadians(335));

    private final Pose p3e  = new Pose(128.62085308056873, 71.07109004739335, Math.toRadians(0));
    private final Pose p3c1 = new Pose(110.82464454976304, 72.84123222748815);

    private final Pose p4e  = new Pose(84.80094786729858, 83.44075829383887, Math.toRadians(45));
    private final Pose p4c1 = new Pose(85.13744075829384, 69.11374407582939);

    private final Pose p5e  = new Pose(124.86255924170616, 59.80568720379148, Math.toRadians(355));
    private final Pose p5c1 = new Pose(61.62796208530804,  55.120853080568736);

    private final Pose p6e  = new Pose(83.5781990521327,  11.260663507109001, Math.toRadians(66));
    private final Pose p6c1 = new Pose(88.77014218009479, 44.42890995260664);
    private final Pose p6c2 = new Pose(100.760663507109,  59.760663507109);

    private final Pose p7e  = new Pose(125.72037914691943, 36.35545023696683, Math.toRadians(355));
    private final Pose p7c1 = new Pose(62.25118483412322,  42.654028436018955);
    private final Pose p7c2 = new Pose(94.55924170616113,  30.533175355450233);

    private final Pose p8e  = new Pose(84.22274881516587, 11.767772511848342, Math.toRadians(66));

    private PathChain driveP1, driveP2, driveP3, driveP4, driveP5, driveP6, driveP7, driveP8;

    public void buildPaths() {
        driveP1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1e))
                .setLinearHeadingInterpolation(startPose.getHeading(), p1e.getHeading())
                .build();

        driveP2 = follower.pathBuilder()
                .addPath(new BezierLine(p1e, p2e))
                .setLinearHeadingInterpolation(p1e.getHeading(), p2e.getHeading())
                .build();

        driveP3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2e, p3c1, p3e))
                .setLinearHeadingInterpolation(p2e.getHeading(), p3e.getHeading())
                .build();

        driveP4 = follower.pathBuilder()
                .addPath(new BezierCurve(p3e, p4c1, p4e))
                .setLinearHeadingInterpolation(p3e.getHeading(), p4e.getHeading())
                .build();

        driveP5 = follower.pathBuilder()
                .addPath(new BezierCurve(p4e, p5c1, p5e))
                .setLinearHeadingInterpolation(p4e.getHeading(), p5e.getHeading())
                .build();

        driveP6 = follower.pathBuilder()
                .addPath(new BezierCurve(p5e, p6c1, p6c2, p6e))
                .setLinearHeadingInterpolation(p5e.getHeading(), p6e.getHeading())
                .build();

        driveP7 = follower.pathBuilder()
                .addPath(new BezierCurve(p6e, p7c1, p7c2, p7e))
                .setLinearHeadingInterpolation(p6e.getHeading(), p7e.getHeading())
                .build();

        driveP8 = follower.pathBuilder()
                .addPath(new BezierLine(p7e, p8e))
                .setLinearHeadingInterpolation(p7e.getHeading(), p8e.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case Path1:
                follower.followPath(driveP1, true);
                setPathState(PathState.Shoot1);
                break;

            case Shoot1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(driveP2, true);
                    setPathState(PathState.Path2);
                }
                break;

            case Path2:
                if (!follower.isBusy()) {
                    follower.followPath(driveP3, true);
                    setPathState(PathState.Path3);
                }
                break;

            case Path3:
                if (!follower.isBusy()) {
                    setPathState(PathState.Release);
                }
                break;

            case Release:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(driveP4, true);
                    setPathState(PathState.Path4);
                }
                break;

            case Path4:
                if (!follower.isBusy()) {
                    setPathState(PathState.Shoot2);
                }
                break;

            case Shoot2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(driveP5, true);
                    setPathState(PathState.Path5);
                }
                break;

            case Path5:
                if (!follower.isBusy()) {
                    follower.followPath(driveP6, true);
                    setPathState(PathState.Path6);
                }
                break;

            case Path6:
                if (!follower.isBusy()) {
                    setPathState(PathState.Shoot3);
                }
                break;

            case Shoot3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(driveP7, true);
                    setPathState(PathState.Path7);
                }
                break;

            case Path7:
                if (!follower.isBusy()) {
                    follower.followPath(driveP8, true);
                    setPathState(PathState.Path8);
                }
                break;

            case Path8:
                if (!follower.isBusy()) {
                    setPathState(PathState.Shoot4);
                }
                break;

            case Shoot4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
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
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
