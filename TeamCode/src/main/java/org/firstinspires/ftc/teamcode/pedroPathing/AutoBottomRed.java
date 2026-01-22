package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoBottomRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private DcMotor intake = null;
    private DcMotor guider = null;
    private DcMotor shootLeft = null;
    private DcMotor shootRight = null;

    // --- Added for guiderReversal timing (non-blocking) ---
    private Timer actionTimer = new Timer();
    private boolean guiderReversalActive = false;

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

    // Field mirror: x' = 145 - x, y' = y, heading' = PI - heading
    private final Pose startPose = new Pose(87.294, 8.853, Math.toRadians(90));

    private final Pose p1e = new Pose(86.3127962085308, 15.758293838862556, Math.toRadians(66));
    private final Pose p2e = new Pose(125.48815165876776, 74.21327014218008, Math.toRadians(355)); //reduced
    private final Pose p2c1 = new Pose(66.45023696682465, 87.98104265402844);

    private final Pose p3e = new Pose(130.39810426540285, 71.25592417061613, Math.toRadians(0)); //reduced
    private final Pose p3c1 = new Pose(121.92417061611374, 68.77725118483411);

    private final Pose p4e = new Pose(85.1042654028436, 88.9526066350711, Math.toRadians(45));

    private final Pose p5e = new Pose(135.12322274881518, 49.914691943127956, Math.toRadians(355)); //reduced
    private final Pose p5c1 = new Pose(95.521327014218, 51.767772511848335);
    private final Pose p5c2 = new Pose(85.06398104265404, 57.6658767772512);

    private final Pose p6e = new Pose(84.09004739336493, 14.672985781990521, Math.toRadians(66));
    private final Pose p6c1 = new Pose(76.56635071090048, 57.66824644549762);

    private final Pose p7e = new Pose(135.05687203791467, 27.45497630331754, Math.toRadians(350)); //reduced
    private final Pose p7c1 = new Pose(90.47630331753555, 36.17772511848342);
    private final Pose p7c2 = new Pose(76.46208530805687, 32.15402843601896);

    private final Pose p8e = new Pose(84.09004739336493, 14.672985781990521, Math.toRadians(66));

    private PathChain driveP1, driveP2, driveP3, driveP4, driveP5, driveP6, driveP7, driveP8;

    public void buildPaths() {
        driveP1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1e))
                .setLinearHeadingInterpolation(startPose.getHeading(), p1e.getHeading())
                .build();

        driveP2 = follower.pathBuilder()
                .addPath(new BezierCurve(p1e, p2c1, p2e))
                .setLinearHeadingInterpolation(p1e.getHeading(), p2e.getHeading())
                .build();

        driveP3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2e, p3c1, p3e))
                .setLinearHeadingInterpolation(p2e.getHeading(), p3e.getHeading())
                .build();

        driveP4 = follower.pathBuilder()
                .addPath(new BezierLine(p3e, p4e))
                .setLinearHeadingInterpolation(p3e.getHeading(), p4e.getHeading())
                .build();

        driveP5 = follower.pathBuilder()
                .addPath(new BezierCurve(p4e, p5c1, p5c2, p5e))
                .setLinearHeadingInterpolation(p4e.getHeading(), p5e.getHeading())
                .build();

        driveP6 = follower.pathBuilder()
                .addPath(new BezierCurve(p5e, p6c1, p6e))
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
                // First: wait until Path1 completes
                if (!follower.isBusy()) {
                    // When we first arrive, start shooter + intake/guider and start timer
                    shooterOn();
                    allOn();

                    // Now wait 5 seconds BEFORE starting the next path
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        follower.setMaxPower(0.5);
                        follower.followPath(driveP2, true);
                        setPathState(PathState.Path2);
                    }
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
                    follower.setMaxPower(0.99);
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
                    follower.setMaxPower(0.5);
                    follower.followPath(driveP5, true);

                    setPathState(PathState.Path5);
                }
                break;

            case Path5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.99);
                    follower.followPath(driveP6, true);

                    setPathState(PathState.Path6);
                }
                break;

            case Path6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.99);
                    setPathState(PathState.Shoot3);
                }
                break;

            case Shoot3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(0.5);
                    follower.followPath(driveP7, true);

                    setPathState(PathState.Path7);
                }
                break;

            case Path7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.99);
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
                    intake.setPower(0);
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

    // ============================
    // Added public helper functions
    // ============================

    public void intakeOn() {
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void allOn() {
        guider.setPower(1);
        intake.setPower(1);
    }

    public void allOff() {
        guider.setPower(0);
        intake.setPower(0);
    }

    // Starts the reversal action; completion is handled non-blocking in loop()
    public void guiderReversal() {
        guiderReversalActive = true;
        actionTimer.resetTimer();
        guider.setPower(-1);     // reversed guider
        intake.setPower(0.25);   // intake reduced power while reversing
    }

    public void shooterOn() {
        shootRight.setPower(-0.8);
        shootLeft.setPower(0.8);
    }

    public void shooterOff() {
        shootRight.setPower(0);
        shootLeft.setPower(0);
    }

    @Override
    public void init() {
        pathState = PathState.Path1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        intake = hardwareMap.get(DcMotor.class,"intake");
        shootLeft = hardwareMap.get(DcMotor.class,"sLeft");
        shootRight = hardwareMap.get(DcMotor.class,"sRight");
        guider = hardwareMap.get(DcMotor.class,"guide");
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

        // Non-blocking completion for guiderReversal()
        if (guiderReversalActive && actionTimer.getElapsedTimeSeconds() >= 0.753) {
            guider.setPower(0);
            intake.setPower(0);
            guiderReversalActive = false;
        }

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
