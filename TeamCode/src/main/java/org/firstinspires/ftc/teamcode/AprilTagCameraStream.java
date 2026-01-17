package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AprilTagWebcamSetup;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagCameraStream extends OpMode {

    AprilTagWebcamSetup aprilTagWebcam = new AprilTagWebcamSetup();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificID(21);
        //aprilTagWebcam.displayDetectionTelemetry(id21);
        telemetry.addData("id21 String", id21.toString());
    }
}
