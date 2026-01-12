package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagAuto extends OpMode {

    AprilTagWebcamSetup aprilTagWebcam = new AprilTagWebcamSetup();
    Integer detectedTagID = null;

    @Override
    public void init() {

        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addLine("Camera initialized. Scanning for AprilTags...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Continuously scan for tags while waiting to start
        aprilTagWebcam.update();
        for (AprilTagDetection tag : aprilTagWebcam.getDetectedTags()) {
            // Save the first detected tag ID
            detectedTagID = tag.id;
            telemetry.addData("Detected Tag ID", detectedTagID);
            break; // Stop after detecting one tag
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // Use the saved tag ID during autonomous
        if (detectedTagID != null) {
            telemetry.addData("Saved Tag ID", detectedTagID);
        } else {
            telemetry.addLine("No tag detected yet.");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
