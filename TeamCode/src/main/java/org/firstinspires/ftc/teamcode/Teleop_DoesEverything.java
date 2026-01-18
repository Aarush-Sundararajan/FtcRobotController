package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.prefs.BackingStoreException;

@TeleOp(name="Teleop_DoesEverything", group="Linear OpMode")
public class Teleop_DoesEverything extends LinearOpMode {

    // Replaced IMU with Pinpoint as the angle source
    private GoBildaPinpointDriver pinpoint;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shootLeft = null;
    private DcMotor shootRight = null;
    private DcMotor intake = null;
    private Servo s1 = null;
    //Sorter
    private CRServo s2 = null;
    //Left guiding servo on bottom
    private Servo s3 = null;
    //linear servo for ramp
    private CRServo s4 = null;
    //right guiding servo on bottom
    private CRServo s5 = null;
    //left guiding servo on top (1)
    private CRServo s6 = null;
    //right guiding servo on top (-1)

    double left1Y, right1Y, left1X, right1X;
    double left2Y, right2Y, left2X, right2X;
    boolean flag_correction = true;
    double m2Power, blPower, flPower, brPower, frPower;
    static final double DEADZONE = 0.1;

    private double clampDeadzone(double val) {
        if (Math.abs(val) < DEADZONE) {
            return 0;
        } else {
            return val;
        }
    }

    @Override
    public void runOpMode() {

        // Initialize Pinpoint (configured on I2C Bus 0 as "pinpoint")
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        telemetry.addData("Gyro Status", "Pinpoint Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");
        intake = hardwareMap.get(DcMotor.class,"intake");
        shootLeft = hardwareMap.get(DcMotor.class,"sLeft");
        shootRight = hardwareMap.get(DcMotor.class,"sRight");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(CRServo.class, "s4");
        s5 = hardwareMap.get(CRServo.class,"s5");
        s6 = hardwareMap.get(CRServo.class,"s6");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        boolean gay = false;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            left1Y = this.clampDeadzone(gamepad1.left_stick_y * -1);
            left1X = this.clampDeadzone(gamepad1.left_stick_x);
            right1Y = this.clampDeadzone(gamepad1.right_stick_y * -1);
            right1X = this.clampDeadzone(gamepad1.right_stick_x);

            left2Y = this.clampDeadzone(gamepad2.left_stick_y * -1);
            left2X = this.clampDeadzone(gamepad2.left_stick_x);
            right2Y = this.clampDeadzone(gamepad2.right_stick_y * -1);
            right2X = this.clampDeadzone(gamepad2.right_stick_x);

            shootLeft.setPower(0);
            shootRight.setPower(0);
            intake.setPower(0);
            s2.setPower(0);

            if(gamepad2.right_bumper){
                s1.setDirection(Servo.Direction.FORWARD);
                s1.setPosition(0.5);
            }
            if(gamepad2.left_bumper){
                s1.setDirection(Servo.Direction.REVERSE);
                s1.setPosition(-0.5);
            }

            if (gamepad2.x){
                intake.setPower(-1);
            }else{intake.setPower(0);}
            if (gamepad2.y){
                intake.setPower(1);
            }else {intake.setPower(0);}

            if (gamepad2.a){
                shootRight.setPower(0.8);
                shootLeft.setPower(-0.8);
            }else{
                shootRight.setPower(0);
                shootLeft.setPower(0);
            }

            if (gamepad2.b){
                shootRight.setPower(-1);
                shootLeft.setPower(1);
            }else{
                shootRight.setPower(0);
                shootLeft.setPower(0);
            }

            if(gamepad2.dpad_up){
                frontLeftDrive.setPower(0.3);
                frontRightDrive.setPower(-0.3);
                backRightDrive.setPower(-0.3);
                backLeftDrive.setPower(0.3);
            }

            if(gamepad2.dpad_down){
                frontLeftDrive.setPower(0.3);
                frontRightDrive.setPower(0.3);
                backRightDrive.setPower(0.3);
                backLeftDrive.setPower(0.3);
            }

            if(gamepad2.right_trigger > 0.1) {
                s3.setPosition(0.5);
            }

            if(gamepad2.left_trigger > 0.1){
                s3.setPosition(0.7);
            }

            //movement
            boolean leftStickActive = (left2X != 0) || (left2Y != 0);
            boolean rightStickActive = right2X != 0;

            //Forwards/Backward for gamepad 1
            if (leftStickActive == rightStickActive) {
                flPower = 0;
                frPower = 0;
                brPower = 0;
                blPower = 0;

            } else if (leftStickActive) {

                double THRESH_WM_POWER = 0.8; // max abs wheel power

                // Replaced IMU orientation read with Pinpoint yaw
                double headingDeg = pinpoint.getYawScalar(); // degrees
                double correction = headingDeg / 180.0;

                correction = (10.0 * correction * Math.abs(left2Y) / THRESH_WM_POWER);
                if (flag_correction == false) {
                    correction = 0;
                }
                correction = 0;

                double maxPow = THRESH_WM_POWER;
                double flPow = left2Y + (-left2X) + correction;
                maxPow = Math.max(maxPow, Math.abs(flPow));
                double blPow = left2Y - (-left2X) + correction;
                maxPow = Math.max(maxPow, Math.abs(blPow));
                double frPow = left2Y - (-left2X) - correction;
                maxPow = Math.max(maxPow, Math.abs(frPow));
                double brPow = left2Y + (-left2X) - correction;
                maxPow = Math.max(maxPow, Math.abs(brPow));
                flPow = (flPow / maxPow) * THRESH_WM_POWER;
                blPow = (blPow / maxPow) * THRESH_WM_POWER;
                frPow = (frPow / maxPow) * THRESH_WM_POWER;
                brPow = (brPow / maxPow) * THRESH_WM_POWER;

                flPower = (Range.clip(flPow, -THRESH_WM_POWER, THRESH_WM_POWER));
                blPower = (Range.clip(blPow, -THRESH_WM_POWER, THRESH_WM_POWER));
                frPower = (Range.clip(frPow, -THRESH_WM_POWER, THRESH_WM_POWER));
                brPower = (Range.clip(brPow, -THRESH_WM_POWER, THRESH_WM_POWER));

                telemetry.addData("third Angle", headingDeg);
                telemetry.addData("correction", correction);
                telemetry.addData("leftY", left1Y);
                telemetry.addData("leftX", left1X);
                telemetry.addData("flPow", flPow);
                telemetry.addData("blPow", blPow);
                telemetry.addData("frPow", frPow);
                telemetry.addData("brPow", brPow);
                telemetry.addData("fl Enc Count", frontLeftDrive.getCurrentPosition());
                telemetry.addData("bl Enc Count", backLeftDrive.getCurrentPosition());
                telemetry.addData("fr Enc Count", frontRightDrive.getCurrentPosition());
                telemetry.addData("br Enc Count", backRightDrive.getCurrentPosition());
                telemetry.update();
                telemetry.update();

            } else {
                //rightstick active
                double THRESH_WM_POWER_FORTURN = 0.8;
                flPower = (Range.clip((-right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                blPower = (Range.clip((-right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                frPower = (Range.clip((-right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                brPower = (Range.clip(-(-right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));

                // Replaced IMU orientation read with Pinpoint yaw
                double headingDeg = pinpoint.getYawScalar();
                telemetry.addData("THIS IS THE ANGLE", headingDeg);
                telemetry.update();
                //imu.resetYaw();
                idle();
            }

            backLeftDrive.setPower(blPower);
            backRightDrive.setPower(brPower);
            frontLeftDrive.setPower(flPower);
            frontRightDrive.setPower(frPower);

        }
    }
}
