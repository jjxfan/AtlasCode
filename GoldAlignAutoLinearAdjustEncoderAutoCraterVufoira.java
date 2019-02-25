/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="CraterAutoVufoira", group="Linear Opmode")

public class GoldAlignAutoLinearAdjustEncoderAutoCraterVufoira extends LinearOpMode {

    // Declare OpMode members.
    private GoldDetector detector;
    private DcMotor left_drive = null;
    private DcMotor right_drive = null;
    private DcMotor intake_drive = null;
    private ElapsedTime runtime = new ElapsedTime();
    private TouchSensor top = null;
    private Servo Marker;
    private DcMotor linear_drive = null;
    private DigitalChannel liftSense = null;
    double goldPosition = 2;
    boolean GoldIsFound = false;
    boolean getAligned = false;
    double Pi =  3.141596;
    int WheelDiameter = 4;
    int CountsPerMotor = 1440;
    double gearReduction = 1.0;
    int INCHES = 0;
    int left_driveTicks = 1;
    int right_driveTicks = 1;
    double timeout = 10;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_drive  = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        liftSense = hardwareMap.get(DigitalChannel.class, "linearLimit");
        liftSense.setMode(DigitalChannel.Mode.INPUT);
        Marker = hardwareMap.get(Servo.class, "Marker");
        linear_drive = hardwareMap.get(DcMotor.class, "linear_drive");
        intake_drive = hardwareMap.get(DcMotor.class, "intake_drive");
        // Set up detector
        telemetry.addData("Status", "DogeCV 2019.1 - Gold & VuMark Example");
        detector = new GoldDetector(); // Create a Gold Detector

        //Sets the Vuforia license key. ALWAYS SET BEFORE INIT!
        detector.VUFORIA_KEY = "AX7Am6f/////AAABmWRXjuyewU7IpBnJX1GHbZghip4pjePEh0T7VU+pXxYWK+MFGBKrBTpUmtrdYuBMI1UyyTWcxp86B/lcB44dMEUX9iODqeP5kcqWZB4sn1yO1wpBO9QuMmaF3tsv60EgTz8KEluNceXcc8OkBbpXmQ/f6WWPxE3VEJbSFrQ52bNrJkUUzpr8D2qwKWoQ5IJ7GKTPIqg3O+/FJqEUj5eRbCJ+f9zgDKHPQmy4pldYCoLUZ0egA1Gx6+drkU6DMkfQbhJu3uSGSMtr67uSrTmKyICB61KPgyqDo0W7a+e0PiCZIVUMxnER4dXr+fGMqwhb5/jLOKuVKd4UnMRkIx6wHQ4IJNa8YSGjwhZ1kqqTjxUS";

        detector.setCAMERA_FORWARD_DISPLACEMENT(4); //eg: Camera is 4 inches in front of robot center
        detector.setCAMERA_LEFT_DISPLACEMENT(0); //eg: Camera is ON the robot's center line
        detector.setCAMERA_VERTICAL_DISPLACEMENT(6); //eg: Camera is 6 inches above the ground

        //Inits the detector. Choose which camera to use, and whether to detect VuMarks here
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), DogeCV.CameraMode.BACK, true);

        //Basic detector settings
        detector.useDefaults(); // Use default settings
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring
        detector.enable();
        detector.enableVuMarkDetection();

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());
        telemetry.update();
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        Marker.setPosition(0);
        while (runtime.seconds() < 1 && opModeIsActive()){
            telemetry.addData("Path","Marking");
        }
        INCHES = (int) Math.abs(gearReduction*CountsPerMotor/Pi*WheelDiameter);

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        left_drive.setPower(0);
        right_drive.setPower(0);
        while (liftSense.getState() && opModeIsActive()) {
            telemetry.addData("Limit Switch", "Is Not Pressed");
            linear_drive.setPower(1);
            left_drive.setPower(0);
            right_drive.setPower(0);
        }
        if (!liftSense.getState()){
            linear_drive.setPower(0);
        }

        telemetry.update();
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path2",  "Running at %7d :%7d",
                left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition());
        telemetry.update();
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setTargetPosition(500 +left_drive.getCurrentPosition());
        right_drive.setTargetPosition(-500+right_drive.getCurrentPosition());
        left_drive.setPower(1);
        right_drive.setPower(1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(-950+left_drive.getCurrentPosition());
        left_drive.setPower(1);
        runtime.reset();
        while(left_drive.isBusy()  && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        left_drive.setTargetPosition(-5280 +left_drive.getCurrentPosition());
        right_drive.setTargetPosition(5280+right_drive.getCurrentPosition());
        left_drive.setPower(1);
        right_drive.setPower(1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(970+left_drive.getCurrentPosition());
        right_drive.setTargetPosition(970+right_drive.getCurrentPosition());
        left_drive.setPower(1);
        right_drive.setPower(1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(-5100+left_drive.getCurrentPosition());
        right_drive.setTargetPosition(5100+right_drive.getCurrentPosition());
        left_drive.setPower(1);
        right_drive.setPower(1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        Marker.setPosition(1);
        runtime.reset();
        while (runtime.seconds() < 0.5  && opModeIsActive()){
            telemetry.addData("Path","Marking");
        }
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (runtime.seconds() < 0.25 && opModeIsActive()){
            telemetry.addData("Path","Reset");
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(0);
        right_drive.setTargetPosition(0);
        left_drive.setPower(-1);
        right_drive.setPower(-1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(5100);
        right_drive.setTargetPosition(-5100);
        left_drive.setPower(-1);
        right_drive.setPower(-1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(-970+left_drive.getCurrentPosition());
        right_drive.setTargetPosition(-970+right_drive.getCurrentPosition());
        left_drive.setPower(1);
        right_drive.setPower(1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setTargetPosition(5080 +left_drive.getCurrentPosition());
        right_drive.setTargetPosition(-5080+right_drive.getCurrentPosition());
        left_drive.setPower(-1);
        right_drive.setPower(-1);
        while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
            telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!detector.isFound() && opModeIsActive()){
            left_drive.setPower(-0.3);
        }
        left_drive.setPower(0);
        Rect rect = detector.getFoundRect();
        if (rect.x + rect.width*0.5 > 315 && rect.x + rect.width*0.5 < 345){
            getAligned = true;

        }else{
            getAligned = false;
        }
        while (!getAligned  && opModeIsActive()) {


            right_drive.setPower(0);
            left_drive.setPower(0);
            while (rect.x + rect.width*0.5 > 345 && rect.x + rect.width*0.5 < 639 && opModeIsActive()) {
                left_drive.setPower(-0.3);


            }
            right_drive.setPower(0);
            left_drive.setPower(0);
            while (rect.x + rect.width*0.5< 315 && rect.x + rect.width*0.5 > 0 && opModeIsActive()) {
                left_drive.setPower(0.3);
                right_drive.setPower(0);
            }
            if (rect.x + rect.width*0.5 > 315 && rect.x + rect.width*0.5 < 345){
                getAligned = true;

            }else{
                getAligned = false;
            }
        }

        right_drive.setPower(0);
        left_drive.setPower(0);
        runtime.reset();


        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setTargetPosition(100 + left_drive.getCurrentPosition());
        right_drive.setTargetPosition(100 + right_drive.getCurrentPosition());
        left_drive.setPower(0.5);
        right_drive.setPower(0.5);
        while ((left_drive.isBusy() || right_drive.isBusy()) && runtime.seconds() < 30 && opModeIsActive()) {
            telemetry.addData("LMotorPos", "%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos", "%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }



        left_drive.setTargetPosition(-3320 + left_drive.getCurrentPosition());
        right_drive.setTargetPosition(3320 + right_drive.getCurrentPosition());
        left_drive.setPower(0.5);
        right_drive.setPower(0.5);
        while ((left_drive.isBusy() || right_drive.isBusy()) && runtime.seconds() < 30 && opModeIsActive()) {
            telemetry.addData("LMotorPos", "%7d", left_drive.getCurrentPosition());
            telemetry.addData("RMotorPos", "%7d", right_drive.getCurrentPosition());
            telemetry.update();
        }

        left_drive.setPower(0);
        right_drive.setPower(0);


        /*runtime.reset();
        detector.enable();

        while ((runtime.seconds() < 1) && opModeIsActive()) {
            telemetry.addData("Path","DetectorEnable");
            right_drive.setPower(0);
            left_drive.setPower(0);

        }
        if (detector.isFound()){
            GoldIsFound = true;
        }

        if (detector.getXPosition() > 320 && detector.isFound()) {
            goldPosition = 2;
            telemetry.addData("goldPosition", "2");

        } else if (detector.getXPosition() < 319 && detector.isFound()) {
            goldPosition = 1;
            telemetry.addData("goldPosition", "1");

        } else if (!detector.isFound()) {
            goldPosition = 3;
            telemetry.addData("goldPosition", "3");
        }
        telemetry.update();

        if (detector.getXPosition() > 320 && detector.isFound()) {
            goldPosition = 2;
            telemetry.addData("goldPosition", "2");

        } else if (detector.getXPosition() < 319 && detector.isFound()) {
            goldPosition = 1;
            telemetry.addData("goldPosition", "1");

        } else if (!detector.isFound()) {
            goldPosition = 3;
            telemetry.addData("goldPosition", "3");
        }
        telemetry.update();
        telemetry.update();
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!detector.isFound() && opModeIsActive()){
            left_drive.setPower(-0.3);
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        while  (detector.isFound() && detector.getXPosition() < 305 && detector.getXPosition() > 0 && opModeIsActive() ) {
            left_drive.setPower(0.3);
            right_drive.setPower(0);
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        while (detector.isFound() && detector.getXPosition() > 335 && detector.getXPosition() < 639 && opModeIsActive() ) {
            left_drive.setPower(-0.3);


        }
        right_drive.setPower(0);
        left_drive.setPower(0);


        runtime.reset();


        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (detector.getAligned() && goldPosition == 1) {
            left_drive.setTargetPosition(-4320 + left_drive.getCurrentPosition());
            right_drive.setTargetPosition(4320 + right_drive.getCurrentPosition());
            left_drive.setPower(0.5);
            right_drive.setPower(0.5);
            while((left_drive.isBusy() || right_drive.isBusy()) && runtime.seconds() < 30 && opModeIsActive()) {
                telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }

        } else if (detector.getAligned() && goldPosition == 2) {
            left_drive.setTargetPosition(-4320 + left_drive.getCurrentPosition());
            right_drive.setTargetPosition(4320 + right_drive.getCurrentPosition());
            left_drive.setPower(0.5);
            right_drive.setPower(0.5);
            while((left_drive.isBusy() || right_drive.isBusy()) && runtime.seconds() < 30 && opModeIsActive()) {
                telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }

        } else if (detector.getAligned() && goldPosition == 3) {
            left_drive.setTargetPosition(-3500 + left_drive.getCurrentPosition());
            right_drive.setTargetPosition(3500 + right_drive.getCurrentPosition());
            left_drive.setPower(0.5);
            right_drive.setPower(0.5);
            while((left_drive.isBusy() || right_drive.isBusy()) && runtime.seconds() < 30 && opModeIsActive()) {
                telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }

        }
        if(goldPosition == 1) {
            telemetry.addData("goldPosition", "1");
        } else if(goldPosition == 2) {
            telemetry.addData("goldPosition", "2");
        } else if(goldPosition == 3) {
            telemetry.addData("goldPosition", "3");
        }


        if (opModeIsActive() && goldPosition == 1 ){
            left_drive.setTargetPosition(-1440+left_drive.getCurrentPosition());
            left_drive.setPower(0.4);
            while(left_drive.isBusy() &&  runtime.seconds() < 10 && opModeIsActive()) {
                telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }

        }


        if ( opModeIsActive() && goldPosition == 3) {

            right_drive.setTargetPosition(1900+right_drive.getCurrentPosition());
            right_drive.setPower(-0.4);
            while(right_drive.isBusy()  && runtime.seconds() < 10 && opModeIsActive()) {
                telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }

        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        runtime.reset();
        if (goldPosition != 2 &&  opModeIsActive()) {
            left_drive.setPower(-1);
            right_drive.setPower(1);
            left_drive.setTargetPosition(-1440 + left_drive.getCurrentPosition());
            right_drive.setTargetPosition(1440 + right_drive.getCurrentPosition());
            while (left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
                telemetry.addData("LMotorPos", "%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos", "%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        runtime.reset();
        if (goldPosition == 2 && opModeIsActive()){
            left_drive.setTargetPosition(-720+left_drive.getCurrentPosition());
            right_drive.setTargetPosition(720+right_drive.getCurrentPosition());
            left_drive.setPower(-0.4);
            right_drive.setPower(0.4);
            while(left_drive.isBusy() && right_drive.isBusy() && runtime.seconds() < 10 && opModeIsActive()) {
                telemetry.addData("LMotorPos","%7d", left_drive.getCurrentPosition());
                telemetry.addData("RMotorPos","%7d", right_drive.getCurrentPosition());
                telemetry.update();
            }
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        runtime.reset();
        Marker.setPosition(1);
        while (runtime.seconds() < 1 && opModeIsActive()){
            telemetry.addData("Path","Marking");
        }

        telemetry.update();
        */
        detector.disable();



    }
}
