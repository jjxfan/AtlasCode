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

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Lawson Drive2")

public class Drive_Lawson extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;
    private DcMotor left_drive = null;
    private DcMotor right_drive = null;
    private DcMotor intake_drive = null;
    private TouchSensor top = null;
    private Servo Marker;
    private DcMotor linear_drive = null;
    private DigitalChannel liftSense = null;
    private Servo arm_servo;
    double drive = 0.0;
    boolean slowCheck = false;
    double leftSlow;
    double rightSlow;
    double turn = 0.0;
    double slowDrive;
    double slowTurn;
    double leftPower;
    double rightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_drive  = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        liftSense = hardwareMap.get(DigitalChannel.class, "linearLimit");
        liftSense.setMode(DigitalChannel.Mode.INPUT);
        linear_drive = hardwareMap.get(DcMotor.class, "linear_drive");
        intake_drive = hardwareMap.get(DcMotor.class, "intake_drive");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        intake_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //telemetry.addData("Xpos", gamepad1.left_stick_y);
        //telemetry.update();
        if(gamepad1.left_stick_y < 0.15 && gamepad1.left_stick_y > -0.15 ){
            drive = 0.0;
        } else {
            drive = Math.pow(gamepad1.left_stick_y, 3) + 0.15;
        }

        turn = Math.pow(-gamepad1.left_stick_x, 3) ;
        slowDrive = drive / 2.0;
        slowTurn = turn / 2.0;
        if (gamepad1.dpad_up && gamepad1.a && liftSense.getState() == true) {
            linear_drive.setPower(1.0);

        } else if (gamepad1.dpad_down && gamepad1.a) {
            linear_drive.setPower(-1.0);
        } else {
            linear_drive.setPower(0.0);
        }
        slowCheck = gamepad1.b;

        if (slowDrive > 0.3) {
            slowDrive = 0.3;
        } else if (slowTurn < -0.3){
            slowTurn = -0.3;
        }
        if (gamepad1.y){
            arm_servo.setPosition(0.95);
        } else if (gamepad1.a) {
            arm_servo.setPosition(0.0);
        } else if(gamepad1.x && !gamepad1.dpad_down && !gamepad1.dpad_up){
            arm_servo.setPosition(0.6);
        }

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftSlow    = Range.clip(slowDrive + slowTurn, -0.5, 0.5);
        rightSlow    = Range.clip(slowDrive - slowTurn, -0.5, 0.5);

        // Send calculated power to wheels
        if (slowCheck) {
            left_drive.setPower(leftSlow);
            right_drive.setPower(rightSlow);
        } else {
            left_drive.setPower(leftPower);
            right_drive.setPower(rightPower);
        }
        intake_drive.setPower(gamepad1.right_stick_y / 2.0);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        left_drive.setPower(0.0);
        right_drive.setPower(0.0);
        intake_drive.setPower(0.0);
        linear_drive.setPower(0.0);
    }

}