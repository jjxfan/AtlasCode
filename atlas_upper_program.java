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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="TeleopDrive", group="Linear Opmode")
//@Disabled
public class atlas_upper_program extends LinearOpMode {

    // Declare OpMode members.
    private DigitalChannel liftSense;
    private ElapsedTime runtime = new ElapsedTime();
    private atlas_under_program robot = new atlas_under_program();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        liftSense = hardwareMap.get(DigitalChannel.class, "linearLimit");
        liftSense.setMode(DigitalChannel.Mode.INPUT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
       /*robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
       robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
       robot.depositDrive.setDirection(DcMotor.Direction.FORWARD);*/
        robot.init(hardwareMap);
        double clawOffset = 0;                       // Servo mid position
        final double CLAW_SPEED = 0.02;
        double leftPower;
        double rightPower;
        double intakePower;
        double linearPower;
        double drive = 0;
        double turn = 0;
        double intake = 0;
        double depositUp = 0;
        double depositDown = 0;
        double linearDrive = 0;
        double depositPower = 0;
        double depositServoPower = 0;
        boolean slowCheck = false;
        double leftSlow;
        double rightSlow;
        double slowDrive;
        double slowTurn;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            drive = gamepad1.left_stick_y;
            turn = -gamepad1.left_stick_x;
            slowDrive = drive / 2.0;
            slowTurn = turn / 2.0;

            if (gamepad1.right_bumper) {
                intake = 1.0;

            } else if (gamepad1.left_bumper) {
                intake = -1.0;

            } else {
                intake = 0;
            }

            if (gamepad1.dpad_right)
                clawOffset += CLAW_SPEED;
            else
                clawOffset = 0;


            if (gamepad1.right_stick_y > 0.1)
                depositUp += robot.DEPOSIT_UP_POWER;

            if (gamepad1.right_stick_y < -0.1)
                depositDown += robot.DEPOSIT_DOWN_POWER;

            if (gamepad1.dpad_up && gamepad1.a && liftSense.getState() == true) {
                linearDrive = 1.0;
            } else {
                linearDrive = 0;
            }

            if (gamepad1.dpad_down && gamepad1.a) {
                linearDrive = -1.0;
            }

            if (gamepad1.dpad_left) {
                linearDrive = 0.0;
            }
            if (gamepad1.b) {
                slowCheck = true;
            } else {
                slowCheck = false;

            }
            if (slowDrive > 0.3) {
                slowDrive = 0.3;
            } else if (slowTurn < -0.3){
                slowTurn = -0.3;
            }


                depositUp = Range.clip(depositUp, -0.75, 0.75);
                depositDown = Range.clip(depositDown, -0.75, 0.75);
                clawOffset = Range.clip(clawOffset, -0.5, 0.5);
                intakePower = Range.clip(intake, -0.65, 0.65);
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
                linearPower = Range.clip(linearDrive, -1.0, 1.0);
                leftSlow    = Range.clip(slowDrive + slowTurn, -0.5, 0.5);
                rightSlow    = Range.clip(slowDrive - slowTurn, -0.5, 0.5);

                depositPower = depositUp - depositDown;
                depositServoPower = robot.MID_SERVO + clawOffset;
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            if (slowCheck) {
                robot.leftDrive.setPower(leftSlow);
                robot.rightDrive.setPower(rightSlow);
            } else {
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }
            robot.intakeDrive.setPower(intakePower);
            robot.linearServo.setPosition(depositServoPower);
            //robot.depositDrive.setPower(depositPower);
            robot.linearDrive.setPower(linearPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), deposit (%.2f), intake (%.2f), linear (%.2f)", leftPower, rightPower, depositPower, intakePower, depositServoPower);
            telemetry.update();

        }
    }
}
