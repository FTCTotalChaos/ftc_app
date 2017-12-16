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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Ghost", group = "Ghost")
public class MechWheelsOp extends OpMode {

    double armDelta = 0.01;
    boolean iSawDpadUpAlready = false;
    boolean iSawDpadDownAlready = false;
    boolean iSawDpadUpAlready2 = false;
    boolean iSawDpadDownAlready2 = false;
    public DcMotor rf;
    public DcMotor lf;
    public DcMotor rb;
    public DcMotor lb;
    public Servo rg;
    public Servo lg;
    public Servo rjk;
    public Servo ljk;
    public CRServo rsg;
    public CRServo lsg;
    public CRServo Slidy;
    public CRServo rs;
    public CRServo ls;
    //public Servo rextention;
    //public Servo lextentions;
    public DcMotor up;
    double rightposition = 0.575;
    double leftposition = 0.39;
    double rightposition2 = 0.694;
    double leftposition2 = 0.64;
    double rightposition3 = 0.5;
    double leftposition3 = 0.5;
    final static double FAST = 1.0;
    final static double MED_FAST = 0.75;
    final static double MEDIUM = 0.5;
    final static double SLOW = 0.25;
    double armMode = MEDIUM;
    double mode = FAST;
    double upPos = 0;

    double flapPosition = 1;

    public void init()
    {
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        up = hardwareMap.get(DcMotor.class, "up");
        rg = hardwareMap.get(Servo.class, "rg");
        lg = hardwareMap.get(Servo.class, "lg");
        rjk = hardwareMap.get(Servo.class, "rjk");
        ljk = hardwareMap.get(Servo.class, "ljk");
        Slidy = hardwareMap.get(CRServo.class, "sc");
        rsg = hardwareMap.get(CRServo.class, "rsg");
        lsg = hardwareMap.get(CRServo.class, "lsg");
        rs = hardwareMap.get(CRServo.class, "rs");
        ls = hardwareMap.get(CRServo.class, "ls");
        //rextention = hardwareMap.get(Servo.class, "re");
        //lextentions = hardwareMap.get(Servo.class, "le");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        up.setDirection(DcMotorSimple.Direction.REVERSE);
        rjk.setPosition(rightposition2);
        ljk.setPosition(leftposition2);
        lg.setPosition(leftposition);
        rg.setPosition(rightposition);
        rsg.setPower(-0.01);
        lsg.setPower(0);
        Slidy.setPower(0.5);
        ls.setPower(0);
        rs.setPower(0);
    }
    @Override
    public void start(){

    }
    @Override
    public void loop()
    {
        // When dpad is pushed up increase one mode
        //When dpad is pushed down decrease by one mode
        if (gamepad1.dpad_up) {
            if(!iSawDpadUpAlready) {
                iSawDpadUpAlready = true;
                mode = mode + 0.25;
            }
        }
        else {
            iSawDpadUpAlready = false;
        }

        if (gamepad1.dpad_down) {
            if(!iSawDpadDownAlready) {
                iSawDpadDownAlready = true;
                mode = mode - 0.25;
            }
        }
        else {
            iSawDpadDownAlready = false;
        }
        mode = Range.clip(mode, 0.25, 0.75 );


        double forward = gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (side == 0 || forward == 0 || turn == 0) {
            if (Math.abs(forward) > Math.abs(side)) {
                lf.setPower(forward*mode);
                lb.setPower(forward*mode);
                rf.setPower(forward*mode);
                rb.setPower(forward*mode);
            } else if (Math.abs(side) > Math.abs(forward)) {
                rf.setPower(side*mode);
                lf.setPower(-side*mode);
                rb.setPower(-side*mode);
                lb.setPower(side*mode);
            } else if (turn > 0) {
                lf.setPower(-turn*mode);
                lb.setPower(-turn*mode);
                rb.setPower(turn*mode);
                rf.setPower(turn*mode);
            } else if (turn < 0) {
                lf.setPower(-turn*mode);
                lb.setPower(-turn*mode);
                rb.setPower(turn*mode);
                rf.setPower(turn*mode);
            } else if (side == 0 && forward == 0 && turn == 0) {
                lb.setPower(0);
                lf.setPower(0);
                rb.setPower(0);
                rf.setPower(0);
            }
        }
        if (gamepad2.y){
            rs.setPower(1);
            ls.setPower(-1);
        }
        if (gamepad2.a){
            rs.setPower(-1);
            ls.setPower(1);
        }
        if (gamepad2.x){
            rs.setPower(0);
            ls.setPower(0);
        }
        if (gamepad2.right_bumper) {
            rg.setPosition(0.15);
            lg.setPosition(0.77);
        }
        else if (gamepad2.left_bumper) {
            rg.setPosition(0.575);
            lg.setPosition(0.39);
        }
        if (gamepad2.right_trigger > 0){
            rsg.setPower(0.5);
            lsg.setPower(-0.5);
        }
        else if (gamepad2.left_trigger > 0){
            rsg.setPower(-0.5);
            lsg.setPower(0.5);
        }
        else{
            rsg.setPower(0);
            lsg.setPower(0);
        }
        if (gamepad2.right_stick_x>0){
            Slidy.setPower(1); 
            telemetry.addData("Slidy Power", Slidy.getPower());
            telemetry.update();
        }
        else if (gamepad2.right_stick_x < 0){
            Slidy.setPower(0);
            telemetry.addData("Slidy Power", Slidy.getPower());
            telemetry.update();
        }
        else {
            Slidy.setPower(0.5);
        }
        upPos = gamepad2.left_stick_y;
        up.setPower(upPos);


    }



       /*
       //q1
       if (side > 0 && forward >  0) {
           leftFront.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           rightBack.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           leftBack.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
           rightFront.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
       } else if (side == 0 && forward == 0) {
           leftBack.setPower(0);
           leftFront.setPower(0);
           rightBack.setPower(0);
           rightFront.setPower(0);

       }
       //q4
       else if (side < 0 && forward > 0) {
           leftBack.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           rightFront.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           leftFront.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
           rightBack.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
       }//q2
       else
       if (side > 0 && forward < 0) {
           leftBack.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           rightFront.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           leftFront.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
           leftBack.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
       } else if (side == 0 && forward == 0) {
           leftBack.setPower(0);
           leftFront.setPower(0);
           rightBack.setPower(0);
           rightFront.setPower(0);
       }
       //q3
       if (side < 0 && forward < 0) {
           leftFront.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           rightBack.setPower(
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))));
           leftBack.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
           rightFront.setPower((
                   (Math.sqrt(
                           (gamepad1.left_stick_x * gamepad1.left_stick_x
                                   + gamepad1.left_stick_y * gamepad1.left_stick_y))) *
                           Math.tanh((gamepad1.left_stick_y / gamepad1.left_stick_x) - 45)) / 45);
       } else if (side == 0 && forward == 0) {
           leftBack.setPower(0);
           leftFront.setPower(0);
           rightBack.setPower(0);
           rightFront.setPower(0);
       }
   }
       /*
       right = (double)scaleInput(right);
       left =  (double)scaleInput(left);

       right= Range.clip(right, -mode, mode);
       left= Range.clip(left, -mode, mode);


       leftFront.setPower(left);
       leftBack.setPower(left);
       rightFront.setPower(right);
       rightBack.setPower(right);

   }
*/

    @Override
    public void stop()
    {
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale * mode;
    }
}

