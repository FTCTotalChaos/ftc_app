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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Creed: Telop", group="Creed")
//@Disabled
public class CreedTeleOp extends OpMode {

    /* Declare OpMode members. */
    public DcMotor rf;
    public DcMotor lf;
    public DcMotor rb;
    public DcMotor lb;
    public Servo rg;
    public Servo lg;
    public Servo rjk;
    public Servo ljk;
    public Servo rextention;
    public Servo lextentions;
    public DcMotor rs;
    public DcMotor ls;
    double rightposition = -0.5;
    double leftposition = 0.5;
    double rightposition2 = 0.15;
    double leftposition2 = 1;
    double position3 = 0;




    @Override
    public void init() {
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rs = hardwareMap.get(DcMotor.class, "rs");
        ls = hardwareMap.get(DcMotor.class, "ls");
        rg = hardwareMap.get(Servo.class, "rg");
        lg = hardwareMap.get(Servo.class, "lg");
        rjk = hardwareMap.get(Servo.class, "rjk");
        ljk = hardwareMap.get(Servo.class, "ljk");
        rextention = hardwareMap.get(Servo.class, "re");
        lextentions = hardwareMap.get(Servo.class, "le");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        rjk.setPosition(rightposition2);
        ljk.setPosition(leftposition2);
        lg.setPosition(leftposition);
        rg.setPosition(rightposition);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftJoy;
        double rightJoy;
        double sweeper;


        leftJoy = gamepad1.left_stick_y*0.5;
        rightJoy = gamepad1.right_stick_y*0.5;
        sweeper = gamepad2.right_trigger;
        rs.setPower(sweeper);
        ls.setPower(sweeper);

        if (Math.abs(leftJoy - rightJoy) > 0.2){
            rf.setPower(rightJoy*2);
            rb.setPower(rightJoy*2);
            lf.setPower(leftJoy*2);
            lb.setPower(leftJoy*2);
        }
        else {
            rf.setPower(rightJoy);
            rb.setPower(rightJoy);
            lf.setPower(leftJoy);
            lb.setPower(leftJoy);
        }
        if (gamepad2.right_bumper) {
            rightposition = rightposition + 0.03;
            leftposition =  leftposition + 0.03;
            rg.setPosition(rightposition);
            lg.setPosition(leftposition);
        }
        else if (gamepad2.left_bumper) {
            rightposition = rightposition - 0.03;
            leftposition =  leftposition - 0.03;
            rg.setPosition(rightposition);
            lg.setPosition(leftposition);
        }
        if (gamepad2.y) {
            rightposition2 = rightposition2 + 0.05;
            leftposition2 = leftposition2 + 0.05;
            rjk.setPosition(rightposition2 );
            ljk.setPosition(leftposition2 + 0.65);
        }
        else if (gamepad2.a) {
            rightposition2 = rightposition2 - 0.05;
            leftposition2 = leftposition2 - 0.05;
            rjk.setPosition(rightposition2);
            ljk.setPosition(leftposition2 + 0.65);
        }
        if (gamepad2.b) {
            position3 = position3 + 0.05;
            rextention.setPosition(-position3);
            lextentions.setPosition(position3);
            telemetry.addData("Right Extention Position", rextention.getPosition());
            telemetry.addData("Left Extention Position", lextentions.getPosition());
            telemetry.update();
        }
        else if (gamepad2.x) {
            position3 = position3 - 0.05;
            rextention.setPosition(-position3);
            lextentions.setPosition(position3);
            telemetry.addData("Right Extention Position", rextention.getPosition());
            telemetry.addData("Left Extention Position", lextentions.getPosition());
            telemetry.update();
        }
        // Pause for 40 mS each cycle = update 25 times a second.
    }

    @Override
    public void stop() {
    }
}
