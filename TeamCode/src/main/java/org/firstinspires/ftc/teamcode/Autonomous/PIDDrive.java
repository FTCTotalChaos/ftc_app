package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * An example linear op mode where the robot will drive in
 * a straight line (where the driving directon is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */
@TeleOp(name="AutoRed", group="SDV")
public class PIDDrive extends NewBaseAutoOp {


    @Override
    public void initSteps(){
        steps.add(new Step(10, 0.3, 0.3, RANGE, 0));
        //steps.add(new Step(3, 0.3 , 0.3, VUFORIA, 0));
        //steps.add(new Step(7, 0.1 , 0.1, MOVEARM,  0));

    }
}
