package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name="Red", group="Ghost")
public class AutoRed extends MechBaseAutoOp {


    @Override
    public void initSteps(){
        steps.add(new Step(0, 0 , 0, VUFORIA, 0, RED));
        steps.add(new Step(0, 0 , 0, RAISEBLOCK, 0, RED));
        steps.add(new Step(1.2, 0.1 , 0.1, MOVEARM, 0, RED));
        steps.add(new Step(-32, -0.1 , -0.1, VUMOVE, 0, RED));
        steps.add(new Step(0, 0.1, 0.1, RIGHT, 90, RED));
        steps.add(new Step(-5, -0.1 , -0.1, MOVE, 0, RED));
        steps.add(new Step(-26, -0.1 , -0.1, DROPBLOCK, 0, RED));
        steps.add(new Step(7, 0.1 , 0.1, MOVE, 0, RED));
        steps.add(new Step(-11, -0.1 , -0.1, MOVE, 0, RED));
        steps.add(new Step(3, 0.1 , 0.1, MOVE, 0, RED));





    }
}
