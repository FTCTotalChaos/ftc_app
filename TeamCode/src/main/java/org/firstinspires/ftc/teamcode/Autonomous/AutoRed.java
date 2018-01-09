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
        steps.add(new Step(3, 0.1 , 0.1, MOVEARM, 0, RED));
        if (colorVal == RED){
            steps.add(new Step(-26, -0.1 , -0.1, MOVE, 0, RED));
        }
        else if (colorVal == BLUE){
            steps.add(new Step(-32, -0.1 , -0.1, MOVE, 0, RED));
        }
        else{
            steps.add(new Step(-29, -0.1 , -0.1, MOVE, 0, RED));
        }
        steps.add(new Step(0, 0.1, 0.1, RIGHT, 90, RED));
        steps.add(new Step(-4, -0.1 , -0.1, MOVE, 0, RED));
        steps.add(new Step(-26, -0.1 , -0.1, DROPBLOCK, 0, RED));


    }
}
