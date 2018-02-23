package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.opengl.models.MeshObject;
import org.firstinspires.ftc.teamcode.MechWheelsOp;

/*
 * An example linear op mode where the robot will drive in
 * a straight line (where the driving directon is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller conf,,iguration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */
@Autonomous(name="Blue", group="Ghost")
public class AutoBlue extends MechBaseAutoOp {


    @Override
    public void initSteps(){
        steps.add(new Step(0, 0 , 0, StepType.GRABBLOCK, 0, BLUE));
        steps.add(new Step(0, 0 , 0, StepType.VUFORIA, 0, BLUE));
        steps.add(new Step(0, 0 , 0, StepType.RAISEBLOCK, 0, BLUE));
        steps.add(new Step(1.2, 0.1 , 0.1, StepType.MOVEARM, 0, BLUE));
        steps.add(new Step(-19, -0.1 , -0.1, StepType.VUMOVE, 0, BLUE));
        steps.add(new Step(-5, 0.1, 0.1, StepType.GOTUTOUCH, 40, BLUE));
        steps.add(new Step(0, 0.075, 0.075, StepType.LEFT, 75, BLUE));
        steps.add(new Step(0, 0 , 0, StepType.LOWBLOCK, 0, BLUE));
        steps.add(new Step(-26, -0.1 , -0.1, StepType.BLOCK, 0, BLUE));
        steps.add(new Step(-6, -0.1 , -0.1, StepType.MOVE, 0, BLUE));
        steps.add(new Step(4, 0.1 , 0.1, StepType.MOVE, 0, BLUE));



    }
}
