package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.util.Vector;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public abstract class MechBaseAutoOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor up;
    Servo rjk;
    Servo ljk;
    Servo rjr;
    Servo ljr;
    Servo rextention;
    Servo lextentions;
    Servo rg;
    Servo lg;
    Servo trg;
    Servo tlg;
    Servo re;
    Servo le;
    double rightposition = 0.86;
    double leftposition = 0.08;
    double rightposition2 = 0;
    double leftposition2 = 0.8;
    double rightposition3 = 0.61;
    double leftposition3 = 0.79;
    double rightposition4 = 0.3;
    double leftposition4 = 0.77;
    double positionRed = rightposition3;
    double positionBlue = leftposition3;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;
    private ElapsedTime runtime = new ElapsedTime();
    //navx values
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    ColorSensor colorBlue;
    ColorSensor colorRed;
    ColorSensor colorBlue2;
    ColorSensor colorRed2;
    TouchSensor touchBlue;
    TouchSensor touchRed;
    boolean hasTouched;
    boolean hasStopped = false;
    Vector<Step> steps;
    Step currentStep;
    int counts = 0;
    int currentStepIndex;
    final static int ATREST = 0;
    final static int WAITFORRESETENCODERS = 1;
    final static int WAITFORCOUNTS = 2;
    final static int FINISHED = 3;
    final static int DETECTCOLOR = 4;
    final static int WAITFORTURN = 5;
    final static int TURNTOANGLE = 6;
    final static int JEWELKNOCK = 7;
    final static int VUCHECK = 8;
    final static int TURNARM = 9;
    final static int WAITFORSIDE = 10;
    final static int DROPBLOCK = 11;
    final static int RAISE = 12;
    final static int SPECJKVU = 13;
    final static int SPECJK = 14;
    final static int SPECVU = 15;
    final static int SPECTURN = 16;
    final static int WAITFORTOUCH = 17;
    final static int NEWSPECTURN = 19;
    final static int GRAB = 18;
    final static int LOWER = 20;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 0.5;
    final static double WHEEL_DIAMETER = 4;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    //Start step types
    public enum StepType {
        MOVE,
        RIGHT,
        LEFT ,
        RANGE,
        BACK,
        WAIT,
        MOVEARM,
        BLOCK,
        BACKWARD,
        NONE,
        VUFORIA,
        SIDERIGHT,
        SIDELEFT,
        RAISEBLOCK,
        VUJKMOVE,
        JKMOVE,
        VUMOVE,
        VUTURN,
        GRABBLOCK,
        GOTUTOUCH,
        NEWVUTURN,
        LOWBLOCK
    };
    boolean hasinit = false;
    final static int BLOCKN = 0;
    final static int BLOCKC = 1;
    final static int BLOCKL = 2;
    final static int BLOCKR = 3;
    int block = BLOCKN;
    int counter = 0;
    int counter2 = 0;
    int counter3 = 0;
    double startYaw = 0;
    final static int BLUE = 0;
    final static int RED = 1;
    final static int NOCOLOR = 2;
    int colorVal = NOCOLOR;
    private boolean calibration_complete = false;

    public class Step {
        public double distance;
        public double leftFrontPower;
        public double rightFrontPower;
        public double leftBackPower;
        public double rightBackPower;
        public double rightFrontCounts;
        public double leftFrontCounts;
        public double rightBackCounts;
        public double leftBackCounts;
        public int sweeperDirection;
        public double armPosition;
        public double turnAngle;
        public StepType sType;
        public int colorType;

        public Step(double dist, double left, double right, StepType stepType, int angle, int col) {
            distance = dist;
            sType = stepType;
            colorType = col;
            if (stepType == StepType.MOVE || stepType == StepType.GOTUTOUCH) {
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = right;
            } else if (stepType == StepType.LEFT || (stepType == StepType.VUTURN && col == RED) || (stepType == StepType.NEWVUTURN && col == BLUE)) {
                turnAngle = -angle;
                leftFrontPower = left;
                rightFrontPower = -right;
                leftBackPower = left;
                rightBackPower = -right;

            } else if (stepType == StepType.RIGHT || (stepType == StepType.VUTURN && col == BLUE) || (stepType == StepType.NEWVUTURN && col == RED)) {
                turnAngle = angle;
                leftFrontPower = -left;
                rightFrontPower = right;
                leftBackPower = -left;
                rightBackPower = right;
            } else if (stepType == StepType.BACK) {
                rightFrontCounts = -(convertDistance(distance));
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = -left;
                rightFrontPower = -right;
                leftBackPower = -left;
                rightBackPower = -right;
            } else if (stepType == StepType.SIDELEFT) {
                turnAngle = angle;
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = -rightFrontCounts;
                rightBackCounts = -rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = -left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = -right;
            } else if (stepType == StepType.SIDERIGHT) {
                rightFrontCounts = -(convertDistance(distance));
                leftFrontCounts = -rightFrontCounts;
                rightBackCounts = -rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = -right;
                leftBackPower = -left;
                rightBackPower = right;
            } else if (stepType == StepType.WAIT) {
                leftFrontPower = 0;
                rightFrontPower = 0;
                leftBackPower = 0;
                rightBackPower = 0;
            } else if (stepType == StepType.MOVEARM) {
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = right;
            }
            else if (stepType == StepType.VUJKMOVE || stepType == StepType.VUMOVE ||stepType == StepType.JKMOVE) {
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = right;
            }
            else {
                armPosition = dist;
            }
        }
    }

    public static int convertDistance(double distance) {
        double rotations = distance / CIRCUMFERENCE;
        double counts = ENCODER_CPR * rotations * GEAR_RATIO;
        return (int) counts;
    }

    public void initializeNavX(double angle) {
        yawPIDController.setSetpoint(angle);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        navx_device.zeroYaw();

    }

    public void init() {
        if (!hasinit) {
            hasinit = true;
            leftFront = hardwareMap.get(DcMotor.class, "lf");
            rightFront = hardwareMap.get(DcMotor.class, "rf");
            leftBack = hardwareMap.get(DcMotor.class, "lb");
            rightBack = hardwareMap.get(DcMotor.class, "rb");
            up = hardwareMap.get(DcMotor.class, "up");
            colorBlue = hardwareMap.get(ColorSensor.class, "cb");
            colorRed = hardwareMap.get(ColorSensor.class, "cr");
            colorRed.setI2cAddress(I2cAddr.create8bit(0x4c) );
            colorBlue.setI2cAddress(I2cAddr.create8bit(0x3c) );
            rjr = hardwareMap.get(Servo.class, "rjr");
            ljr = hardwareMap.get(Servo.class, "ljr");
            touchRed = hardwareMap.get(TouchSensor.class, "tr");
            touchBlue = hardwareMap.get(TouchSensor.class, "tb");
            //rs = hardwareMap.get(DcMotor.class, "rs");
            //ls = hardwareMap.get(DcMotor.class, "ls");
            rg = hardwareMap.get(Servo.class, "rg");
            lg = hardwareMap.get(Servo.class, "lg");
            rjk = hardwareMap.get(Servo.class, "rjk");
            ljk = hardwareMap.get(Servo.class, "ljk");
            tlg = hardwareMap.get(Servo.class, "tlg");
            trg = hardwareMap.get(Servo.class, "trg");
            //Slidy = hardwareMap.get(CRServo.class, "sc");
            //rsg = hardwareMap.get(CRServo.class, "rsg");
            //lsg = hardwareMap.get(CRServo.class, "lsg");
            re = hardwareMap.get(Servo.class, "re");
            le = hardwareMap.get(Servo.class, "le");
            //rextention = hardwareMap.get(Servo.class, "re");
            //lextentions = hardwareMap.get(Servo.class, "le");
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            up.setDirection(DcMotorSimple.Direction.REVERSE);
            rjk.setPosition(rightposition2);
            ljk.setPosition(leftposition2);
            lg.setPosition(0.82);
            rg.setPosition(0.04);
            rjr.setPosition(rightposition3);
            ljr.setPosition(leftposition3);
            tlg.setPosition(leftposition);
            trg.setPosition(rightposition);
            re.setPosition(rightposition4);
            le.setPosition(leftposition4);

            navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("navx"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData,
                    NAVX_DEVICE_UPDATE_RATE_HZ);
            yawPIDController = new navXPIDController(navx_device,
                    navXPIDController.navXTimestampedDataSource.YAW);
            while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
                calibration_complete = !navx_device.isCalibrating();
                if ( calibration_complete ) {
                    navx_device.zeroYaw();
                } else {
                    telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                }
            }

            steps = new Vector<Step>();
            initSteps();
            currentStep = steps.get(0);
            currentStepIndex = 0;

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

            parameters.vuforiaLicenseKey = "AYydEH3/////AAAAGXMATUMRIE4Pv8w0T+lHxs5Vah12gKSD60BBnydPYF3GeoUEUBpr9Q4NXikGwa+wLuElb3hZH2ujmFnni6yudqsshk91NxEEeOBZBscu60T3JbZVW05gvgAbxrAQgQRbMomuW3rFL/KhLVeOL+pb0k0DJEAsgTcoL7dahj1z/9tfrZC0vFDIW4qXsnzmjXRyT1MWXc8odL8npQI+FJZoyh8gpfGs6iuY6ZCi+QkjdlRIpsZnozIPCN5S9K1Zv8/3CnOmBz50I7x+fiZM9Soj3jbShvKQyfHRMTYX4b1DAspwJ6ekaU10UxtUeijN2pjfRv8jE857LRDmrBsuO6YBrlI9C49idhYLXADg8DlegTq4 ";

            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        } else {
            telemetry.addData("Ignoring second click", "");
            telemetry.update();
        }
    }

    public abstract void initSteps();
    @Override
    public void start(){
        int count = 0;
        rg.setPosition(1);
        lg.setPosition(0.09);
    }
    @Override
    public void loop() {
        if (state == ATREST) {
            if (currentStep.sType == StepType.WAIT) {
                if (counts == 400) {
                    currentStepIndex = currentStepIndex + 1;
                    counts = 0;
                    if (currentStepIndex >= steps.size()) {
                        state = FINISHED;
                    } else {
                        currentStep = steps.get(currentStepIndex);
                        state = ATREST;

                    }
                } else {
                    counts = counts + 1;
                }
            } else if (currentStep.sType == StepType.MOVE || currentStep.sType == StepType.BACK) {
                resetEncoders();
                state = WAITFORRESETENCODERS;
                initializeNavX(0);

            } else if (currentStep.sType == StepType.RIGHT || currentStep.sType == StepType.LEFT) {
                state = WAITFORTURN;
                initializeNavX(currentStep.turnAngle);
            } else if (currentStep.sType == StepType.MOVEARM) {
                state = JEWELKNOCK;
                initializeNavX(0);
                resetEncoders();
            } else if (currentStep.sType == StepType.VUFORIA) {
                state = VUCHECK;
                counter = 0;
            }
            else if (currentStep.sType == StepType.VUMOVE){
                state = SPECVU;
            }
            else if (currentStep.sType == StepType.JKMOVE){
                state = SPECJK;
            }
            else if (currentStep.sType == StepType.VUJKMOVE){
                state = SPECJKVU;
            }
            else if (currentStep.sType == StepType.VUTURN){
                state = SPECTURN;
                telemetry.addData("SPECTURN","SPEC");
                telemetry.update();
            }
            else if (currentStep.sType == StepType.NEWVUTURN){
                state = NEWSPECTURN;
                telemetry.addData("SPECTURN","SPEC");
                telemetry.update();
            }
            else if (currentStep.sType == StepType.RAISEBLOCK){
                state = RAISE;
                up.setPower(0.75);
                counter = 0;
            }
            else if ( currentStep.sType == StepType.BLOCK){
                state = DROPBLOCK;
            }
            else if (currentStep.sType == StepType.GOTUTOUCH){
                resetEncoders();
                state = WAITFORRESETENCODERS;
                initializeNavX(0);
            }
            else if (currentStep.sType == StepType.LOWBLOCK){
                state = LOWER;
                up.setPower(-0.75);
                counter = 0;
            }
            else if (currentStep.sType == StepType.GRABBLOCK){
                state = GRAB;
            }
        } else if (state == WAITFORRESETENCODERS) {
            if (areEncodersReset()) {
                if (currentStep.sType == StepType.SIDELEFT || currentStep.sType == StepType.SIDERIGHT) {
                    setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                    state = WAITFORSIDE;
                }
                else if (currentStep.sType == StepType.GOTUTOUCH){
                    setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                    state = WAITFORTOUCH;
                }
                else {
                    setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                    state = WAITFORCOUNTS;
                }
            }
        }
        else if (state == WAITFORTURN){
            startYaw = navx_device.getYaw();
            if (startYaw != 0 && counter < 100){
                telemetry.addData("Still Zeroing",startYaw);
                telemetry.update();
                counter++;
            }
            else{
                setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                state = TURNTOANGLE;
            }
        }
        else if (state == WAITFORCOUNTS) {
            if (areCountsReached(currentStep.leftFrontCounts, currentStep.rightFrontCounts, currentStep.leftBackCounts, currentStep.rightBackCounts)) {
                setMotorPower(0, 0, 0, 0);
                rjk.setPosition(rightposition2);
                ljk.setPosition(leftposition2);
                nextStep();
            } else {
                setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
                try {
                    if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                        if (yawPIDResult.isOnTarget()) {
                            telemetry.addData("Yaw: isOntarget", "");
                            telemetry.addData("Left Front", leftFront.getPower());
                            telemetry.addData("Left Back", leftBack.getPower());
                            telemetry.addData("Right Front", rightFront.getPower());
                            telemetry.addData("Right Back", rightBack.getPower());
                            telemetry.update();
                            setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);

                        } else {
                            double output = yawPIDResult.getOutput();
                            telemetry.addData("Yaw:", output);
                            telemetry.update();
                            if (output < 0) {
                                setMotorPower(currentStep.leftFrontPower - output / 3, currentStep.rightFrontPower + output / 3, currentStep.leftBackPower - output / 3, currentStep.rightBackPower + output / 3);
                                telemetry.addData("Left Front", leftFront.getPower());
                                telemetry.addData("Left Back", leftBack.getPower());
                                telemetry.addData("Right Front", rightFront.getPower());
                                telemetry.addData("Right Back", rightBack.getPower());
                                telemetry.update();

                            } else {
                                setMotorPower(currentStep.leftFrontPower + output / 3, currentStep.rightFrontPower - output / 3, currentStep.leftBackPower + output / 3, currentStep.rightBackPower - output / 3);
                                telemetry.addData("Left Front", leftFront.getPower());
                                telemetry.addData("Left Back", leftBack.getPower());
                                telemetry.addData("Right Front", rightFront.getPower());
                                telemetry.addData("Right Back", rightBack.getPower());
                                telemetry.update();

                            }
                        }
                    }
                } catch (InterruptedException e) {

                }

            }
        }

        else if (state == WAITFORSIDE) {
            if (areCountsReached(currentStep.leftFrontCounts, currentStep.rightFrontCounts, currentStep.leftBackCounts, currentStep.rightBackCounts)) {
                setMotorPower(0, 0, 0, 0);
                nextStep();
            } else {
                setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
            }
        }
        else if (state == WAITFORTOUCH){
            telemetry.addData("IN WIT FOR TOUCH","");
            telemetry.update();
            if (currentStep.colorType == RED) {
                rjk.setPosition(0.45);
                setMotorPower(-0.01, -0.01, -0.01, -0.01);
                if (touchRed.isPressed()) {
                    setMotorPower(0, 0, 0, 0);
                    rjk.setPosition(rightposition2);
                    nextStep();
                } else if (areCountsReached(currentStep.leftFrontCounts, currentStep.rightFrontCounts, currentStep.leftBackCounts, currentStep.rightBackCounts)) {
                    setMotorPower(0, 0, 0, 0);
                    rjk.setPosition(rightposition2);
                    nextStep();
                }
            }
            else if (currentStep.colorType == BLUE) {
                ljk.setPosition(0.1);
                setMotorPower(-0.01, -0.01, -0.01, -0.01);
                if (touchBlue.isPressed()) {
                    setMotorPower(0, 0, 0, 0);
                    ljk.setPosition(leftposition2);
                    nextStep();
                }
                else if (areCountsReached(currentStep.leftFrontCounts, currentStep.rightFrontCounts, currentStep.leftBackCounts, currentStep.rightBackCounts)) {
                    setMotorPower(0,0,0,0);
                    rjk.setPosition(rightposition2);
                    nextStep();
                }
            }
        }
        else if (state == TURNTOANGLE){

            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            double yaw = navx_device.getYaw();
            if (Math.abs(yaw - startYaw - currentStep.turnAngle) >= 1) {
                telemetry.addData("Current yaw: ", yaw);
                Log.w("Current yaw: ", Double.toString(yaw));
                telemetry.update();
                try {
                    if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                        if (hasStopped){
                            hasStopped = false;
                            setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                        }
                        if (!yawPIDResult.isOnTarget()) {

                            double output = yawPIDResult.getOutput();
                            if (output < 0) {
                                setMotorPower(-output, output, -output, output);
                            } else {
                                setMotorPower(-output, output, -output, output);
                            }
                            telemetry.addData("NOt on target ", output);
                            Log.w("NOt on target ", Double.toString(output));
                            telemetry.update();
                        }
                    } else {
                        telemetry.addData("NOT GETTING UPDATE","DONT FREAK OUT ITS ALL GOOD");
                        telemetry.update();
                        Log.w("navXRotateToAnglePID Op", "Yaw PID waitForNewUpdate() TIMEOUT.");
                        setMotorPower(0,0,0,0);
                        hasStopped = true;
                    }
                } catch (InterruptedException e) {
                    telemetry.addData("LOOK IM GETTING INTERPUTED", "");
                    Log.w("LOOK IM GETTING ","BORED");
                    telemetry.update();
                }
            } else {
                setMotorPower(0, 0, 0, 0);
                nextStep();
            }

        }
        else if (state == SPECTURN){
            if (currentStep.colorType == RED) {
                if (block == BLOCKL) {
                    currentStep.turnAngle = currentStep.turnAngle - 15;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else if (block == BLOCKR){
                    currentStep.turnAngle = currentStep.turnAngle + 15;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else {
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
            }
            else if (currentStep.colorType == BLUE){
                if (block == BLOCKL) {
                    currentStep.turnAngle = currentStep.turnAngle - 20;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else if (block == BLOCKR){
                    currentStep.turnAngle = currentStep.turnAngle + 6;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else {
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
            }
        }
        else if (state == NEWSPECTURN){
            if (currentStep.colorType == RED) {
                if (block == BLOCKL) {
                    currentStep.turnAngle = currentStep.turnAngle-15;
                    state = WAITFORTURN;
                    telemetry.addData( "TO BLOCKL",currentStep.turnAngle);
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else if (block == BLOCKR){
                    currentStep.turnAngle = currentStep.turnAngle + 25;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKR");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else {
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCK NOTHING");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
            }
            else if (currentStep.colorType == BLUE){
                if (block == BLOCKL) {
                    currentStep.turnAngle = currentStep.turnAngle - 30;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else if (block == BLOCKR){
                    currentStep.turnAngle = currentStep.turnAngle + 10 ;
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
                else {
                    state = WAITFORTURN;
                    telemetry.addData("TURNING","TO BLOCKL");
                    telemetry.update();
                    initializeNavX(currentStep.turnAngle);
                }
            }


        }
        else if (state == JEWELKNOCK){
            if (counter < 150) {
                counter++;
                if (currentStep.colorType == RED) {
                    rjk.setPosition(0.65);
                } else if (currentStep.colorType == BLUE) {
                    ljk.setPosition(0);
                }
                telemetry.addData("TWO CHAINZZZZ:", rjk.getPosition());
                telemetry.update();
            }
            else {
                state = DETECTCOLOR;
                counter = 0;
            }
        }
        else if (state == RAISE){
            if (counter < 125) {
                counter++;
            }
            else {
                counter = 0;
                up.setPower(0);
                nextStep();
            }
        }
        else if (state == LOWER){
            if (counter < 100) {
                counter++;
            }
            else {
                counter = 0;
                up.setPower(0);
                nextStep();
            }
        }
        else if (state == DETECTCOLOR){
            if (colorRed.red() > 1 || colorBlue.red() > 1) {
                colorVal = RED;
                telemetry.addData("Getting Red", "");
                state = TURNARM;
            }
            else if (colorBlue.blue() > 1 || colorRed.blue() > 1){
                colorVal = BLUE;
                telemetry.addData("Getting blue", "");
                state = TURNARM;
            }
            else if (counter > 500){
                rjk.setPosition(rightposition2);
                ljk.setPosition(leftposition2);
                rjr.setPosition(rightposition3);
                ljr.setPosition(leftposition3);
                nextStep();
            }
            else{
                telemetry.addData("Not getting anything", "");
                counter++;
                if(counter < 100 && currentStep.colorType == RED){
                    positionRed = positionRed - 0.001;
                    rjr.setPosition(positionRed);
                }
                else if(counter < 100 && currentStep.colorType == BLUE){
                    positionBlue = positionBlue + 0.001;
                    ljr.setPosition(positionBlue);
                }
            }
        }
        else if (state == TURNARM){
            if (currentStep.colorType == RED){
                if (colorVal == BLUE){
                    if (counter < 100) {
                        rjr.setPosition(0.35);
                        counter++;
                    }
                    else {
                        rjk.setPosition(rightposition2);
                        ljk.setPosition(leftposition2);
                        rjr.setPosition(rightposition3);
                        ljr.setPosition(leftposition3);
                        nextStep();
                    }
                }
                else if (colorVal == RED){
                    if (counter <100) {
                        rjr.setPosition(0.8);
                        counter++;
                    }
                    else {
                        rjk.setPosition(rightposition2);
                        ljk.setPosition(leftposition2);
                        rjr.setPosition(rightposition3);
                        ljr.setPosition(leftposition3);
                        nextStep();
                    }
                }
            }
            else if (currentStep.colorType == BLUE){
                if (colorVal == RED){
                    if (counter < 100) {
                        ljr.setPosition(1);
                        counter++;
                    }
                    else {
                        rjk.setPosition(rightposition2);
                        ljk.setPosition(leftposition2);
                        rjr.setPosition(rightposition3);
                        ljr.setPosition(leftposition3);
                        nextStep();
                    }
                }
                else if (colorVal == BLUE){
                    if (counter < 100) {
                        ljr.setPosition(0.69);
                        counter++;
                    }
                    else {
                        rjk.setPosition(rightposition2);
                        ljk.setPosition(leftposition2);
                        rjr.setPosition(rightposition3);
                        ljr.setPosition(leftposition3);
                        nextStep();
                    }
                }
            }
        }
        else if (state == GRAB){
            if (counter < 100) {
                rg.setPosition(1);
                lg.setPosition(0.09);
                counter++;
            }
            else{
                counter = 0;
                nextStep();
            }
        }
        else if (state == SPECJKVU){
            double jewelMoveBack = convertDistance(2);
            double jewelMoveForward = convertDistance(5);
            double blockMoveExtra = convertDistance(5);
            double blockMoveLess = convertDistance(7);

            resetEncoders();
            initializeNavX(0);
            if (block == BLOCKR){
                if (currentStep.colorType == RED) {
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + blockMoveLess;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + blockMoveLess;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + blockMoveLess;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + blockMoveLess;
                }
                else if (currentStep.colorType == BLUE){
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - blockMoveExtra;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - blockMoveExtra;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - blockMoveExtra;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - blockMoveExtra;
                }
                if (colorVal != currentStep.colorType){
                    //go most
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - jewelMoveBack;

                }
                else if (colorVal == currentStep.colorType){
                    //go least
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                }
                telemetry.addData("Getting block right",currentStep.distance);
                telemetry.update();
            }
            else if (block == BLOCKC){
                if (colorVal != currentStep.colorType){
                    //go most
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - jewelMoveBack;

                }
                else if (colorVal == currentStep.colorType){
                    //go least
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                }
                telemetry.addData("Getting block center",currentStep.distance);
                telemetry.update();
            }
            else if (block == BLOCKL){
                if (currentStep.colorType == RED) {
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - blockMoveExtra;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - blockMoveExtra;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - blockMoveExtra;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - blockMoveExtra;
                }
                else if (currentStep.colorType == BLUE){
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + blockMoveLess;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + blockMoveLess;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + blockMoveLess;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + blockMoveLess;
                }
                if (colorVal != currentStep.colorType){
                    //go most
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - jewelMoveBack;

                }
                else if (colorVal == currentStep.colorType){
                    //go least
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                }
                telemetry.addData("Getting block left",currentStep.distance);
                telemetry.update();
            }
            else{
                if (colorVal != currentStep.colorType){
                    //go most
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - jewelMoveBack;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - jewelMoveBack;

                }
                else if (colorVal == currentStep.colorType){
                    //go least
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                }
                telemetry.addData("Getting block none",currentStep.distance);
                telemetry.update();
            }
            state = WAITFORRESETENCODERS;
        }
        else if (state == SPECJK){
            double jewelMoveBack = convertDistance(1);
            double jewelMoveForward = convertDistance(3.5);
            resetEncoders();
            initializeNavX(0);
            if (colorVal != currentStep.colorType){
                //go most
                currentStep.rightFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                currentStep.leftFrontCounts = currentStep.rightFrontCounts - jewelMoveBack;
                currentStep.leftBackCounts = currentStep.rightFrontCounts - jewelMoveBack;
                currentStep.rightBackCounts = currentStep.rightFrontCounts - jewelMoveBack;

            }
            else if (colorVal == currentStep.colorType){
                //go least
                currentStep.rightFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                currentStep.leftFrontCounts = currentStep.rightFrontCounts + jewelMoveForward;
                currentStep.leftBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
                currentStep.rightBackCounts = currentStep.rightFrontCounts + jewelMoveForward;
            }
            state = WAITFORRESETENCODERS;
        }
        else if (state == SPECVU){
            double vuMoveExtra = convertDistance(6.5);
            double vuMoveLess = convertDistance(3.5);
            resetEncoders();
            initializeNavX(0);
            if (block == BLOCKR){
                if (currentStep.colorType == BLUE){
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - vuMoveExtra;
                }
                telemetry.addData("Getting block right",currentStep.distance);
                telemetry.update();
            }
            else if (block == BLOCKC){
                currentStep.rightFrontCounts = currentStep.rightFrontCounts - vuMoveLess;
                currentStep.leftFrontCounts = currentStep.rightFrontCounts - vuMoveLess;
                currentStep.leftBackCounts = currentStep.rightFrontCounts - vuMoveLess;
                currentStep.rightBackCounts = currentStep.rightFrontCounts - vuMoveLess;
                telemetry.addData("Getting block center",currentStep.distance);
                telemetry.update();
            }
            else if (block == BLOCKL){
                if (currentStep.colorType == RED) {
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - vuMoveExtra;
                }
                telemetry.addData("Getting block left",currentStep.distance);
                telemetry.update();
            }
            else{
                telemetry.addData("Getting block none",currentStep.distance);
                telemetry.update();
            }
            state = WAITFORRESETENCODERS;
        }
        else if(state ==  VUCHECK){
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.CENTER){
                block = BLOCKC;
                telemetry.addData("Center ", block);
                telemetry.update();
                devuforia();
                nextStep();
            }
            else if(vuMark ==  RelicRecoveryVuMark.LEFT){
                block = BLOCKL;
                telemetry.addData("Left ", block);
                telemetry.update();
                devuforia();
                nextStep();

            }
            else if(vuMark ==  RelicRecoveryVuMark.RIGHT){
                block = BLOCKR;
                telemetry.addData("Right ",block);
                telemetry.update();
                devuforia();
                nextStep();
            }
            else {
                counter++;
                if (counter > 500){
                    nextStep();
                    counter = 0;
                }
                telemetry.addData("VuForia is getting no readings","");
                telemetry.update();
            }
            telemetry.update();
        }
        else if(state == DROPBLOCK){
            if (counter < 10) {
                counter++;
                rg.setPosition(0.1);
                lg.setPosition(0.9);
            }
            else {
                counter = 0;
                nextStep();
            }

        }
        else if (state == FINISHED){
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
    @Override
    public void stop(){
        navx_device.close();
        devuforia();
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public boolean areEncodersReset() {
        return leftFront.getCurrentPosition() == 0 &&
                rightFront.getCurrentPosition() == 0 &&
                leftBack.getCurrentPosition() == 0 &&
                rightBack.getCurrentPosition() == 0;
    }

    public boolean areCountsReached(double leftFrontCounts, double rightFrontCounts, double leftBackCounts, double rightBackCounts) {
        telemetry.addData("Is it negative?",leftFront.getCurrentPosition());
        return ( Math.abs(leftFront.getCurrentPosition()) >= Math.abs(leftFrontCounts) &&
                Math.abs(rightFront.getCurrentPosition()) >= Math.abs(rightFrontCounts) &&
                Math.abs(leftBack.getCurrentPosition())>= Math.abs(leftBackCounts) &&
                Math.abs(rightBack.getCurrentPosition()) >= Math.abs(rightBackCounts) );

    }

    public void setMotorPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
    public void nextStep(){
        counter = 0;
        currentStepIndex = currentStepIndex + 1;
        if (currentStepIndex >= steps.size()) {
            state = FINISHED;
        } else {
            currentStep = steps.get(currentStepIndex);
            state = ATREST;
        }
    }

    public void devuforia(){
        relicTrackables.deactivate();
    }
}