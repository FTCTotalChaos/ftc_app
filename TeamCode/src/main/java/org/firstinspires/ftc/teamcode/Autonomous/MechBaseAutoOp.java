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
    Servo rextention;
    Servo lextentions;
    Servo rg;
    Servo lg;
    String string1;
    double position = 0;
    double rightposition = 0.575;
    double leftposition = 0.39;
    double rightposition2 = 0.66;
    double leftposition2 = 0.65;
    double position3 = 0;
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
    TouchSensor touchBlue;
    TouchSensor touchRed;
    boolean hasTouched;
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
    final static int TURNEXTENTION = 9;
    final static int WAITFORSIDE = 10;
    final static int DROPBLOCK = 11;
    final static int RAISE = 12;
    final static int SPECJKVU = 13;
    final static int SPECJK = 14;
    final static int SPECVU = 15;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 0.5;
    final static double WHEEL_DIAMETER = 4;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    //Start step types
    final static int MOVE = 1;
    final static int RIGHT = 2;
    final static int LEFT = 3;
    final static int RANGE = 4;
    final static int WAITFORTOUCH = 5;
    final static int BACK = 6;
    final static int WAIT = 9;
    final static int MOVEARM = 10;
    final static int BLOCK = 11;
    final static int BACKWARD = 12;
    final static int NONE = 13;
    final static int VUFORIA = 14;
    final static int SIDERIGHT = 15;
    final static int SIDELEFT = 16;
    final static int TOUCHY = 17;
    final static int RAISEBLOCK = 18;
    final static int VUJKMOVE = 19;
    final static int JKMOVE = 19;
    final static int VUMOVE = 19;
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
        public int sType;
        public int colorType;

        public Step(double dist, double left, double right, int stepType, int angle, int col) {
            distance = dist;
            sType = stepType;
            colorType = col;
            if (stepType == MOVE) {
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = right;
            } else if (stepType == LEFT) {
                turnAngle = -angle;
                leftFrontPower = left;
                rightFrontPower = -right;
                leftBackPower = left;
                rightBackPower = -right;

            } else if (stepType == RIGHT) {
                turnAngle = angle;
                leftFrontPower = -left;
                rightFrontPower = right;
                leftBackPower = -left;
                rightBackPower = right;
            } else if (stepType == BACK) {
                rightFrontCounts = -(convertDistance(distance));
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = -left;
                rightFrontPower = -right;
                leftBackPower = -left;
                rightBackPower = -right;
            } else if (stepType == SIDELEFT) {
                turnAngle = angle;
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = -rightFrontCounts;
                rightBackCounts = -rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = -left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = -right;
            } else if (stepType == SIDERIGHT) {
                rightFrontCounts = -(convertDistance(distance));
                leftFrontCounts = -rightFrontCounts;
                rightBackCounts = -rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = -right;
                leftBackPower = -left;
                rightBackPower = right;
            } else if (stepType == WAIT) {
                leftFrontPower = 0;
                rightFrontPower = 0;
                leftBackPower = 0;
                rightBackPower = 0;
            } else if (stepType == MOVEARM) {
                rightFrontCounts = convertDistance(distance);
                leftFrontCounts = rightFrontCounts;
                rightBackCounts = rightFrontCounts;
                leftBackCounts = rightFrontCounts;
                leftFrontPower = left;
                rightFrontPower = right;
                leftBackPower = left;
                rightBackPower = right;
            }
            else if (stepType == VUJKMOVE || stepType == VUMOVE ||stepType == JKMOVE) {
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
            //touchRed = hardwareMap.get(TouchSensor.class, "tr");
            //touchBlue = hardwareMap.get(TouchSensor.class, "tb");
            //rs = hardwareMap.get(DcMotor.class, "rs");
            //ls = hardwareMap.get(DcMotor.class, "ls");
            rg = hardwareMap.get(Servo.class, "rg");
            lg = hardwareMap.get(Servo.class, "lg");
            rjk = hardwareMap.get(Servo.class, "rjk");
            ljk = hardwareMap.get(Servo.class, "ljk");
            //rextention = hardwareMap.get(Servo.class, "re");
            //lextentions = hardwareMap.get(Servo.class, "le");
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rjk.setPosition(rightposition2);
            ljk.setPosition(leftposition2);
            rg.setPosition(rightposition);
            lg.setPosition(leftposition);

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
        rg.setPosition(0.15);
        lg.setPosition(0.77);
    }
    @Override
    public void loop() {
        if (state == ATREST) {
            if (currentStep.sType == WAIT) {
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
            } else if (currentStep.sType == MOVE || currentStep.sType == BACK) {
                resetEncoders();
                state = WAITFORRESETENCODERS;
                initializeNavX(0);

            } else if (currentStep.sType == RIGHT || currentStep.sType == LEFT) {
                state = WAITFORTURN;
                initializeNavX(currentStep.turnAngle);
            } else if (currentStep.sType == TOUCHY) {
                state = WAITFORTOUCH;
            } else if (currentStep.sType == MOVEARM) {
                state = JEWELKNOCK;
                initializeNavX(0);
                resetEncoders();
            } else if (currentStep.sType == VUFORIA) {
                state = VUCHECK;
                counter = 0;
            }
            else if (currentStep.sType == VUMOVE){
                state = SPECVU;
            }
            else if (currentStep.sType == JKMOVE){
                state = SPECJK;
            }
            else if (currentStep.sType == VUJKMOVE){
                state = SPECJKVU;
            }
            else if (currentStep.sType == RAISEBLOCK){
                state = RAISE;
                up.setPower(0.75);
                counter = 0;
            }
            else if ( currentStep.sType == BLOCK){
                state = DROPBLOCK;
            }
        } else if (state == WAITFORRESETENCODERS) {
            if (areEncodersReset()) {
                if (currentStep.sType == SIDELEFT || currentStep.sType == SIDERIGHT) {
                    setMotorPower(currentStep.leftFrontPower, currentStep.rightFrontPower, currentStep.leftBackPower, currentStep.rightBackPower);
                    state = WAITFORSIDE;
                } else {
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
            if (currentStep.colorType ==  RED){
                if (touchRed.isPressed()){
                    setMotorPower(0,0,0,0);
                }
                else {
                    setMotorPower(-currentStep.leftFrontPower,currentStep.rightFrontPower, currentStep.leftBackPower, -currentStep.rightBackPower);
                }
            }
            else if (currentStep.colorType ==  BLUE){
                if (touchBlue.isPressed()){
                    setMotorPower(0,0,0,0);
                }
                else {
                    setMotorPower(currentStep.leftFrontPower, -currentStep.rightFrontPower, -currentStep.leftBackPower, currentStep.rightBackPower);
                }
            }
        }
        else if (state == TURNTOANGLE){

            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            double yaw = navx_device.getYaw();
            if (Math.abs(yaw - startYaw - currentStep.turnAngle) >= 1) {
                telemetry.addData("Current yaw: ", yaw);
                telemetry.update();
                try {
                    if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                        if (!yawPIDResult.isOnTarget()) {
                            double output = yawPIDResult.getOutput();
                            if (output < 0) {
                                setMotorPower(-output, output, -output, output);
                            } else {
                                setMotorPower(-output, output, -output, output);
                            }
                            telemetry.addData("NOt on target ", output);
                            telemetry.update();
                        }
                    } else {
                        Log.w("navXRotateToAnglePID Op", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    }
                } catch (InterruptedException e) {

                }
            } else {
                setMotorPower(0, 0, 0, 0);
                nextStep();
            }

        }
        else if (state == JEWELKNOCK){
            if (counter < 150) {
                counter++;
                if (currentStep.colorType == RED) {
                    rjk.setPosition(0);
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
            if (currentStep.colorType == RED) {
                if (colorRed.red() > 0) {
                    colorVal = RED;
                    telemetry.addData("I'm getting red", colorRed.red());
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.leftFrontPower = currentStep.leftFrontPower * -1;
                    currentStep.rightFrontPower = currentStep.rightFrontPower * -1;
                    currentStep.leftBackPower = currentStep.leftBackPower * -1;
                    currentStep.rightBackPower = currentStep.rightBackPower * -1;
                    telemetry.update();
                    counter = 0;
                    state = WAITFORRESETENCODERS;
                } else if (colorRed.blue() > 0) {
                    colorVal = BLUE;
                    telemetry.addData("I'm getting blue", colorRed.blue());
                    telemetry.update();
                    state = WAITFORRESETENCODERS;
                    counter = 0;
                } else if (counter > 600) {
                    rjk.setPosition(rightposition2);
                    ljk.setPosition(leftposition2);
                    counter = 0;
                    nextStep();
                } else {
                    telemetry.addData("Not getting anything", colorRed.red());
                    telemetry.addData("Not getting anything", colorRed.blue());
                    counter++;
                }
            }
            else if (currentStep.colorType == BLUE){
                if (colorBlue.red() > 0) {
                    colorVal = RED;
                    telemetry.addData("I'm getting red", colorBlue.red());
                    telemetry.update();
                    state = WAITFORRESETENCODERS;
                    counter = 0;

                } else if (colorBlue.blue() > 0) {
                    colorVal = BLUE;
                    telemetry.addData("I'm getting blue", colorBlue.blue());
                    telemetry.update();
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + convertDistance(1);
                    currentStep.leftFrontPower = currentStep.leftFrontPower * -1;
                    currentStep.rightFrontPower = currentStep.rightFrontPower * -1;
                    currentStep.leftBackPower = currentStep.leftBackPower * -1;
                    currentStep.rightBackPower = currentStep.rightBackPower * -1;
                    state = WAITFORRESETENCODERS;
                    counter = 0;

                } else if (counter > 300) {
                    counter = 0;
                    rjk.setPosition(rightposition2);
                    ljk.setPosition(leftposition2);
                    nextStep();
                } else {
                    telemetry.addData("Not getting anything", colorBlue.red());
                    telemetry.addData("Not getting anything", colorBlue.blue());
                    counter++;
                }
            }
        }
        else if (state == SPECJKVU){
            double jewelMoveBack = convertDistance(2);
            double jewelMoveForward = convertDistance(13);
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
            double jewelMoveBack = convertDistance(2);
            double jewelMoveForward = convertDistance(13);
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
            double vuMoveExtra = convertDistance(5);
            double vuMoveLess = convertDistance(7);

            resetEncoders();
            initializeNavX(0);
            if (block == BLOCKR){
                if (currentStep.colorType == RED) {
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + vuMoveLess;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + vuMoveLess;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + vuMoveLess;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + vuMoveLess;
                }
                else if (currentStep.colorType == BLUE){
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts - vuMoveExtra;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts - vuMoveExtra;
                }
                telemetry.addData("Getting block right",currentStep.distance);
                telemetry.update();
            }
            else if (block == BLOCKC){
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
                else if (currentStep.colorType == BLUE){
                    currentStep.rightFrontCounts = currentStep.rightFrontCounts + vuMoveLess;
                    currentStep.leftFrontCounts = currentStep.rightFrontCounts + vuMoveLess;
                    currentStep.leftBackCounts = currentStep.rightFrontCounts + vuMoveLess;
                    currentStep.rightBackCounts = currentStep.rightFrontCounts + vuMoveLess;
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
                rg.setPosition(0.575);
                lg.setPosition(0.39);
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