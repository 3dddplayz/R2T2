package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Movement.PIDcontroller;
import org.firstinspires.ftc.teamcode.Movement.PIDlinear;
import org.firstinspires.ftc.teamcode.Odometry.Odometry;

@TeleOp(name = "TeleOp New Mecanum Drive")
public class TeleOpNewDrive extends LinearOpMode {

    DcMotor back_left_drive,back_right_drive,front_right_drive,front_left_drive,right_pully,left_pully,xOd,yOd;
    private CRServo plane;
    private Servo upServo,Push,grabberSecond,paddleR,paddleL,leftTrunk,rightTrunk;
    private IMU imu_IMU;
    YawPitchRollAngles angles;
    double yaw,correction,gain;

    //Lifter limits
    double lifter_power = 0;
    double lifter_gain = 1;
    int max_lifter_height = 3100;
    double lifter_target;

    //Booleans
    boolean grabberFirstClosed = false;
    boolean grabberSecondClosed = false;
    Boolean paddleClosed = true;
    double paddlePos = 0.26;
    double globalX,globalY,lastOdX,lastOdY;
    double yawOffset = 0;
    double lastYaw;
    double yawSum=0;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        double maxSpeed;
        double x2 = 0;
        double y = 0;
        double x = 0;
        double d;
        double max;
        double targetYaw = 0;
        double deceleration;
        double joystickGain;
        double deceleration2;
        double joystickGain2;
        double dpadGain;
        double brakeMult;
        double maxX;
        double maxY;
        double maxX2;
        boolean dpad;
        globalX = 0;
        globalY = 0;
        double x1 = 0;
        double y1 = 0;
        double h1 = 0;
        PIDcontroller pid;
        //double correction = 0;
        int cClamp = 0;
        Boolean overide = false;


        double dSpeed = 1.5;
        Boolean aPressed = false;
        Boolean grabberClosed = false;
        Boolean rbPressed = false;
        Boolean xPressed = false;
        Boolean downStarted = false;
        Boolean extended = false;
        Boolean holdingLT = false;
        Boolean paddleOveride = false;
        Boolean axOveride = false;

        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");

        left_pully = hardwareMap.get(DcMotor.class, "pullyL");
        right_pully = hardwareMap.get(DcMotor.class, "pullyR");

        xOd = hardwareMap.get(DcMotor.class, "xOdo");
        yOd = hardwareMap.get(DcMotor.class, "yOdo");

        plane = hardwareMap.get(CRServo.class, "plane");
        upServo = hardwareMap.get(Servo.class, "up_servo");
        Push = hardwareMap.get(Servo.class, "Push");
        grabberSecond = hardwareMap.get(Servo.class, "grabberSecond");
        paddleR = hardwareMap.get(Servo.class, "RightP");
        paddleL = hardwareMap.get(Servo.class, "LeftP");
        leftTrunk = hardwareMap.get(Servo.class, "left_trunk");
        rightTrunk = hardwareMap.get(Servo.class, "right_trunk");

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imp;
        imp = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu_IMU.initialize(imp);
        imu_IMU.resetYaw();
        yaw = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // Put initialization blocks here.
        waitForStart();

        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // front_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // front_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        left_pully.setDirection(DcMotor.Direction.REVERSE);
        right_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOd.setDirection(DcMotor.Direction.REVERSE);
        xOd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // back_left_drive.setMode(DcMotor.RunMode.RUN
        // _WITHOUT_ENCODER);
        // back_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // front_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // front_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double correction = 0;

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here

//                pid = new PIDcontroller(0.015575, 0, 0.016, 60);
                PIDlinear pidLifter = new PIDlinear(0.00008, 0, 0.00005, 60);

                //drive parameters


                //drive code
                dpadGain = 0.2;

                //stick values
                x2 = gamepad1.right_stick_x;
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                //dpad code
                if (gamepad1.dpad_left) {
                    x = -1;
                } else if (gamepad1.dpad_down) {
                    y = -1;
                } else if (gamepad1.dpad_right) {
                    x = 1;
                } else if (gamepad1.dpad_up) {
                    y = 1;
                }
                //brake code


                double power = Math.hypot(x, y);
                double theta = Math.atan2(y, x);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);

                if (gamepad1.right_bumper) {
                    power = Math.min(.175, power);
                    if (Math.abs(x2) > 0) {
                        x2 = Math.min(Math.abs(x2), .175) * x2 / Math.abs(x2);
                    }

                }

                max = Math.max(Math.abs(sin), Math.abs(cos));

                double div = 1;
                if ((power + Math.abs(x2)) > 1) {
                    div = power + Math.abs(x2);
                }

                front_right_drive.setPower((power * sin / max - x2) / div);
                back_right_drive.setPower((power * cos / max - x2) / div);
                back_left_drive.setPower((power * sin / max + x2) / div);
                front_left_drive.setPower((power * cos / max + x2) / div);


                //Manual Grab code
                if (gamepad2.right_bumper && !rbPressed) {
                    rbPressed = true;
                    if (grabberClosed) {
                        grabberClosed = false;
                        grabberSecond.setPosition(0);
                    } else {
                        grabberClosed = true;
                        grabberSecond.setPosition(0.12);
                    }
                } else if (!gamepad2.right_bumper) {
                    rbPressed = false;
                }
                //Trunk code
                // if (gamepad2.dpad_down){
                //   upServo.setPosition(0.85);
                // }

                if (gamepad2.dpad_up) {
                    upServo.setPosition(.4);
                } else if (gamepad2.dpad_down) {
                    upServo.setPosition(0.85);
                }
                else {
                    upServo.setPosition(upServo.getPosition() + .01 * gamepad2.left_stick_y);
                }
                if(!downStarted&&!gamepad2.a) {
                    leftTrunk.setPosition(Math.min(1-.87, Math.max(.022, (leftTrunk.getPosition() + .01 * gamepad2.right_stick_y))));
                    rightTrunk.setPosition(Math.min(1-.022, Math.max(0.87, (rightTrunk.getPosition() - .01 * gamepad2.right_stick_y))));
                }
                if (gamepad2.left_bumper) {
                    axOveride = true;
                    xPressed = false;
                    aPressed = false;
                }
                if (!gamepad2.a && !gamepad2.x) {
                    axOveride = false;

                }

                if (gamepad2.a && !downStarted && !axOveride) {
                    aPressed = true;
                    right_pully.setTargetPosition(0);
                    left_pully.setTargetPosition(0);
                    right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition()) / 2;
                    right_pully.setPower(.25 + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
                    left_pully.setPower(.25 + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
                    if (!(Math.abs(gamepad2.left_stick_y) >= 0.05) && !overide) {
                        upServo.setPosition(.76);
                    } else overide = true;
                    grabberSecond.setPosition(0);
                    grabberClosed = false;
                }
                if (!gamepad2.a && aPressed && !axOveride) {
                    if (!downStarted) {
                        resetRuntime();
                        downStarted = true;
                        grabberSecond.setPosition(0);
                    }

                    if(getRuntime()<0.4){
                        paddleClosed = false;
                        upServo.setPosition(.845);
                    }else if(getRuntime()<0.5){
                        leftTrunk.setPosition(1-.87);
                        rightTrunk.setPosition(.87);
                    }else if(getRuntime()<0.7){
                        grabberSecond.setPosition(.12);
                    }else if(getRuntime()<.85){
                        paddleOveride = false;
                        paddleClosed = true;
                    } else if (getRuntime()<.9) {
                        leftTrunk.setPosition(0);
                        rightTrunk.setPosition(1);
                        grabberClosed = true;
                        aPressed = false;
                        downStarted = false;
                        overide = false;
                    }
                }
                //Auto Grab code
//                if (gamepad2.a && !downStarted && !axOveride) {
//                    aPressed = true;
//                    right_pully.setTargetPosition(394);
//                    left_pully.setTargetPosition(394);
//                    right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition()) / 2;
//                    right_pully.setPower(1 + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
//                    left_pully.setPower(1 + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
//                    if (!(Math.abs(gamepad2.left_stick_y) >= 0.05) && !overide) {
//                        upServo.setPosition(.98322);
//                    } else overide = true;
//                    grabberSecond.setPosition(0);
//                    grabberClosed = false;
//                }
//                if (!gamepad2.a && aPressed && !axOveride) {
//                    if (!downStarted) {
//                        resetRuntime();
//                        downStarted = true;
//                    }
//                    // paddleOveride = true;
//                    // paddleR.setPosition(paddlePos);
//                    // paddleL.setPosition(1);
//                    right_pully.setTargetPosition(0);
//                    left_pully.setTargetPosition(0);
//                    right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition()) / 2;
//                    right_pully.setPower(1 + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
//                    left_pully.setPower(1 + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
//                    upServo.setPosition(0.84);
//
//                    if (getRuntime() >= 0.5) {
//                        grabberClosed = true;
//                        grabberSecond.setPosition(0.12);
//                        aPressed = false;
//                        downStarted = false;
//                        overide = false;
//                        paddleOveride = false;
//                        paddleClosed = true;
//                    }
//                }
//
//                if (gamepad2.x && !downStarted && !axOveride) {
//                    xPressed = true;
//                    right_pully.setTargetPosition(330);
//                    left_pully.setTargetPosition(330);
//                    right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (gamepad2.left_trigger > 0.05 && (!aPressed) && (!xPressed)) {
                    right_pully.setTargetPosition(max_lifter_height);
                    left_pully.setTargetPosition(max_lifter_height);
                    right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition()) / 2;
                    right_pully.setPower(gamepad2.left_trigger + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
                    left_pully.setPower(gamepad2.left_trigger + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
                } else if (gamepad2.right_trigger > 0.05 && (!aPressed) && (!xPressed)) {
                    right_pully.setTargetPosition(0);
                    left_pully.setTargetPosition(0);
                    right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition()) / 2;
                    right_pully.setPower(gamepad2.right_trigger + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
                    left_pully.setPower(gamepad2.right_trigger + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
                } else if (!aPressed && (!xPressed)) {
                    right_pully.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                    left_pully.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                    lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition()) / 2;
                    right_pully.setPower(pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
                    left_pully.setPower(pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
                }

                //airplane launcher code
                if (gamepad1.y) {
                    plane.setPower(-1);
                } else plane.setPower(0);

                //Linear Servo

                if (gamepad2.b) {
                    if (!extended) {
                        Push.setPosition(0.4);
                        extended = true;
                    }
                }

                if (gamepad2.y) {
                    Push.setPosition(.55);
                    extended = false;
                } else if (!extended) {
                    Push.setPosition(0.25);
                    extended = false;
                }

                //paddles code
                if (gamepad1.left_trigger > 0.05 && !holdingLT) {
                    holdingLT = true;
                    if (paddleClosed) paddleClosed = false;
                    else paddleClosed = true;
                } else if (!(gamepad1.left_trigger > 0.05)) {
                    holdingLT = false;
                }


                if (paddleClosed && !paddleOveride) {
                    paddleR.setPosition(paddlePos);
                    paddleL.setPosition(1 - paddlePos);
                } else if (!paddleOveride) {
                    paddleR.setPosition(0);
                    paddleL.setPosition(1);
                }
                calcPos();

                telemetry.addData("X pos: ", globalX);
                telemetry.addData("Y pos: ", globalY);
                telemetry.addData("Yaw: ", yaw);
                telemetry.addData("Yaw Sum: ", yawSum);
                telemetry.addData("UpServo: ", upServo.getPosition());


                telemetry.addData("Left Paddle: ", paddleL.getPosition());
                telemetry.addData("Right Paddle", paddleR.getPosition());
                telemetry.addData("Lifter: ", lifter_target);
                telemetry.addData("Motor Position: ", (Math.abs(back_right_drive.getCurrentPosition()) + Math.abs(back_left_drive.getCurrentPosition()) + Math.abs(front_right_drive.getCurrentPosition()) + Math.abs(front_left_drive.getCurrentPosition())) / 4);
                telemetry.update();
                telemetry.addData("Push Position: ", Push.getPosition());
                telemetry.addData("Right Trunk: ", rightTrunk.getPosition());
                telemetry.addData("Left Trunk: ", leftTrunk.getPosition());

            }
        }
    }
    public void getYawValue() {
        lastYaw = yaw;
        angles = imu_IMU.getRobotYawPitchRollAngles();
        yaw = -angleSum(angles.getYaw(AngleUnit.DEGREES),yawOffset);
    }
    public void calcPos(){
        double xOdRad = 13.8029;
        double yOdRad = 19.4581;//190
        double odWheelDiam = 4.8;//cm
        double odCostanst = 2000/(odWheelDiam*Math.PI);
        getYawValue();
        double xPos = xOd.getCurrentPosition();
        double yPos = yOd.getCurrentPosition();


        double deltaYaw = (yaw - lastYaw);
        if(Math.abs(deltaYaw+360)<Math.abs(deltaYaw)){
            deltaYaw = deltaYaw+360;
        } else if(Math.abs(deltaYaw-360)<Math.abs(deltaYaw)){
            deltaYaw = deltaYaw-360;
        }

        yawSum += deltaYaw;
        telemetry.addData("delta yaw", deltaYaw);
        double y =(yPos-lastOdY) - deltaYaw*(yOdRad*Math.PI/180)*odCostanst;
        double x =(xPos-lastOdX) + deltaYaw*(xOdRad*Math.PI/180)*odCostanst;
        double[] output = rotate(x,y,-yaw);
        globalX += output[0];//-(deltaYaw*(xOdRad*2*Math.PI/360))/odWheelRad*encoderWheelRad;
        globalY += output[1];


//        telemetry.addData("y Correction", (yPos-lastOdY));
//        telemetry.addData("y Correction Pos", y);
//        telemetry.addData("y Correction value", deltaYaw*(yOdRad*Math.PI/180)*odCostanst);
//
//        telemetry.addData("y Correction", (xPos-lastOdX));
//        telemetry.addData("y Correction Pos", x);
//        telemetry.addData("y Correction value", deltaYaw*(xOdRad*Math.PI/180)*odCostanst);
//        telemetry.addData("raw y", xPos);

        lastOdX = xPos;
        lastOdY = yPos;
    }




    public double[] rotate(double x, double y, double angle) {
        angle *= 2 * Math.PI / 360;
        double cos = Math.cos  (angle);
        double sin = Math.sin(angle);
        double newX = x * cos - y * sin;
        double newY = x * sin + y * cos;
        System.out.println("x:" + newX + " y:" + newY);
        double[] output = {newX, newY};
        return output;
    }
    public double angleSum(double ang1,double ang2) {
        double result = ang1 + ang2;
        while (result < -180) {
            result += 360;
        }
        while (result > 180) {
            result -= 360;
        }
        return result;
    }
}

