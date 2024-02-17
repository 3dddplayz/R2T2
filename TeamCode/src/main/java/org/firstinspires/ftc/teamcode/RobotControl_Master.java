package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Movement.PIDcontroller;
import org.firstinspires.ftc.teamcode.Movement.PIDlinear;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;



@Autonomous(name="Auto Master")
public class RobotControl_Master extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    public DcMotor back_left_drive;
    public DcMotor back_right_drive;
    public DcMotor front_left_drive;
    public DcMotor front_right_drive;
    private DcMotor left_pully;
    private DcMotor right_pully;
    private CRServo plane;

    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;
    public DistanceSensor distance;
    private DcMotor odometryY;
    public IMU imu_IMU;
    public IMU.Parameters imp;
    public PIDcontroller driverPID;
    public PIDcontroller turnPID;
    double targetYaw = 0;
    YawPitchRollAngles angles;
    private Servo upServo;
    private Servo Push;
    private Servo grabberSecond;
    double yaw;
    double correction;
    double gain;
    int count;
    double time;
    double endTime;
    double time1;
    double endTime1;
    List<Recognition> currentRec;
    int loco;

    private DcMotor xOd;
    private DcMotor yOd;
    public Deadline rateLimit;

    double lifter_power = 0;
    double lifter_gain = 1;
    int max_lifter_height = 6500;
    double lifter_target;
    PIDcontroller pid;
    PIDlinear strafePID;
    PIDlinear movePID;
    PIDlinear pidLifter;
    public int matCoEff = 7250; //<- motor encoder
    public int matCoEffStrafe = 7250;
    public int matCoEffOd = 7250;
    //public int matCoEff = 6125; // <- odometry encoder
    private final int READ_PERIOD = 1;
    int targetPos;
    private HuskyLens huskyLens;
    public boolean Check = false;
    int x;
    int y;
    int result;
    String color1;
    BNO055IMU imu;

    public double globalX = 0;
    public double globalY = 0;
    public int lastOdX = 0;
    public int lastOdY = 0;
    public double yawOffset = 0;
    public double distance1;
    final static double L = 20.12; // distance between encoder 1 and 2 in cm
    final static double B = 11.5; //distance between the midpoint of encoder 1 and 2 and encoder 3
    final static double R = 3.0; //wheel radius in cm
    final static double N = 8192;
    final static double cm_per_tick = 2.0 * Math.PI * R / N;
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    public int oldRightPosition = 0;
    public int oldLeftPosition = 0;
    public int oldAuxPosition = 0;
//    public XyhVector START_POS = new XyhVector
//    public XyhVector pos = new XyhVector(START_POS);

    @Override
    public void runOpMode() {
//        Tl=7.250;
//        Tb=7.750;
//        Tr=7.250;

        createRobot();
        resetEncorder();
        imuStart();
        waitForStart();
        moveOd(0,0.5,0,0.5);

//        if (opModeIsActive()) {
//            resetRuntime();
//            telemetry.addData("Red",color.red());
//            sleep(1000);
//        }
    }






    public void HuskyLens(double time){
        resetRuntime();
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while(opModeIsActive() && !Check && getRuntime()<time) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            HuskyLens.Block[] blocks = huskyLens.blocks();
            //bob = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                x = blocks[i].x;
                y = blocks[i].y;
                if (blocks[i].id ==1){
                    Check = true;
                }

            }

            telemetry.update();

        }

        telemetry.addData("test",x);
        telemetry.addData("test1",y);
        telemetry.update();

    }
    public void createRobot() {


        IMU.Parameters imp;
        imp = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // BNO055IMU.Parameters parameters = new BNO005IMU.Parameters();
        // parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        // parameters.loggingEnabled = true;
        // parameters.loggingTag = "IMU";
        // parameters.acelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        right_pully = hardwareMap.get(DcMotor.class, "pullyR");
        left_pully = hardwareMap.get(DcMotor.class, "pullyL");
        xOd = hardwareMap.get(DcMotor.class, "xOdo");
        yOd = hardwareMap.get(DcMotor.class, "yOdo");
        upServo = hardwareMap.get(Servo.class, "up_servo");
        Push = hardwareMap.get(Servo.class, "Push");
        grabberSecond = hardwareMap.get(Servo.class, "grabberSecond");

        distance = hardwareMap.get(DistanceSensor.class, "Distance");


        driverPID = new PIDcontroller(0.008,0,0.005,60);

        turnPID = new PIDcontroller(0.015575,0,0.056,60);
        movePID = new PIDlinear(0.000075,0.8,0.018,20); //<- motor encoders
        strafePID = new PIDlinear(0.00075,0.9,0.0185,20);
        // movePID = new PIDlinear(0.00014,0.25,0.002,20);
        pidLifter = new PIDlinear(0.00008,0,0.00005,60);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        //List<HuskyLens.Block> myHuskyLensBlocks;
        //HuskyLens.Block myHuskyLensBlock;

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */

    }
//    public void double [] getPosition(){
//        double data = (deltaL-deltaR)/(Tl + Tr);
//        double deltaL = (radius +Tl)*data;
//        double deltaR = (radius - Tl)*data;
//        double deltaB = (radius + Tb)*data;
//        double y = 2*((deltaR/data)+Tr)*(Math.sin(data/2));
//        double x = 2*((deltaB/data)+Tb)*(Math.sin(data/2));
//        double [] coords = {x,y};
//        return coords;
//    }

    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = xOd.getCurrentPosition();
        currentLeftPosition = yOd.getCurrentPosition();
//        currentAuxPosition = .getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        double dtheta = cm_per_tick * (dn2-dn1) / L;
        double dx = cm_per_tick * (dn1 + dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L);

//        double theta = pos.h + (dtheta / 2.0 );
//        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
//        pos.y += dx * Math.cos(theta) + dy * Math.cos(theta);
//        pos.h += dtheta;

    }
    public void imuStart(){
        telemetry.addData("debug", 3.5);
        telemetry.update();
        imp = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        telemetry.addData("debug", 4);
        telemetry.update();
        imu_IMU.initialize(imp);
        telemetry.addData("debug", 5);
        telemetry.update();
        imu_IMU.resetYaw();
        telemetry.addData("debug", 6);
        telemetry.update();
    }
    public double getAvMotorPos(){
        return (Math.abs(back_right_drive.getCurrentPosition())+Math.abs(back_left_drive.getCurrentPosition())+Math.abs(front_right_drive.getCurrentPosition())+Math.abs(front_left_drive.getCurrentPosition()))/4;
    }
    public void strafe(double distanceM, double maxPower){
        resetEncorder();
        double sign = distanceM/Math.abs(distanceM);
        double power;
        double pos = getAvMotorPos();
        double distance = Math.abs(distanceM * matCoEffStrafe);
        while(distance>getAvMotorPos()&&opModeIsActive()){
            power = sign*maxPower*Math.cos(((Math.PI)*(getAvMotorPos()-(.45*distance)))/(distance*1.1))/1.5;
            getYawValue();
            correction = turnPID.update(targetYaw, yaw);
            setMotorPower(-power + correction, power + correction, -power - correction, power - correction);
            lifterWhile();
            addTelemetryMovement(distance);
        }
        setMotorPower(0,0,0,0);
    }
    public void newMove(double distanceM, double maxPower){
        resetEncorder();
        double sign = distanceM/Math.abs(distanceM);
        double power;
        double pos = getAvMotorPos();
        double distance = Math.abs(distanceM * matCoEff);
        while(distance>getAvMotorPos()&&opModeIsActive()){
            power = sign*maxPower*Math.cos(((Math.PI)*(getAvMotorPos()-(.45*distance)))/(distance*1.1))/2;
            getYawValue();
            correction = turnPID.update(targetYaw, yaw);
            setMotorPower(power + correction, power + correction, power - correction, power - correction);
            lifterWhile();
            addTelemetryMovement(distance);
        }
        setMotorPower(0,0,0,0);
    }
    public void turn_right_IMU(double angle, double power) {
        double sign;
        getYawValue();
        targetYaw = angle;
        time1 = getRuntime();
        endTime1 = time1+1.5;
        while (yaw > targetYaw+2.5  && opModeIsActive() && time1<endTime1) {
            time1=getRuntime();
            getYawValue();
            correction = turnPID.update(targetYaw, yaw);
            sign = correction/Math.abs(correction);
            correction = Math.min(Math.abs(correction), power)*sign;
            setMotorPower(correction,correction,-correction,-correction);
            addTelemetryTurn(angle);
        }
        while(getRuntime()<.5 && opModeIsActive()){
            time1=getRuntime();
            getYawValue();
            correction = turnPID.update(targetYaw, yaw);
            sign = correction/Math.abs(correction);
            correction = Math.min(Math.abs(correction), power)*sign;
            setMotorPower(correction,correction,-correction,-correction);
            addTelemetryTurn(angle);
        }
        setMotorPower(0,0,0,0);
        imu_IMU.resetYaw();
        targetYaw = 0;
    }
    public void turn_left_IMU(double angle, double power) {
        double sign;
        getYawValue();
        targetYaw = angle;
        time1 = getRuntime();
        endTime1 = time1+1.5;
        while (yaw < targetYaw+2.5 && opModeIsActive() && time1<endTime1) {
            time1=getRuntime();
            getYawValue();
            correction = turnPID.update(targetYaw, yaw);
            sign = correction/Math.abs(correction);
            correction = Math.min(Math.abs(correction), power)*sign;
            setMotorPower(correction,correction,-correction,-correction);
            addTelemetryTurn(angle);
        }

        resetRuntime();
        while(getRuntime()<.5 && opModeIsActive()){
            time1=getRuntime();
            getYawValue();
            correction = turnPID.update(targetYaw, yaw);
            sign = correction/Math.abs(correction);
            correction = Math.min(Math.abs(correction), power)*sign;
            setMotorPower(correction,correction,-correction,-correction);
            addTelemetryTurn(angle);
        }

        setMotorPower(0,0,0,0);
        imu_IMU.resetYaw();
        targetYaw = 0;
    }
    public void lifterWhile(){

        right_pully.setTargetPosition(targetPos);
        left_pully.setTargetPosition(targetPos);
        right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition())/2;
        right_pully.setPower(1 + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
        left_pully.setPower(1 + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
    }
    public void liftAdjust(int pos){
        left_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPos = pos;

    }
    public void up(int position, double power){
        left_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition())/2;
        PIDlinear pidLifter = new PIDlinear(0.00008,0,0.00005,60);
        while (Math.abs(lifter_target) < position && opModeIsActive()) {
            right_pully.setTargetPosition(position);
            left_pully.setTargetPosition(position);
            right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition())/2;
            right_pully.setPower(power + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
            left_pully.setPower(power + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
        }
    }
    public void down(int position, double power){
        left_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_pully.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition())/2;
        PIDlinear pidLifter = new PIDlinear(0.00008,0,0.00005,60);
        while ((Math.abs(lifter_target) < position) && opModeIsActive()) {
            right_pully.setTargetPosition(-position);
            left_pully.setTargetPosition(-position);
            right_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_pully.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter_target = (right_pully.getCurrentPosition() + left_pully.getCurrentPosition())/2;
            right_pully.setPower(power + pidLifter.update(lifter_target, right_pully.getCurrentPosition()));
            left_pully.setPower(power + pidLifter.update(lifter_target, left_pully.getCurrentPosition()));
        }
    }
    public void grabberOpen(){
        grabberSecond.setPosition(0.12);
        sleep(1000);
    }
    public void grabberClose(){
        grabberSecond.setPosition(0);
        sleep(1000);
    }
    public void grabberUp(){
        upServo.setPosition(0.1);

    }
    public void grabberDown(){
        upServo.setPosition(.85);
    }
    public void LinearServo(){
        Push.setPosition(0.55);
    }
    public void getDistance(){

          distance1 = distance.getDistance(DistanceUnit.CM);
          telemetry.addData("Value: ", distance1);
        if (distance1<10){
            back_right_drive.setPower(.5);
        }
    }
    public void resetEncorder() {
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // front_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // front_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_pully.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_pully.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yOd.setDirection(DcMotor.Direction.REVERSE);
        right_pully.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setMotorPower(double p1, double p2, double p3, double p4) {
        front_right_drive.setPower(p1);
        back_right_drive.setPower(p2);
        back_left_drive.setPower(p3);
        front_left_drive.setPower(p4);
    }
    public void addTelemetryMovement(double position) {
        telemetry.addData("x pos", globalX);
        telemetry.addData("y pos", globalY);
        telemetry.addData("pos", JavaUtil.formatNumber(position, 1));
        telemetry.addData("heading", JavaUtil.formatNumber(yaw, 1));
        telemetry.addData("wheel pos", JavaUtil.formatNumber(getAvMotorPos(), 1));
        telemetry.addData("correction1", correction);
        telemetry.update();
    }
    public void addTelemetryMovement(double position, double posDif) {
        telemetry.addData("pos", JavaUtil.formatNumber(position, 1));
        telemetry.addData("pos dif", JavaUtil.formatNumber(posDif, 1));
        telemetry.addData("heading", JavaUtil.formatNumber(yaw, 1));
        telemetry.addData("wheel pos", JavaUtil.formatNumber(getAvMotorPos(), 1));
        telemetry.addData("x pos", xOd.getCurrentPosition());
        telemetry.addData("y pos", yOd.getCurrentPosition());
        telemetry.addData("correction1", correction);
        telemetry.update();
    }
    public void addTelemetryTurn(double angle) {
        telemetry.addData("key", Double.parseDouble(JavaUtil.formatNumber(angles.getYaw(AngleUnit.DEGREES), 2)));
        telemetry.addData("key", Double.parseDouble(JavaUtil.formatNumber(angle, 2)));
        telemetry.update();
    }
    public void getYawValue() {

        angles = imu_IMU.getRobotYawPitchRollAngles();
        yaw = angleSum(angles.getYaw(AngleUnit.DEGREES),yawOffset);


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
    public double angleSum(double ang1,double ang2){
        double result = ang1+ang2;
        while(result<-180){
            result+=360;
        }
        while(result>180){
            result-=360;
        }
        return result;
    }
    public void calcPos(){
        getYawValue();
        double[] output = rotate(xOd.getCurrentPosition()-lastOdX,yOd.getCurrentPosition()-lastOdY,yaw);
        globalX += output[0];
        globalY += output[1];
        lastOdX = xOd.getCurrentPosition();
        lastOdY = yOd.getCurrentPosition();
    }
    public void moveOd(double xPos, double yPos, double angle,  double maxPower){
        resetEncorder();
        getYawValue();
        targetYaw = angle;
        double yPower,xPower,d;
        double distanceX = Math.abs(xPos * matCoEffOd);
        double distanceY = Math.abs(yPos * matCoEffOd);

        while(opModeIsActive()){
//        while(Math.abs(targetYaw)+2>yaw && distanceY>globalY && distanceX>globalX && opModeIsActive()){
            getYawValue();
            calcPos();
            xPower=movePID.update(distanceX,globalX);
            yPower=movePID.update(distanceY,globalY);

            double[] rotPos = rotate(xPower, yPower, yaw);

            xPower = rotPos[0];
            yPower = rotPos[1];

            correction = turnPID.update(targetYaw, yaw);

            d = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(xPower), Math.abs(yPower))), correction, maxPower));


            front_right_drive.setPower(((yPower - xPower)+ correction) / d);
            back_right_drive.setPower(((yPower + xPower)+ correction) / d);
            back_left_drive.setPower(((yPower - xPower)- correction) / d);
            front_left_drive.setPower(((yPower + xPower)- correction) / d);

            lifterWhile();
            addTelemetryMovement(0);
        }
        setMotorPower(0,0,0,0);
    }
}