package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Inverse K")
public class InverseK extends LinearOpMode {

    DcMotor arm, base; // motors that move the shoulder and hips
    private CRServo twist; // wrist
    private Servo grabber1, grabber2, grab; // moves the elbow
    final double a1 = 10; // first arm(shoulder)
    final double a2 = 10.5; // second arm(elbow)





    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


        base = hardwareMap.get(DcMotor.class, "base");
        arm = hardwareMap.get(DcMotor.class, "arm");

        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        twist = hardwareMap.get(CRServo.class, "twist");
        grab = hardwareMap.get(Servo.class, "grab");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        base.setTargetPosition(0);
        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double elbowPos = 0;

        // configurations for robot


        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            // calling functions to perform "surgery"
            moveToPos(1,5);
            sleep(1000);
            moveToPos(1,0);
            sleep(1000);
           base.setTargetPosition(100);
            base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            base.setPower(.5);
            moveToPos(1,5);
            sleep(1000);
            moveToPos(1,0);


            while (opModeIsActive()) {



                telemetry.addData("x", x);
                telemetry.addData("y", y);

                telemetry.addData("base pos", base.getCurrentPosition());
                telemetry.addData("base pos", base.getTargetPosition());

                telemetry.addData("arm pos", arm.getCurrentPosition());
                telemetry.addData("arm pos", arm.getTargetPosition());
                telemetry.update();

                // printing out data using encoder cables
            }
        }


    }

        public void moveToPos(double x, double y){ // inverse kinematics function
            double d = Math.sqrt(Math.pow(x,2)+Math.pow(y,2)); // distance from joint in elbow and shoulder
            double law = (Math.pow(a1,2)+Math.pow(a2,2)-Math.pow(d,2))/(2*a1*a2);
            double alpha = Math.acos(law); // angle of point between the two arms
            double q2 = Math.PI - Math.acos((Math.pow(a1,2)+Math.pow(a2,2)-Math.pow(x,2)-Math.pow(y,2))/(2*a1*a2)); // change in elbow angle(radians) to go to endpoint
            double q1 = Math.atan(y/x)+Math.atan((a2*Math.sin(q2))/(a1+a2*Math.cos(q2))); // change in shoulder angle(radians) to go to endpoint
            double elbow = Math.max(Math.min((q2*180/Math.PI)/180,1),0); // convert radians to [0,1] servo position
            double shoulder = (q1*180/Math.PI)*4; // convert radians to ticks
            arm.setTargetPosition((int)shoulder); // sets motor position to the correct
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // tells motor to go to given target position
            arm.setPower(.5); // gives motor power

            telemetry.addData("q1", q1);
            telemetry.addData("elbow", elbow);
            telemetry.addData("shoulder", shoulder);
            telemetry.addData("alpha", alpha);

            // data printing out values for each servo and motor

        }


    }


