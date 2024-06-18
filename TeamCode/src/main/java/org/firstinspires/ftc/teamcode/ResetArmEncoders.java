package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Reset Arm Encoders")
public class ResetArmEncoders extends LinearOpMode {

    DcMotor arm, base;
    private CRServo twist;
    private Servo grabber1, grabber2, grab;



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

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double elbowPos = 0;

        waitForStart();
        if (opModeIsActive()) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // resets the 0 position of all the motors

    }
}

