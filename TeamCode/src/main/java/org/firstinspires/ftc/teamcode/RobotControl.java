package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Movement.RoboticMovement;
import org.firstinspires.ftc.teamcode.Movement.PIDcontroller;

@Autonomous(name="Sean Test")
public class RobotControl extends LinearOpMode {
    private void createRobot(RoboticMovement Robot) {
        Robot.back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        Robot.back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        Robot.front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        Robot.front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        Robot.imu_IMU = hardwareMap.get(IMU.class, "imu");
        Robot.driverPID = new PIDcontroller(0.008,0,0.005,60);
    }
    @Override
    public void runOpMode() {
        waitForStart();
        RoboticMovement Robot = new RoboticMovement();
        createRobot(Robot);
        Robot.forward(1000,0.5);
        Robot.backward(-1000, -0.5);
        Robot.turn_right_IMU(90, 0.2);
        Robot.strafe_left(1000, 0.5);
        Robot.strafe_right(-1000, 0.5);
        Robot.turn_left_IMU(-90, 0.2);
    }


}

