package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOpNewDrive;

public class Odometry {
    double xOdRad = 13;
    double yROdRad = 19;
    double yLOdRad = 19;

    double[] lastGLobalPos = {0,0};
    double lastYaw = 0;
    double lastResetYaw = 0;
    public Odometry(){

    }

}