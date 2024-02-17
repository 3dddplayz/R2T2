package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto RedRight")
public class Auto_RedRight extends RobotControl_Master {
    @Override
    public void runOpMode() {
        createRobot();
        resetEncorder();
        imuStart();
        waitForStart();
        if(opModeIsActive()){
            addTelemetryMovement(10);
        }
    }
}
