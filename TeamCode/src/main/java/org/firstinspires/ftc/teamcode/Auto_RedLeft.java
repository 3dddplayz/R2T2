package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto RedLeft")
public class Auto_RedLeft extends RobotControl_Master {
    @Override
    public void runOpMode() {
        createRobot();
        resetEncorder();
        imuStart();
        waitForStart();
        if(opModeIsActive()){
            moveOd(3,0,0,1);
            moveOd(0,0,0,1);
        }
    }
}
