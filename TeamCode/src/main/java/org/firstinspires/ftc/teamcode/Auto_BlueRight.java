package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto BlueRight")
public class Auto_BlueRight extends RobotControl_Master {
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
