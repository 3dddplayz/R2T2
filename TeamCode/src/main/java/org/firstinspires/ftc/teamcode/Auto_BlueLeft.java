package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto BlueLeft")
public class Auto_BlueLeft extends RobotControl_Master {
    @Override
    public void runOpMode() {
        createRobot();
        resetEncorder();
        imuStart();
        waitForStart();
        if(opModeIsActive()){
            HuskyLens(2);
            paddleClose();
        grabberOpen();
        grabberUp();
            liftAdjust(1000);
        if (x>250 || !Check){
            moveOd(0, -1.2, -90, 1);
            moveOd(-.23, -1.15, -90, 1);
            paddleOpen();
            sleep(500);
            moveOd(0, -1.15, -90, 1);
            moveOd(1.55, -1.3, 90, .8);
            grabberClose();
            sleep(500);
            liftAdjust(-1000);
            moveOd(1.4, -1.3, 90, .8);
            moveOd(1.4,.1, -90, 1);
        } else if (x>150){
            moveOd(0,-1.25,180,1);
            paddleOpen();
            sleep(500);
            moveOd(0,-.7,180,1);
            moveOd(1.525,-1,90,1);
            grabberClose();
            sleep(1000);
            liftAdjust(-1000);
            moveOd(1.4,-1,90,1);
            moveOd(1.4,.1,90,1);
        } else {
            moveOd(.65,-1.2,-90,1);
            paddleOpen();
            moveOd(1,-1.2,-90,1);
            moveOd(1.55,-.8,90,.8);
            grabberClose();
            sleep(500);
            liftAdjust(-1000);
            moveOd(1.4,-.8,90,.8);
            moveOd(1.4,.1,-90,1);
        }

//
        }
    }
}
