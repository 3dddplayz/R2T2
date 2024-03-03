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
            paddleClose();
            grabberOpen();
            grabberUp();
        moveOd(-.3,-1.5,0,1);
        paddleOpen();
        moveOd(-.3,-2.25,0,1);
        sleep(1000);
        moveOd(2.5,-2.25,0,1);
        liftAdjust(1000);
        moveOd(3.5,-1.2,90,1);
        moveOd(3.6,-1.2,90,1);
        grabberClose();
        sleep(500);
        moveOd(3.5,-1.2,90,1);
        moveOd(3.5,0,90,1);
            //right
//            liftAdjust(1000);
//        moveOd(1,-1.2,90,1);
//        moveOd(1.47,-1.2,90,1);
//        grabberClose();
//        sleep(500);
//        liftAdjust(-1000);
//        moveOd(1,-1.1,90,1);
//        moveOd(-.18,-1.15,-90,1);
//        paddleOpen();
//        sleep(500);
//        moveOd(1,-1.15,-90,1);
//        moveOd(1.45,0,-90,1);
//        //left
//        moveOd(1,-.8,90,1);
//        moveOd(1.47,-.8,90,1);
//        grabberClose();
//        sleep(500);
//        liftAdjust(-1000);
//        moveOd(1,-.8,90,1);
//        moveOd(.65,-1.2,-90,1);
//        paddleOpen();
//        sleep(500);
//        moveOd(1,-1,-90,1);
//        moveOd(1.45,0,90,1);
        //middle
//        moveOd(1,-1,90,1);
//        moveOd(1.47,-1,90,1);
//        grabberClose();
//        sleep(1000);
//        liftAdjust(-1000);
//        moveOd(0,-1.2,180,1);
//        paddleOpen();
//        sleep(500);
//        moveOd(0,-.5,180,1);
//        moveOd(1.45,0,90,1);


    }
}
