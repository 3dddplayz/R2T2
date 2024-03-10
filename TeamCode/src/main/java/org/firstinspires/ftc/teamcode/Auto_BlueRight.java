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
        if (opModeIsActive()) {
            HuskyLens(2);
            paddleClose();
            grabberOpen();
            grabberUp();

            liftAdjust(0);

            if (x>220){
                moveOd(-.3, -1.5, 0, 1);
                paddleOpen();
                moveOd(-.3, -2.2, 0, 1);
                moveOd(2.5, -2.2, 0, 1);
                pause(20);
                liftAdjust(1000);
                moveOd(3.68, -1.3, 90, 1);
                grabberClose();
                sleep(500);
                liftAdjust(-1000);
                moveOd(3.5, -1.3, 90, 1);
            } else if (x>100){
                moveOd(0,-1.8,0,1);
        paddleOpen();
        moveOd(0,-2.2,0,1);
        moveOd(2.5,-2.2,0,1);
        liftAdjust(1000);
        pause(19);
        moveOd(3.5,-1,90,1);
        moveOd(3.68,-1.1,90,1);
        grabberClose();
        sleep(500);
        liftAdjust(-1000);
        moveOd(3.5,-1.1,90,1);
            } else {
                moveOd(0,-1.15,90,1);
        moveOd(.2,-1.15,90,1);
        paddleOpen();
        moveOd(-.2,-1.15,90,1);
        moveOd(0,-2.2,0,1);
        moveOd(2.5,-2.2,0,1);
        pause(19);
        liftAdjust(1000);
        grabberUp();
        moveOd(3.64,-.9,90,1);
        grabberClose();
        sleep(500);
        liftAdjust(-1000);
        moveOd(3.5,-.9,90,1);
            }

        }
    }
}