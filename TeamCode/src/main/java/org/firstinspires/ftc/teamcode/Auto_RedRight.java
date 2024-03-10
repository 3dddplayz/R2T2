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
        if (opModeIsActive()) {
            HuskyLens(2);
            paddleClose();
            grabberOpen();
            grabberUp();
            liftAdjust(1000);
            //right
            if (x > 200) {

                moveOd(-.65,-1.2,90,1);
                paddleOpen();
                moveOd(-1,-1.2,90,1);
                moveOd(-1.55,-.8,-90,.8);
                grabberClose();
                sleep(500);
                liftAdjust(-1000);
                moveOd(-1.4,-.8,-90,.8);
                moveOd(-1.4,.1,90,1);
            } else if (x > 100) {
                //middle
                moveOd(0,-1.2,-180,1);
                paddleOpen();
                sleep(500);
                moveOd(0,-.7,-180,1);
                moveOd(-1.525,-1,-90,1);
                grabberClose();
                sleep(1000);
                liftAdjust(-1000);
                moveOd(-1.4,-1,-90,1);
                moveOd(-1.4,.1,-90,1);
            } else {
                //left
                moveOd(0, -1.2, 90, 1);
                moveOd(.23, -1.15, 90, 1);
                paddleOpen();
                sleep(500);
                moveOd(0, -1.15, 90, 1);
                moveOd(-1.55, -1.3, -90, .8);
                grabberClose();
                sleep(500);
                liftAdjust(-1000);
                moveOd(-1.4, -1.3, -90, .8);
                moveOd(-1.4,.1, 90, 1);

            }
        }
    }
}