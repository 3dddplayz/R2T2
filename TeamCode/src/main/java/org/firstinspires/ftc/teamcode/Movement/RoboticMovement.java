package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotControl;

public class RoboticMovement extends RobotControl {

    public DcMotor back_left_drive;
    public DcMotor back_right_drive;
    public DcMotor front_left_drive;
    public DcMotor front_right_drive;
    public IMU imu_IMU;
    public PIDcontroller driverPID;
    int targetYaw = 0;
    YawPitchRollAngles angles;
    double yaw;
    double correction;
    public void forward(int position, double power) {
        resetEncorder();
        while (back_left_drive.getCurrentPosition() < position) {
            getYawValue();
            correction = driverPID.update(targetYaw, yaw);
            setMotorPower(power + correction, power + correction, power - correction, power - correction);
            addTelemetryMovement(position);
        }
        setMotorPower(0,0,0,0);
    }
    public void backward(int position, double power) {
        resetEncorder();
        while (back_left_drive.getCurrentPosition() > position && opModeIsActive()) {
            getYawValue();
            correction = driverPID.update(targetYaw, yaw);
            setMotorPower(power + correction, power + correction, power - correction, power - correction);
            addTelemetryMovement(position);
        }
        setMotorPower(0,0,0,0);
    }
    public void strafe_right(int position, double power) {
        resetEncorder();
        while (back_left_drive.getCurrentPosition() > position && opModeIsActive()) {
            getYawValue();
            correction = driverPID.update(targetYaw, yaw);
            setMotorPower(-power + correction, -power + correction, -power - correction, -power - correction);
            getYawValue();
            addTelemetryMovement(position);
        }
        setMotorPower(0,0,0,0);
    }
    public void strafe_left(int position, double power) {
        resetEncorder();
        while (back_left_drive.getCurrentPosition() < position && opModeIsActive()) {
            getYawValue();
            correction = driverPID.update(targetYaw, yaw);
            setMotorPower(power + correction, -power + correction, power - correction, -power - correction);
            addTelemetryMovement(position);
        }
        setMotorPower(0,0,0,0);
    }
    public void turn_right_IMU(int angle, double power) {
        targetYaw = angle;
        while (opModeIsActive() && yaw > angle) {
            addTelemetryTurn(angle);
            getYawValue();
            setMotorPower(-power,-power,power,power);
        }
        setMotorPower(0,0,0,0);
    }
    public void turn_left_IMU(int angle, double power) {
        targetYaw = angle;
        while (opModeIsActive() && yaw < angle) {
            addTelemetryTurn(angle);
            getYawValue();
            setMotorPower(power,power,-power,-power);
        }
        setMotorPower(0,0,0,0);
    }
    private void resetEncorder() {
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setMotorPower(double p1, double p2, double p3, double p4) {
        front_right_drive.setPower(p1);
        back_right_drive.setPower(p2);
        back_left_drive.setPower(p3);
        front_left_drive.setPower(p4);
    }
    private void addTelemetryMovement(int position) {
        telemetry.addData("pos", JavaUtil.formatNumber(position, 1));
        telemetry.addData("heading", JavaUtil.formatNumber(yaw, 1));
        telemetry.addData("wheel pos", JavaUtil.formatNumber(back_left_drive.getCurrentPosition(), 1));
        telemetry.update();
    }
    private void addTelemetryTurn(int angle) {
        telemetry.addData("key", Double.parseDouble(JavaUtil.formatNumber(angles.getYaw(AngleUnit.DEGREES), 2)));
        telemetry.addData("key", Double.parseDouble(JavaUtil.formatNumber(angle, 2)));
        telemetry.update();
    }
    private void getYawValue() {
        angles = imu_IMU.getRobotYawPitchRollAngles();
        yaw = angles.getYaw(AngleUnit.DEGREES);
    }
}
