package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.tests.ShooterTest.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.Poses;
import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.Objects;

public class Shooter implements Subsystem {
    private final DcMotorEx fly1;
    private final DcMotorEx fly2;

    private final DcMotorEx encoder;
    private final Servo hoodServo;

    private final Servo turret1;

    private final Servo turret2;

    private final MultipleTelemetry telemetry;

    private boolean telemetryOn = false;

    private double manualPower = 0.0;
    private double hoodPos = 0.0;

    private double turretPos = 0.0;
    private double velocity = 0.0;
    private double posPower = 0.0;

    public double velo = 0.0;

    private int targetPosition = 0;

    public double powPID = 1.0;

    private double p = 0.0003, i = 0, d = 0.00001;

    private PIDFController controller;
    private double pow = 0.0;

    private String shooterMode = "AUTO";

    private String turretMode = "AUTO";

    public Shooter(Robot robot, MultipleTelemetry TELE) {
        this.fly1 = robot.shooter1;
        this.fly2 = robot.shooter2;
        this.telemetry = TELE;
        this.hoodServo = robot.hood;

        // Reset encoders
        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(p, i, d, f);

        controller.setPIDF(p, i, d, f);

        this.turret1 = robot.turr1;

        this.turret2 = robot.turr2;

        this.encoder = robot.shooterEncoder;

    }

    public double gethoodPosition() {
        return (hoodServo.getPosition());
    }

    public void sethoodPosition(double pos) { hoodPos = pos; }

    public double getTurretPosition() {
        return ((turret1.getPosition() + (1 - turret2.getPosition())) / 2);
    }

    public void setTurretPosition(double pos) { turretPos = pos; }

    public double getVelocity(double vel) {
        return vel;
    }

    public void setVelocity(double vel) { velocity = vel; }

    public void setPosPower(double power) { posPower = power; }

    public void setTargetPosition(int pos) {
        targetPosition = pos;
    }

    public void setTolerance(int tolerance) {
        controller.setTolerance(tolerance);
    }

    public void setControllerCoefficients(double kp, double ki, double kd, double kf) {
        p = kp;
        i = ki;
        d = kd;
        f = kf;
        controller.setPIDF(p, i, d, f);

    }

    public PIDCoefficients getControllerCoefficients() {

        return new PIDCoefficients(p, i, d);

    }

    public void setManualPower(double power) { manualPower = power; }

    public String getShooterMode() { return shooterMode; }

    public String getTurretMode() { return turretMode; }

    public double getECPRPosition() {
        return fly1.getCurrentPosition() / (2 * ecpr);
    }

    public double getMCPRPosition() {
        return fly1.getCurrentPosition() / (2 * mcpr);
    }

    public void setShooterMode(String mode) { shooterMode = mode; }

    public void setTurretMode(String mode) { turretMode = mode; }

    public double trackGoal(Pose2d robotPose, Pose2d goalPose, double offset) {

        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d deltaPose = new Pose2d(
                goalPose.position.x - robotPose.position.x,
                goalPose.position.y - robotPose.position.y,
                goalPose.heading.toDouble() - (robotPose.heading.toDouble())
        );

        double distance = Math.sqrt(
                deltaPose.position.x * deltaPose.position.x
                + deltaPose.position.y * deltaPose.position.y
                + Poses.relativeGoalHeight * Poses.relativeGoalHeight
        );

        telemetry.addData("dst", distance);

        double shooterPow = getPowerByDist(distance);

        double hoodAngle = getAngleByDist(distance);

//        hoodServo.setPosition(hoodAngle);

        moveTurret(getTurretPosByDeltaPose(deltaPose, offset));

        return distance;

        //0.9974 * 355

    }

    public double getTurretPosByDeltaPose(Pose2d dPose, double offset) {

        double deltaAngle = Math.toDegrees(dPose.heading.toDouble());

        double aTanAngle = Math.toDegrees(Math.atan(dPose.position.y / dPose.position.x));

        telemetry.addData("deltaAngle", deltaAngle);

        if (deltaAngle > 90) {
            deltaAngle -= 360;
        }

//        deltaAngle += aTanAngle;

        deltaAngle /= (335);

        telemetry.addData("dAngle", deltaAngle);

        telemetry.addData("AtanAngle", aTanAngle);

        return ((0.30 - deltaAngle) + offset);

    }

    //62, 0.44

    //56.5, 0.5

    public double getPowerByDist(double dist) {

        //TODO: ADD LOGIC
        return dist;
    }

    public double getAngleByDist(double dist) {

        double newDist = dist - 56.5;

        double pos = newDist * ((0.44 - 0.5) / (62 - 56.5)) + 0.46;

        return pos;
    }

    public void setTelemetryOn(boolean state) { telemetryOn = state; }

    public void moveTurret(double pos) {
        turret1.setPosition(pos);
        turret2.setPosition(1 - pos);
    }

    public double getpowPID() {
        return powPID;
    }

    @Override
    public void update() {

        if (Objects.equals(shooterMode, "MANUAL")) {
            fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            fly1.setPower(manualPower);
            fly2.setPower(manualPower);
        } else if (Objects.equals(shooterMode, "VEL")) {
            powPID = velocity;

            fly1.setPower(powPID);
            fly2.setPower(powPID);

        }

    }
}
