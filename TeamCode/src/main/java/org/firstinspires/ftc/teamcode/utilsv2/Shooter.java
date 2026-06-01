package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.utils.Robot;

public class Shooter {

    Robot robot;
    Flywheel fly;
    Turret turr;
    VelocityCommander commander;

    double goalX = 0.0;
    double goalY = 0.0;
    double obeliskX = 72;
    double obeliskY = 144;

    private boolean red = true;


    Follower follow;

    public Shooter(Robot rob, MultipleTelemetry TELE, Follower follower, boolean redAlliance) {
        this.robot = rob;
        this.fly = new Flywheel(rob);
        this.turr = new Turret(rob);
        this.follow = follower;
        this.commander = new VelocityCommander();

        setRedAlliance(redAlliance);

        if (redAlliance) {
            goalX = 144;
            goalY = 144;
        } else {
            goalX = 0;
            goalY = 144;
        }
    }

    public void setRedAlliance(boolean input) {
        this.red = input;
    }

    private double flywheelVelocity = 0.0;
    private double turretPosition = 0.5;

    enum ShooterState {
        READ_OBELISK,
        TRACK_GOAL,
        MANUAL_FLYWHEEL_TRACK_TURR,
        MANUAL_TURRET_TRACK_FLY,
        MANUAL,
        NOTHING
    }


    private ShooterState state = ShooterState.NOTHING;

    public void setState(ShooterState shooterState) {
        this.state = shooterState;
    }

    public void setTurretPosition(double input) {
        this.turretPosition = input;
    }

    public void setFlywheelVelocity(double input) {
        this.flywheelVelocity = input;
    }

    public int getObeliskID() {
        return turr.getObeliskID();
    }


    public void update() {

        switch (state) {
            case NOTHING:
                break;
            case MANUAL:
                fly.manageFlywheel(flywheelVelocity);
                turr.manual(turretPosition);
                break;
            case TRACK_GOAL:
                turr.trackGoal(
                        (follow.getPose().getX() - goalX),
                        (follow.getPose().getY() - goalY),
                        follow.getHeading(),
                        follow.getAngularVelocity(),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                flywheelVelocity = commander.getVeloPredictive(
                        (follow.getPose().getX() - goalX),
                        (follow.getPose().getY() - goalY),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                fly.manageFlywheel(flywheelVelocity);
                break;
            case READ_OBELISK:
                turr.trackObelisk(
                        (follow.getPose().getX() - goalX),
                        (follow.getPose().getY() - goalY),
                        follow.getHeading()
                );

                flywheelVelocity = commander.getVeloPredictive(
                        (follow.getPose().getX() - goalX),
                        (follow.getPose().getY() - goalY),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                fly.manageFlywheel(flywheelVelocity);
                break;

            case MANUAL_TURRET_TRACK_FLY:
                turr.manual(turretPosition);
                flywheelVelocity = commander.getVeloPredictive(
                        (follow.getPose().getX() - goalX),
                        (follow.getPose().getY() - goalY),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );

                fly.manageFlywheel(flywheelVelocity);
                break;

            case MANUAL_FLYWHEEL_TRACK_TURR:
                turr.trackGoal(
                        (follow.getPose().getX() - goalX),
                        (follow.getPose().getY() - goalY),
                        follow.getHeading(),
                        follow.getAngularVelocity(),
                        follow.getVelocity().getXComponent(),
                        follow.getAcceleration().getXComponent(),
                        follow.getVelocity().getYComponent(),
                        follow.getAcceleration().getYComponent()
                );
                fly.manageFlywheel(flywheelVelocity);
                break;

        }

    }

}
