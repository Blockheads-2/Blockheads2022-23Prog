package org.firstinspires.ftc.teamcode.swerve.common.kinematics;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHub;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing.AutoTest;
import org.firstinspires.ftc.teamcode.swerve.common.Accelerator;
import org.firstinspires.ftc.teamcode.swerve.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.swerve.common.pid.HeaderControlPID;
import org.firstinspires.ftc.teamcode.swerve.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.SplineMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.TurnMath;

public class SwervePod {
    Constants constants = new Constants();
    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();
    TurnMath turnMath = new TurnMath();
    public HeaderControlPID controlHeader;
    Accelerator accelerator;

    Telemetry telemetry;

    public enum Side{
        RIGHT,
        LEFT
    }
    Side side;

//    GlobalPosSystem.WheelOrientation wheelOrientation;

    //teleop
    private double currentW = 0;
    private double currentR = 0;
    private double optimizedCurrentW = 0;
    private double fieldCentricCurrentW = 0;
    private double robotCentricCurrentW = 0;
    public double controlHeaderReference = 0;
    private boolean initPole = true;

    private double throttle = 0;
    private int direction;
    private int initDirection;
    private double turnAmount = 0;

    private double power = 0;
    public double spinClicksTarget = 0;
    public double rotClicksTarget = 0;

    public boolean slantedCycle = false;

    //auto
    private double distance = 0;
    private double speed = 0;
    private double finalAngle = 0;
    private RevisedKinematics.DriveType driveType = RevisedKinematics.DriveType.NOT_INITIALIZED;

    //both teleop & auto
    SnapSwerveModulePID pid;

    private double[] output = new double[3];

    public SwervePod(int spinDirection, Side side){
        pid = new SnapSwerveModulePID();
        this.direction = spinDirection;
        this.initDirection = spinDirection;
        this.side = side;
        this.accelerator = new Accelerator();
        this.controlHeader = new HeaderControlPID();
    }

    public void grabTelemetry(Telemetry t){
        this.telemetry = t;
    }

    TelemetryPacket packet;
    public void grabDashboard(TelemetryPacket t){
        packet = t;
    }

    public void setPID(double kp, double ki, double kd){
        pid.setTargets(kp, ki, kd);
    }

    public void setPID(double kp, double ki, double kd, boolean snap){
        pid.setTargets(kp, ki, kd);
        pid.setSnap(snap);
    }

    public void setCurrents(double currentW, double currentR){
        this.currentW = currentW;
        this.currentR = currentR;
        this.fieldCentricCurrentW = clamp(this.currentW + this.currentR);
        this.robotCentricCurrentW = currentW;
//        this.currentW = clamp(this.currentW + this.currentR);
    }

    public void setRotClicks(double target){
        turnAmount = wheelOptimization(target, fieldCentricCurrentW);
        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;
    }

    public void robotCentricSetRotClicks(double target){
        turnAmount = wheelOptimization(target, currentW);
        rotClicksTarget = turnAmount * constants.CLICKS_PER_DEGREE;
    }

    public void forceSetRotClicks(int clicks){
        rotClicksTarget = clicks;
        turnAmount = rotClicksTarget * constants.DEGREES_PER_CLICK;
    }

    //for spin power and spin clicks
    public RevisedKinematics.DriveType setSpinClicksAndPower(double powerFactor, double rightTrigger, double leftTrigger, boolean turn, boolean eligibleForTurning, boolean spline, boolean specialSpliningCondition, double rightStickX, double distanceTravelledL, double distanceTravelledR){ //teleop
        this.power = powerFactor;
        if (rightTrigger > 0){
            this.spinClicksTarget = (power * (constants.SPIN_CLICK_FACTOR * (1.0 + (rightTrigger))));
        } else {
            this.spinClicksTarget = power * constants.SPIN_CLICK_FACTOR / (1.0 + (Math.abs(leftTrigger)));
        }
        this.throttle = 1.0;

        if (turn) {
            if (eligibleForTurning) {
                robotCentricSetRotClicks(0);
                if (rightStickX < 0 && side == Side.LEFT) direction *= -1;
                else if (rightStickX >= 0 && side == Side.RIGHT) direction *= -1;

                if (rightTrigger > 0){
                    this.spinClicksTarget = (Math.abs(rightStickX) * (constants.SPIN_CLICK_FACTOR * (1.0 + (rightTrigger))));
                } else {
                    this.spinClicksTarget = Math.abs(rightStickX) * constants.SPIN_CLICK_FACTOR / (1.0 + (Math.abs(leftTrigger)));
                }

                power = rightStickX;

                driveType = RevisedKinematics.DriveType.TURN;
            } else {
                robotCentricSetRotClicks(0);
                spinClicksTarget = 0;
                power = 1.0;
                driveType = RevisedKinematics.DriveType.NOT_INITIALIZED;
            }
//            controlHeader.reset(distanceTravelledL, distanceTravelledR);
//            controlHeaderReference = currentR;

//            driveType = RevisedKinematics.DriveType.TURN;
            return driveType;
        } else if (spline){
            this.spinClicksTarget = (power * constants.SPIN_CLICK_FACTOR * 2);

            double throttle = 1.0 - Math.sin(Math.abs(rightStickX));
            if (Math.abs(currentR) <= 90){
                if (rightStickX < 0 && side == Side.LEFT) this.throttle = throttle;
                else if (rightStickX >= 0 && side == Side.RIGHT) this.throttle = throttle;
            } else {
                if (rightStickX < 0 && side == Side.RIGHT) this.throttle = throttle;
                else if (rightStickX >= 0 && side == Side.LEFT) this.throttle = throttle;
            }

//            if (specialSpliningCondition){
//                int offset = rightStickX < 0 ? -constants.slantedOrientation : constants.slantedOrientation;
//                double target = closerAngle(this.robotCentricCurrentW, 90, -90);
//                target = this.wheelOrientation == GlobalPosSystem.WheelOrientation.FRONT ? target + offset : target - offset;
//                target = clamp(target);
//                robotCentricSetRotClicks(target);
//
//                slantedCycle = true;
//
//                this.throttle = 1;
//            } else {
//                slantedCycle = false;
//            }

//            controlHeaderReference = currentR;
//            controlHeader.reset(distanceTravelledL, distanceTravelledR);

            driveType = RevisedKinematics.DriveType.VARIABLE_SPLINE;
            return driveType;
        } else {

            direction = (initPole ? initDirection : -initDirection);
//            controlHeader.calculateThrottle(distanceTravelledL, distanceTravelledR, currentR, controlHeaderReference);
//            throttle = controlHeader.getThrottle(side);
            if (side == Side.RIGHT) spinClicksTarget *= constants.RIGHT_SIDE_LIMITER; //move this so it only affects it when the robot is translating, not turning.
        }

        if (rightStickX == 0 && power == 0) {
            driveType = RevisedKinematics.DriveType.STOP;
            return driveType;
        }

        driveType = RevisedKinematics.DriveType.LINEAR;
        return driveType;
    }

    public double headerCorrectionTop(double topLToTopR){
        if (side == Side.RIGHT){
            return 1;
        } else { //correcting left to match right, so no need to change the left side.
            return (Math.abs(topLToTopR) > 1 ? 1.0 - (Math.abs(topLToTopR - 1.0)) : 1.0 + (1.0 - Math.abs(topLToTopR)));
        }
    }

    //Left : Right --> 120 %
    //Left : Right --> 90 %

    public double headerCorrectionBottom(double botLToBotR){
        if (side == Side.RIGHT){
            return 1;
        } else { //correcting left to match right, so no need to change the left side.
            return (Math.abs(botLToBotR) > 1 ? 1.0 - (Math.abs(botLToBotR - 1.0)) : 1.0 + (1.0 - Math.abs(botLToBotR)));
        }
    }

    public double wheelOptimization(double target, double currentW){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - clamp(currentW + (initPole ? 0 : 180));
        double turnAmount2 = target2 - clampConventional(current2 + (initPole ? 0 : 180));

        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        if(Math.abs(turnAmount) > 90){
            initPole = !initPole;
            double temp_target = clamp(target + 180);
            turnAmount = changeAngle(temp_target, currentW);
        }

        if (initPole){
            direction = initDirection;
            optimizedCurrentW = fieldCentricCurrentW;
            robotCentricCurrentW = this.currentW;
        } else{
            direction = -initDirection;
            optimizedCurrentW = clamp(fieldCentricCurrentW + 180);
            robotCentricCurrentW = clamp(this.currentW + 180);
        }

        return turnAmount;
    }

    public double closerAngle(double current, double t1, double t2){
        double firstDelta = changeAngle(t1, current);
        double secondDelta = changeAngle(t2, current);
        return Math.abs(firstDelta) >= Math.abs(secondDelta) ? t1 : t2;
    }

    public boolean getPole(){
        return initPole;
    }

    public boolean getSlantedCycle(){
        return slantedCycle;
    }

    public double getOptimizedCurrentW(){return optimizedCurrentW;} //can get rid of optimizedCurrentW and robotCentricW if initPole() thing works
    public double getRobotCentricCurrentW(){return currentW;}

    public void setSpinClicks(int clicks) {
        spinClicksTarget = (double) (clicks);
    }

    public void setThrottle(double throttle){
        this.throttle = throttle;
    }

    public double setPosAuto(double x, double y, double finalAngle, double speed, RevisedKinematics.DriveType driveType, double distanceTravelledL, double distanceTravelledR, double currentW, double currentR){
        setCurrents(currentW, currentR);

        this.accelerator.actuallyAccelerate(true);
        this.accelerator.setAccelFactor(constants.accelTimeAuto);
        this.accelerator.resetAccelerationAuto();

        this.driveType = driveType;
        if (this.driveType == RevisedKinematics.DriveType.SNAP || this.driveType == RevisedKinematics.DriveType.CONSTANT_SPLINE) this.finalAngle = finalAngle;
        else if (this.driveType == RevisedKinematics.DriveType.LINEAR) finalAngle = (initPole ? fieldCentricCurrentW : clamp(fieldCentricCurrentW + 180));
        else this.finalAngle = (initPole ? currentW : clamp(currentW + 180));

        //target position
        this.speed = speed;

        direction = (initPole ? initDirection : -initDirection);

        if (driveType == RevisedKinematics.DriveType.LINEAR){
            linearMath.setPos(x, y, finalAngle);
            distance = Math.abs(linearMath.getDistance());
//            this.controlHeaderReference = this.currentR;
        }
        else if (driveType == RevisedKinematics.DriveType.VARIABLE_SPLINE){
            splineMath.setPos(x, y, finalAngle, (side == Side.RIGHT));
            distance = (side == Side.LEFT ? Math.abs(splineMath.getDistance()[0]) : Math.abs(splineMath.getDistance()[1]));
        }
        else if (driveType == RevisedKinematics.DriveType.TURN){
            turnMath.setPos(finalAngle, currentR, (side == Side.RIGHT));
            distance = Math.abs(turnMath.getDistance());
        }

        if (driveType == RevisedKinematics.DriveType.SNAP) setPID(constants.kpRotation, constants.kiRotation, constants.kdRotation, true);
        else if (driveType == RevisedKinematics.DriveType.TURN) setPID(constants.kpTurning, constants.kiTurning, constants.kdTurning, true);
        else setPID(constants.kpTranlation, constants.kiTranslation, constants.kdTranslation, false);

        packet.put("Target Distance", linearMath.getDistance());

//        controlHeader.reset(distanceTravelledL, distanceTravelledR);

        double powerSpin = Math.abs(pid.update(distance)) * speed;
        double powerRotate = Math.abs(pid.update(turnAmount)) * speed;
        power = (driveType == RevisedKinematics.DriveType.SNAP ? powerRotate : powerSpin);

        return (distance / (speed * constants.MAX_VELOCITY_DT * constants.INCHES_PER_CLICK));
    }

    public void setPower(double power){
        this.power = power;
    }

    public void autoLogic(double distanceRan, double distanceTravelledL, double distanceTravelledR){
        telemetry.addData("acceleration?", accelerator.actuallyAccelerate);
        telemetry.addData("Acceleration factor", accelerator.accelerationFactor);

        //determining distance left for rotating and spinning the module
        switch(driveType){
            case CONSTANT_SPLINE:

            case LINEAR:
                setRotClicks(finalAngle);

                distance = linearMath.distanceRemaining(distanceRan);
                telemetry.addData("distance remaining" + (side == Side.RIGHT ? "R" : "L"), distance);
                telemetry.addData("Target pos" + (side == Side.RIGHT ? "R" : "L"), linearMath.getTargetDistance());


                power = Math.abs(pid.update(distance)) * speed; //probably needs a way to keep the power alive to take into account power directed toward rotating the module.
                throttle = 1.0;
//                controlHeader.calculateThrottle(distanceTravelledL, distanceTravelledR, currentR, controlHeaderReference);
//                throttle = controlHeader.getThrottle(side, specialSpliningCondition);
                telemetry.addData("Header error", controlHeader.error);

                direction = (initPole ? initDirection : -initDirection) * (distance < 0 ? -1 : 1);

                if (side == Side.RIGHT) {
                    distance *= constants.RIGHT_SIDE_LIMITER_AUTO;
                    power *= constants.RIGHT_SIDE_LIMITER_AUTO;
                }

                break;

            case VARIABLE_SPLINE: //using gps for variable_spline may be incredibly unreliable.  Though the nature of clicks (they are integers) gives us an inaccurate account (but this fear depends on the fact that the code loops really really quickly).
                robotCentricSetRotClicks(0);

                distance = splineMath.distanceRemaining(distanceRan);
                direction = (initPole ? initDirection : -initDirection) * (distance < 0 ? -1 : 1);
                power = Math.abs(pid.update(distance)) * speed;
                throttle = splineMath.getThrottle();
                break;

            case SNAP:
                setRotClicks(finalAngle);

                direction = (initPole ? initDirection : -initDirection);
                power = Math.abs(pid.update(turnAmount)) * speed;
                distance = 0;
                throttle = 1.0;

                break;

            case TURN:
                robotCentricSetRotClicks(0);
                direction = (initPole ? initDirection : -initDirection);

                turnMath.setPos(this.finalAngle, currentR, (side == Side.RIGHT));
                telemetry.addData("Target distance", turnMath.getTargetDistance());
                distance = turnMath.distanceRemaining(distanceRan);
                packet.put("Target Distance", turnMath.getDistance());
                throttle = 1;

                power = Math.abs(pid.update(distance)) * speed;
//                if (finalAngle < 0 && side == Side.LEFT) direction *= -1;
//                else if (finalAngle >= 0 && side == Side.RIGHT) direction *= -1;

                break;

            case STOP:
                distance = 0;
                turnAmount = 0;
                power = 0;
                throttle = 1.0;
                rotClicksTarget = 0;
                direction = (initPole ? initDirection : -initDirection);
                accelerator.resetAccelerationAuto();
                break;

        }

        spinClicksTarget = distance * constants.CLICKS_PER_INCH;
    }

    public void setResetValues(){
        robotCentricSetRotClicks(0);
        setSpinClicks(0);
        power = 0.8;
        throttle = 1;
        direction = (initPole ? initDirection : -initDirection);
        driveType = RevisedKinematics.DriveType.RESET;
    }

    public void turn(double finalAngle, double speed){
        robotCentricSetRotClicks(0);

        double turnAmountLeft = SwervePod.changeAngle(finalAngle, currentR);
        distance = Math.toRadians(turnAmountLeft) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;

        direction = (initPole ? initDirection : -initDirection) * (side == Side.RIGHT ? -1 : 1) * (distance < 0 ? -1 : 1);

//        double growthFactorForTurning = turnLogarithmicFunction(turnAmountLeft);
        spinClicksTarget = Math.abs(distance) * constants.CLICKS_PER_INCH;
//        spinClicksTarget *= growthFactorForTurning;

        throttle = 1.0;
        power = pid.update(distance) * speed;

        driveType = RevisedKinematics.DriveType.TURN;
    }

    public double turnLogarithmicFunction(double turnAmountLeft){
        return (1.0 / (1 + Math.pow(Math.E, -turnAmountLeft+3.5))) + 1;
    }

    public static double changeAngle(double target, double current){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (current < 0 ? current + 360 : current);
        double turnAmount1 = target - current;
        double turnAmount2 = target2 - current2;
        return (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;
        if (degrees == -180) degrees = 180;

        if (degrees < -180){
            degrees = 180 - (Math.abs(degrees) - 180);
        } else if (degrees > 180){
            degrees = -180 + (Math.abs(degrees) - 180);
        }
        return degrees;
    }

    public double clampConventional(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < 0){
            degrees = 360 - Math.abs(degrees);
        } else if (degrees >= 360){
            degrees = Math.abs(degrees) - 360;
        }
        return degrees;
    }

    public boolean onlyRotate(boolean firstMovement){
        if (firstMovement){
            if (Math.abs(turnAmount) >= constants.degreeTOLERANCE){
                if (Math.abs(rotClicksTarget) > Math.abs(spinClicksTarget)) power = 1.0;
                return true;
            } else return false;
        }
        return false;
    }

    public boolean rotPriority(){
        return (Math.abs(rotClicksTarget) >= Math.abs(spinClicksTarget));
    }

//    public void setWheelOrientation(GlobalPosSystem.WheelOrientation orientation){
//        this.wheelOrientation = orientation;
//    }

    public RevisedKinematics.DriveType getDriveType(){
        return driveType;
    }

    public double[] getOutput(){
        power *= accelerator.getAccelerationFactor(Math.abs(power) * constants.POWER_LIMITER, turnAmount);
        spinClicksTarget *= accelerator.getAccelerationFactor(Math.abs(spinClicksTarget), turnAmount);

        if (power > constants.POWER_LIMITER) power = constants.POWER_LIMITER;
        else if (power < -constants.POWER_LIMITER) power = -constants.POWER_LIMITER;

        throttle = Math.abs(throttle);
//        power *= throttle;
        spinClicksTarget = Math.abs(spinClicksTarget) * direction * throttle;

        if (Math.abs(rotClicksTarget) > Math.abs(spinClicksTarget)) power = 1;

        output[0] = spinClicksTarget;
        output[1] = rotClicksTarget;
        output[2] = power;

        return output;
    }

    public double[] getOutputAuto(){
        if (driveType != RevisedKinematics.DriveType.SNAP && driveType != RevisedKinematics.DriveType.RESET) {
            power *= accelerator.getAccelerationFactor();
            spinClicksTarget *= accelerator.getAccelerationFactor();
        }

        if (power > constants.POWER_LIMITER) power = constants.POWER_LIMITER;
        else if (power < -constants.POWER_LIMITER) power = -constants.POWER_LIMITER;

        throttle = Math.abs(throttle);
//        power *= throttle;
        spinClicksTarget = Math.abs(spinClicksTarget) * direction * throttle;

        output[0] = spinClicksTarget;
        output[1] = rotClicksTarget;
        output[2] = power;

        return output;
    }

    public Accelerator getAccelerator(){
        return accelerator;
    }

    public int getSpinDirection(){
        return direction;
    }

    public SnapSwerveModulePID getPID(){
        return pid;
    }

    public double getTurnAmount(){
        return turnAmount;
    }

    public double getDistance(){
        return distance;
    }

    public double getThrottle(){
        return throttle;
    }

}
