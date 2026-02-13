package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SharkDrive {
//    SwerveSubsystem dt = new SwerveSubsystem();
    WorkingSwerve dt = new WorkingSwerve();
    AprilTagLimelight limelight = new AprilTagLimelight();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime dxTime = new ElapsedTime();
    ElapsedTime dyTime = new ElapsedTime();
    ElapsedTime dhTime = new ElapsedTime();
    ElapsedTime dihTime = new ElapsedTime();

    SparkFunOTOS odometry;
    SparkFunOTOS.Pose2D pos;
    SparkFunOTOS.Pose2D lastLimelightPosition = new SparkFunOTOS.Pose2D();

    double lastValidIMUReading = 0;
    double dihP = 0.1;
    double dihI = 0;
    double dihD = 0.0145;
    double previousDihError = 0;
    double lastHeadinerorrthingy = 0;
    double dihband = 5;

    boolean[] completedBools = new boolean[3];
    boolean[] completedStopBools = new boolean[3];

    double deltaTime, last_time;
    double integralX, integralY, integralH = 0;
    double kix, kiy = 0;
    double iX, iY, iH, pX, dX;
    double xAverage, yAverage, hAverage = 0;
    double dihAverage;// 3 in flacid, 5.5 erect
    double[] output = new double[3];
    double[] errors = new double[3];
    double[] previous = new double[3];

    double maximumOutputX = 1;
    double maximumOutputY = 1;

    double diagonalScalar = 0;
    double angleLenience = 60;

    double autograbZeroX = 10;
    double autograbZeroY = 0;
    double integralDih = 0;
    boolean lostSight = false;
    public int countingTelemetry = 0;

    double odometryInputMixPercentage = 0.7;

    public void init(HardwareMap hwMap, boolean isAuton) {
        last_time = 0;
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.calibrateImu();
        odometry.setAngularScalar(1);
        odometry.setLinearScalar(1);
        odometry.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));

        if (isAuton) {
            odometry.resetTracking();
            odometry.begin();
        }

        dt.init(hwMap);

        dt.swerveDrive(0,0,0);
    }

    private double LowPass(double average, double newValue) {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    // maybe derivative not needed as velocity becomes near linear
    public double pid(double error, int index, double distanceLenience) {
        double output = 0;
        if (index == 0) {
            integralX += error * dxTime.seconds();

            if (previous[0] * error < 0) {
                integralX = 0;
            }
            iX = integralX;

            double derivative = (error - previous[0]) / dxTime.seconds();
            derivative = LowPass(xAverage, derivative);

//            output = kpx * error + kix * integralX + kdx * derivative;
            output = SampleConstants.KPX * error + kix * integralX + SampleConstants.KDX * derivative;

            dxTime.reset();
            previous[0] = error;

            NewHighestOutputX(Math.abs(output));
            output = Range.clip(output, -1, 1); // old coef 2*
            if (Math.max(Math.abs(maximumOutputX), Math.abs(maximumOutputY)) == Math.abs(maximumOutputY) && error > Math.abs(0.5)) {
                output = DiagonalScalar(Math.abs(maximumOutputX), Math.abs(maximumOutputY), 0.35) * output / Math.abs(output);
            }
        } else if (index == 1) {
            integralY += error * dyTime.seconds();

            if (previous[1] * error < 0) {
                integralY = 0;
            }

            double derivative = (error - previous[1]) / dyTime.seconds();
            derivative = LowPass(yAverage, derivative);
            iY = integralY;
            pX = SampleConstants.KPY * error;
            dX = derivative;

//            output = kpy * error + kiy * integralY + kdy * derivative;
            output = SampleConstants.KPY * error + kiy * integralY + SampleConstants.KDY * derivative;

            dyTime.reset();

            previous[1] = error;
            NewHighestOutputY(Math.abs(output));
            output = Range.clip(output, -1, 1); // old coef 2*

            if (Math.max(Math.abs(maximumOutputX), Math.abs(maximumOutputY)) == Math.abs(maximumOutputX) && error > Math.abs(0.5)) {
                output *= DiagonalScalar(Math.abs(maximumOutputX), Math.abs(maximumOutputY), 0.3) * output / Math.abs(output);
            }
        } else if (index == 2){
            integralH += error * dhTime.seconds();

            if (previous[2] * error < 0) {
                integralH = 0;
            }

            double derivative = (error - previous[2]) / dhTime.seconds();
            derivative = LowPass(hAverage, derivative);

            iH = integralH;

            output = SampleConstants.KPH * error + SampleConstants.KIH * integralH + SampleConstants.KDH * derivative;
            dhTime.reset();

            previous[2] = error;

            output = Range.clip(output, -1, 1); // old coef 2*
        } else {

            integralDih += error * dihTime.seconds();

            double dihrivative = (error - previousDihError) / dihTime.seconds();
            //lowpass????? idk
            dihrivative = LowPass(dihAverage, dihrivative);
            dihTime.reset();




            previousDihError = error;
            output = dihP * error + dihI * integralDih + dihD * dihrivative;
            output = Range.clip(output, -1, 1);
        }
        return output;
    }

//    public double basicpid(double error, int index) {
//        double output = 0;
//        if (index == 0) {
//            integralX += error * dxTime.seconds();
//
//            if (previous[0] * error < 0) {
//                integralX = 0;
//            }
//
//            double derivative = (error - previous[0]) / dxTime.seconds();
//            derivative = LowPass(xAverage, derivative);
//
//            output = kpx * error + kix * integralX + kdx * derivative;
//            dxTime.reset();
//            previous[0] = error;
//        } else if (index == 1) {
//            integralY += error * dyTime.seconds();
//
//            if (previous[1] * error < 0) {
//                integralY = 0;
//            }
//
//            double derivative = (error - previous[1]) / dyTime.seconds();
//            derivative = LowPass(yAverage, derivative);
//
//            output = kpy * error + kiy * integralY + kdy * derivative;
//            dyTime.reset();
//
//            previous[1] = error;
//        } else {
//            integralH += error * dhTime.seconds();
//
//            if (previous[2] * error < 0) {
//                integralH = 0;
//            }
//
//            double derivative = (error - previous[2]) / dhTime.seconds();
//            derivative = LowPass(hAverage, derivative);
//
//            output = kph * error + kih * integralH + kdh * derivative;
//            dhTime.reset();
//
//            previous[2] = error;
//        }
//        output = Range.clip(output, -1, 1); // old coef 2*
//        return output;
//    }

    public void OdometryControl(double speed, double tgtX, double tgtY, double tgtRot, double distanceLenience, int axis) {
        if (!odometry.isConnected()) {
            return;
        }
//        distanceLenience; //best value 1.75

        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;

//        pos = limelight.GetLimelightData(false, GetOdometryLocalization().h);
//        pos = GetLocalization();

//        pos = PoseEstimator();

        pos = GetOdometryLocalization();

        if (axis == 3 || axis == 4) {
            angleLenience = 15;
        } else {
            angleLenience = 60;
        }

        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)));

        completedBools[2] = Math.abs(errors[2]) < angleLenience;

        errors[2] /= 10; // err crunch tunable

//        if (new ArmLiftMotor().GetLocalNeutral() == 1250) {
//            TuningUp();
//        } else {
//            TuningDown();
//        }

        // normalized against one another
        // should create weird diagonal movement
        // might have to add increased magnitude to error, currently between -1 and 1
        output[0] = pid(errors[0], 0, distanceLenience);
        output[1] = pid(errors[1], 1, distanceLenience);
        output[2] = pid(errors[2], 2, angleLenience);

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;

        completedStopBools[0] = Math.abs(errors[0]) < 0.6;
        completedStopBools[1] = Math.abs(errors[1]) < 0.6;

        if (axis == 0) {
//            output[1] = output[1] / Math.abs(output[1]) * 0.2;
            output[1] *= 0;
//            output[2] *= 0;
            completedBools[1] = true;
        } else if (axis == 1) {
            output[0] *= 0;
            completedBools[0] = true;
        }
        if (tgtRot == 1){
            completedBools[2] = true;
            output[2] = 0;
        }

        if (axis == 4)
        {
            completedBools[0] = true;
            completedBools[1] = true;
            output[0] = 0;
            output[1] = 0;
        }

//        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], GetOrientation());
        dt.swerveDrive(speed * output[1], speed * output[0], -speed * output[2]);
    }

    // creates odometry fallback if the limelight stops working
//    private SparkFunOTOS.Pose2D GetLocalization() {
//        SparkFunOTOS.Pose2D limelightPos = limelight.GetLimelightData(false, GetImuReading());
//        if (limelight.GetIsValid()) {
//            lostSight = false;
//            return limelightPos;
//        } else {
//            if (!lostSight) {
//                lastValidIMUReading = GetImuReading() + lastValidIMUReading;
//                Rehome();
//                lastLimelightPosition = limelight.GetLastPosition();
//                lostSight = true;
//            }
//            return GetOdometryLocalization();
//        }
//    }


//    private SparkFunOTOS.Pose2D PoseEstimator()
//    {
//        SparkFunOTOS.Pose2D output = new SparkFunOTOS.Pose2D();
//        SparkFunOTOS.Pose2D limelightPosition = limelight.GetLimelightData(false, GetOrientation());
//
//        if (limelightPosition.x == 0 && limelightPosition.y == 0)
//        {
//            odometryInputMixPercentage = 1;
//        }
//        else
//        {
//            odometryInputMixPercentage = 0.9;
//        }
//
//        output.x = (odometry.getPosition().x * odometryInputMixPercentage) + (limelightPosition.x * 1-odometryInputMixPercentage);
//        output.y = ((odometry.getPosition().y+15) * odometryInputMixPercentage) + (limelightPosition.y * 1-odometryInputMixPercentage);
//        output.h = odometry.getPosition().h;
//
//        return output;
//    }

//    public void SetKPX(double temp)
//    {
//        kpx = temp;
//    }
//
//    public void SetKPY(double temp)
//    {
//        kpy = temp;
//    }
//
//    public void SetKDX(double temp)
//    {
//        kdx = temp;
//    }
//
//    public void SetKDY(double temp)
//    {
//        kdy = temp;
//    }
//
//    public double GetKPX()
//    {
//        return kpx;
//    }
//
//    public double GetKPY()
//    {
//        return kpy;
//    }
//
//    public double GetKDX()
//    {
//        return kdx;
//    }
//
//    public double GetKDY()
//    {
//        return kdy;
//    }

    private SparkFunOTOS.Pose2D GetOdometryLocalization() {
        SparkFunOTOS.Pose2D output = new SparkFunOTOS.Pose2D();

        output.x = odometry.getPosition().x;
        output.y = odometry.getPosition().y;
        output.h = odometry.getPosition().h;

        return output;
    }

    public SparkFunOTOS.Pose2D PrintOdometryLocalization()
    {
        return GetOdometryLocalization();
    }

//    public void SetLastLimelightPosition(SparkFunOTOS.Pose2D value)
//    {
//        lastLimelightPosition = value;
//    }

    public void SetLastValidIMUReading()
    {
        lastValidIMUReading = GetImuReading() + lastValidIMUReading;
    }

    private void NewHighestOutputX(double x) {
        if (x > maximumOutputX) {
            maximumOutputX = x;
        }
    }

    private void NewHighestOutputY(double y) {
        if (y > maximumOutputY) {
            maximumOutputY = y;
        }
    }

    private double DiagonalScalar(double x, double y, double min) {
        diagonalScalar = Math.max((Math.min(x, y) / Math.max(x, y)), min);
        return diagonalScalar;
    }

    public void Rehome()
    {
        odometry.resetTracking();
    }

    public double GetDiagonalScalar()
    {
        return diagonalScalar;
    }

    public double angleWrap(double rad)
    {
        while (rad > Math.PI)
        {
            rad -= 2 * Math.PI;
        }
        while (rad < -Math.PI)
        {
            rad += 2 * Math.PI;
        }
        return -rad;
    }

    public void OverrideOtosPos(SparkFunOTOS.Pose2D position)
    {
        odometry.setPosition(position);
    }

    public double GetLastValidIMUReading()
    {
        return lastValidIMUReading;
    }

    public void SetAngleLenience(double temp)
    {
        angleLenience = temp;
    }

    public double GetImuReading()
    {
        return odometry.getPosition().h;
    }
    public double GetOrientation() {return GetImuReading() + lastValidIMUReading;}

    public double GetPositionX()
    {
        return odometry.getPosition().x;
    }

    public double GetPositionY()
    {
        return odometry.getPosition().y;
    }

//    public double GetLocalizationX() { return GetLocalization().x; }
//    public double GetLocalizationY() { return GetLocalization().y; }

    public double GetErrorX()
    {
        return errors[0];
    }

    public double GetErrorY()
    {
        return errors[1];
    }

    public double GetErrorH()
    {
        return errors[2];
    }

    public double GetOutputX() {return output[0];}
    public double GetOutputY() {return output[1];}

    public double GetAutograbZeroX() {return autograbZeroX;}
    public double GetAutograbZeroY() {return autograbZeroY;}

    public void SetAutograbZeroX(double temp) {autograbZeroX = temp;}
    public void SetAutograbZeroY(double temp) {autograbZeroY = temp;}

    public double GetDerivativeX() { return dX; }

    public double GetPorportionalX() {return pX;}
//    public boolean CamIsValid() {return limelight.GetIsValid();}

//    public SparkFunOTOS.Pose2D GetLastLimelightPosition()
//    {
//        return lastLimelightPosition;
//    }

    public boolean GetBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (!completedBools[i])
            {
                return false;
            }
        }
        integralX = 0;
        integralY = 0;
        integralH = 0;
        maximumOutputX = 1;
        maximumOutputY = 1;
        return true;
    }

    public boolean GetStopBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (!completedStopBools[i])
            {
                return false;
            }
        }
        integralX = 0;
        integralY = 0;

        integralH = 0;
        maximumOutputX = 1;
        maximumOutputY = 1;
        return true;
    }

    public void DeactivateBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            errors[i] = 0;
            output[i] = 0;
            completedBools[i] = false;
            completedStopBools[i] = false;
        }
    }

    public double initErrX = 0;
    public double initErrY = 0;

    // FIXED DihdometryDihtrol2 method
// Replace your existing method with this

    public void DihdometryDihtrol2(double speed, double tgtX, double tgtY, double tgtRot, double distanceLenience, int axis) {
        if (!odometry.isConnected()) {
            return;
        }

        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;

        pos = GetOdometryLocalization();

        if (axis == 3 || axis == 4) {
            angleLenience = 15;
        } else {
            angleLenience = 60;
        }

        // Position errors
        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;

        // Calculate angle to target position (for driving direction)
        double angleToTarget = Math.toDegrees(Math.atan2(errors[1], errors[0]));
        double distance = Math.hypot(errors[0], errors[1]);

        // FIX: Rotation error should use tgtRot, not angleToTarget!
        errors[2] = Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)));

        completedBools[2] = Math.abs(errors[2]) < angleLenience;

        errors[2] /= 10; // err crunch tunable

        // Calculate outputs
        output[0] = pid(errors[0], 0, distanceLenience);
        output[1] = pid(errors[1], 1, distanceLenience);
        output[2] = pid(errors[2], 2, angleLenience);

        // Magnitude control for driving
        double mag = pid(distance, 3, distanceLenience);

        // Deadband for rotation (prevent jittering)
        double delta = lastHeadinerorrthingy - errors[2];
        if (Math.abs(delta) < dihband) {
            errors[2] = 0;
        }
        output[2] = pid(errors[2], 2, angleLenience);

        lastHeadinerorrthingy = errors[2];

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;

        completedStopBools[0] = Math.abs(errors[0]) < 0.6;
        completedStopBools[1] = Math.abs(errors[1]) < 0.6;

        if (axis == 0) {
            output[1] *= 0;
            completedBools[1] = true;
        } else if (axis == 1) {
            output[0] *= 0;
            completedBools[0] = true;
        }

        if (tgtRot == 1) {
            completedBools[2] = true;
            output[2] = 0;
        }

        if (axis == 4) {
            completedBools[0] = true;
            completedBools[1] = true;
            output[0] = 0;
            output[1] = 0;
        }

        // Drive towards target position (using angleToTarget)
        // But rotate to tgtRot angle (using output[2])
        double yOut = speed * Math.sin(Math.toRadians(angleToTarget)) * mag;
        double xOut = speed * Math.cos(Math.toRadians(angleToTarget)) * mag;

        double maxOut = Math.max(Math.abs(yOut), Math.abs(xOut));

        if (maxOut > 0.001) {  // Prevent divide by zero
            yOut /= maxOut;
            xOut /= maxOut;
        }

        dt.swerveDrive(yOut, xOut, -speed * output[2]);
    }
    public void thing(double speed, double tgtX, double tgtY, double tgtRot, double distanceLenience, int axis) { //tuff as hell
        if (!odometry.isConnected()) {
            return;
        }
//        distanceLenience; //best value 1.75

        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;

//        pos = limelight.GetLimelightData(false, GetOdometryLocalization().h);
//        pos = GetLocalization();

//        pos = PoseEstimator();

        pos = GetOdometryLocalization();

        if (axis == 3 || axis == 4) {
            angleLenience = 15;
        } else {
            angleLenience = 60;
        }

        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)));
        double errorMag = Math.hypot(errors[0], errors[1]);
        double errorAngle = Math.toDegrees(Math.atan2(errors[1], errors[0]));
        double angleThingy = 90 - pos.h; //does pos h do degreees????
        double idkAngle = 90 - (errorAngle - angleThingy); //this depends again on the polarity of the heading and the odometry cordinate system to switch it around if its that way idk
        errors[0] = errorMag * Math.cos(Math.toRadians(idkAngle));
        errors[1] = errorMag * Math.sin(Math.toRadians(idkAngle));

        completedBools[2] = Math.abs(errors[2]) < angleLenience;

        errors[2] /= 10; // err crunch tunable

//        if (new ArmLiftMotor().GetLocalNeutral() == 1250) {
//            TuningUp();
//        } else {
//            TuningDown();
//        }

        // normalized against one another
        // should create weird diagonal movement
        // might have to add increased magnitude to error, currently between -1 and 1
        output[0] = pid(errors[0], 0, distanceLenience);
        output[1] = pid(errors[1], 1, distanceLenience);
        output[2] = pid(errors[2], 2, angleLenience);

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;

        completedStopBools[0] = Math.abs(errors[0]) < 0.6;
        completedStopBools[1] = Math.abs(errors[1]) < 0.6;

        if (axis == 0) {
//            output[1] = output[1] / Math.abs(output[1]) * 0.2;
            output[1] *= 0;
//            output[2] *= 0;
            completedBools[1] = true;
        } else if (axis == 1) {
            output[0] *= 0;
            completedBools[0] = true;
        }
        if (tgtRot == 1){
            completedBools[2] = true;
            output[2] = 0;
        }

        if (axis == 4)
        {
            completedBools[0] = true;
            completedBools[1] = true;
            output[0] = 0;
            output[1] = 0;
        }

//        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], GetOrientation());
        dt.swerveDrive(speed * output[1], speed * output[0], -speed * output[2]);

    }
}

/*
Congratulations!! You found this useless comment

:D

IF YOU ARE PICKING UP THIS CODE BASE I AM GENUINELY SORRY
PS. I strongly recommend coffee when updating this code
 */


