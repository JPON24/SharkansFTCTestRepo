package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.global.constants;

import java.util.List;

public class AprilTagLimelight
{
    private Limelight3A limeLight;
//    private IMU Imu;

    private final double LIMELIGHTANGLECONST_D = constants.LIMELIGHT_ANGLE_CONST_D;

    private final double APRILTAGH_M = constants.APRILTAG_HEIGHT_M;
    private final double LIMELIGHTDISTBOTTOM_M = constants.LIMELIGHT_DIST_BOTTOM_M;
    private final double LIMELIGHTDISTCONST_M = APRILTAGH_M - LIMELIGHTDISTBOTTOM_M;

    private final int POLLING_RATE = constants.LIMELIGHT_POLLING_RATE;

    public void init(HardwareMap hwMap, int pipeline) {
        limeLight = hwMap.get(Limelight3A.class, "limelight");
//        Imu = hwMap.get(IMU.class, "imu");
        limeLight.pipelineSwitch(pipeline); // blue alliance
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        limeLight.setPollRateHz(POLLING_RATE);

//        Imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limeLight.start(); // Uses large  amount of battery btw... IF DELAY MOVE TO INIT METHOD.
    }

    // Cached result to avoid multiple Limelight polls per loop
    private LLResult cachedResult = null;

    /**
     * Call once per loop BEFORE using GetTX/GetDistance/GetLimelightId.
     * Ensures all methods use the same frame data.
     */
    public void cacheResult() {
        limeLight.updateRobotOrientation(0);
        cachedResult = limeLight.getLatestResult();
    }

    private LLResult GetResult()
    {
        // Use cached result if available, otherwise poll fresh
        if (cachedResult != null) {
            LLResult result = cachedResult;
            return result;
        }
        limeLight.updateRobotOrientation(0);
        return limeLight.getLatestResult();
    }



    public double GetDistance()
    {
        LLResult llresult = GetResult();

        if (llresult != null && llresult.isValid()) {

            double theta = llresult.getTy();
            theta += LIMELIGHTANGLECONST_D;

            theta = Math.toRadians(theta);

            double DIST_M = LIMELIGHTDISTCONST_M / Math.tan(theta);
            double DIST_I = DIST_M / 25.4;

            return DIST_I + constants.LIMELIGHT_OFFSET_CONST;
        }
        return -1;
    }

    public double GetTX()
    {
        LLResult llresult = GetResult();
        if (llresult == null || !llresult.isValid()) return 0;
        return llresult.getTx();
    }

    public int GetLimelightId()
    {
        LLResult llresult = GetResult();
        if (llresult == null || !llresult.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fiducial = llresult.getFiducialResults();

        if (fiducial.isEmpty())
        {
            return 0;
        }
        return fiducial.get(0).getFiducialId();
    }

    public double GetYaw()
    {
        LLResult llresult = GetResult();
        if (llresult != null && llresult.isValid()) {
/// THE ABSOLUTE YAHHHHH FOR THE LIM LIGHT
            return llresult.getBotpose().getOrientation().getYaw();
        }
        return 0;
    }
}
