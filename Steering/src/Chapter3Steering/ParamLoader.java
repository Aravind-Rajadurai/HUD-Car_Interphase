/**
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import common.misc.iniFileLoaderBase;
import java.io.IOException;

public class ParamLoader extends iniFileLoaderBase {

    final static ParamLoader Prm;

    static {
        try {
            Prm = new ParamLoader();
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
    }

    public static ParamLoader Instance() {
        return Prm;
    }

    private ParamLoader() throws IOException {
        super(ParamLoader.class.getResource("params.ini"));
        NumAgents = GetNextParameterInt();
        NumObstacles = GetNextParameterInt();
        MinObstacleRadius = GetNextParameterFloat();
        MaxObstacleRadius = GetNextParameterFloat();

        NumCellsX = GetNextParameterInt();
        NumCellsY = GetNextParameterInt();

        NumSamplesForSmoothing = GetNextParameterInt();

        SteeringForceTweaker = GetNextParameterFloat();
        MaxSteeringForce = GetNextParameterFloat() * SteeringForceTweaker;
        MaxSpeed = GetNextParameterFloat();
        VehicleMass = GetNextParameterFloat();
        VehicleScale = GetNextParameterFloat();

        SeparationWeight = GetNextParameterFloat() * SteeringForceTweaker;
        AlignmentWeight = GetNextParameterFloat() * SteeringForceTweaker;
        CohesionWeight = GetNextParameterFloat() * SteeringForceTweaker;
        ObstacleAvoidanceWeight = GetNextParameterFloat() * SteeringForceTweaker;
        WallAvoidanceWeight = GetNextParameterFloat() * SteeringForceTweaker;
        WanderWeight = GetNextParameterFloat() * SteeringForceTweaker;
        SeekWeight = GetNextParameterFloat() * SteeringForceTweaker;
        FleeWeight = GetNextParameterFloat() * SteeringForceTweaker;
        ArriveWeight = GetNextParameterFloat() * SteeringForceTweaker;
        PursuitWeight = GetNextParameterFloat() * SteeringForceTweaker;
        OffsetPursuitWeight = GetNextParameterFloat() * SteeringForceTweaker;
        InterposeWeight = GetNextParameterFloat() * SteeringForceTweaker;
        HideWeight = GetNextParameterFloat() * SteeringForceTweaker;
        EvadeWeight = GetNextParameterFloat() * SteeringForceTweaker;
        FollowPathWeight = GetNextParameterFloat() * SteeringForceTweaker;

        ViewDistance = GetNextParameterFloat();
        MinDetectionBoxLength = GetNextParameterFloat();
        WallDetectionFeelerLength = GetNextParameterFloat();

        prWallAvoidance = GetNextParameterFloat();
        prObstacleAvoidance = GetNextParameterFloat();
        prSeparation = GetNextParameterFloat();
        prAlignment = GetNextParameterFloat();
        prCohesion = GetNextParameterFloat();
        prWander = GetNextParameterFloat();
        prSeek = GetNextParameterFloat();
        prFlee = GetNextParameterFloat();
        prEvade = GetNextParameterFloat();
        prHide = GetNextParameterFloat();
        prArrive = GetNextParameterFloat();

        MaxTurnRatePerSecond = common.misc.utils.Pi;
    }
    public int NumAgents;
    public int NumObstacles;
    public double MinObstacleRadius;
    public double MaxObstacleRadius;
    //number of horizontal cells used for spatial partitioning
    public int NumCellsX;
    //number of vertical cells used for spatial partitioning
    public int NumCellsY;
    //how many samples the smoother will use to average a value
    public int NumSamplesForSmoothing;
    //used to tweak the combined steering force (simply altering the MaxSteeringForce
    //will NOT work! This tweaker affects all the steering force multipliers
    //too).
    public double SteeringForceTweaker;
    public double MaxSteeringForce;
    public double MaxSpeed;
    public double VehicleMass;
    public double VehicleScale;
    public double MaxTurnRatePerSecond;
    public double SeparationWeight;
    public double AlignmentWeight;
    public double CohesionWeight;
    public double ObstacleAvoidanceWeight;
    public double WallAvoidanceWeight;
    public double WanderWeight;
    public double SeekWeight;
    public double FleeWeight;
    public double ArriveWeight;
    public double PursuitWeight;
    public double OffsetPursuitWeight;
    public double InterposeWeight;
    public double HideWeight;
    public double EvadeWeight;
    public double FollowPathWeight;
    //how close a neighbour must be before an agent perceives it (considers it
    //to be within its neighborhood)
    public double ViewDistance;
    //used in obstacle avoidance
    public double MinDetectionBoxLength;
    //used in wall avoidance
    public double WallDetectionFeelerLength;
    //these are the probabilities that a steering behavior will be used
    //when the prioritized dither calculate method is used
    public double prWallAvoidance;
    public double prObstacleAvoidance;
    public double prSeparation;
    public double prAlignment;
    public double prCohesion;
    public double prWander;
    public double prSeek;
    public double prFlee;
    public double prEvade;
    public double prHide;
    public double prArrive;
}
