/**
 * Desc:   class to encapsulate steering behaviors for a Vehicle
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import static Chapter3Steering.ParamLoader.Prm;
import common.D2.Vector2D;
import static common.D2.Vector2D.*;
import static common.D2.Transformation.*;
import static common.D2.geometry.*;
import common.D2.Wall2D;
import common.misc.Cgdi;
import static common.misc.utils.*;
import static common.misc.Cgdi.gdi;
import static common.misc.Stream_Utility_function.*;
import static common.misc.CppToJava.*;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

class SteeringBehavior {
//--------------------------- Constants ----------------------------------
    //the radius of the constraining circle for the wander behavior

    final double WanderRad = 1.2;
    //distance the wander circle is projected in front of the agent
    final double WanderDist = 2.0;
    //the maximum amount of displacement along the circle each frame
    final double WanderJitterPerSec = 80.0;
    //used in path following
    final double WaypointSeekDist = 20;

//------------------------------------------------------------------------
    public static enum summing_method {

        weighted_average(0),
        prioritized(1),
        dithered(2);

        summing_method(int i) {
        }
    }

    private enum behavior_type {

        none(0x00000),
        seek(0x00002),
        flee(0x00004),
        arrive(0x00008),
        wander(0x00010),
        cohesion(0x00020),
        separation(0x00040),
        allignment(0x00080),
        obstacle_avoidance(0x00100),
        wall_avoidance(0x00200),
        follow_path(0x00400),
        pursuit(0x00800),
        evade(0x01000),
        interpose(0x02000),
        hide(0x04000),
        flock(0x08000),
        offset_pursuit(0x10000);
        private int flag;

        behavior_type(int flag) {
            this.flag = flag;
        }

        public int flag() {
            return this.flag;
        }
    }
    //a pointer to the owner of this instance
    private Vehicle m_pVehicle;
    //the steering force created by the combined effect of all
    //the selected behaviors
    private Vector2D m_vSteeringForce = new Vector2D(0, 0);
    //these can be used to keep track of friends, pursuers, or prey
    private Vehicle m_pTargetAgent1;
    private Vehicle m_pTargetAgent2;
    //the current target
    private Vector2D m_vTarget = new Vector2D(0, 0);
    //length of the 'detection box' utilized in obstacle avoidance
    private double m_dDBoxLength;
    //a vertex buffer to contain the feelers rqd for wall avoidance  
    private List<Vector2D> m_Feelers;
    //the length of the 'feeler/s' used in wall detection
    private double m_dWallDetectionFeelerLength;
    //the current position on the wander circle the agent is
    //attempting to steer towards
    private Vector2D m_vWanderTarget;
    //explained above
    private double m_dWanderJitter;
    private double m_dWanderRadius;
    private double m_dWanderDistance;
    //multipliers. These can be adjusted to effect strength of the  
    //appropriate behavior. Useful to get flocking the way you require
    //for example.
    private double m_dWeightSeparation;
    private double m_dWeightCohesion;
    private double m_dWeightAlignment;
    private double m_dWeightWander;
    private double m_dWeightObstacleAvoidance;
    private double m_dWeightWallAvoidance;
    private double m_dWeightSeek;
    private double m_dWeightFlee;
    private double m_dWeightArrive;
    private double m_dWeightPursuit;
    private double m_dWeightOffsetPursuit;
    private double m_dWeightInterpose;
    private double m_dWeightHide;
    private double m_dWeightEvade;
    private double m_dWeightFollowPath;
    //how far the agent can 'see'
    private double m_dViewDistance;
    //pointer to any current path
    private Path m_pPath;
    //the distance (squared) a vehicle has to be from a path waypoint before
    //it starts seeking to the next waypoint
    private double m_dWaypointSeekDistSq;
    //any offset used for formations or offset pursuit
    private Vector2D m_vOffset;
    //binary flags to indicate whether or not a behavior should be active
    private int m_iFlags;

    //Arrive makes use of these to determine how quickly a vehicle
    //should decelerate to its target
    private enum Deceleration {

        slow(3), normal(2), fast(1);
        private int dec;

        Deceleration(int d) {
            this.dec = d;
        }

        public int value() {
            return dec;
        }
    }
    //default
    private Deceleration m_Deceleration;
    //is cell space partitioning to be used or not?
    private boolean m_bCellSpaceOn;
    //what type of method is used to sum any active behavior
    private summing_method m_SummingMethod;

    //this function tests if a specific bit of m_iFlags is set
    private boolean On(behavior_type bt) {
        return (m_iFlags & bt.flag()) == bt.flag();
    }

    /**
     *
     * This function calculates how much of its max steering force the 
     * vehicle has left to apply and then applies that amount of the
     * force to add.
     */
    private boolean AccumulateForce(Vector2D RunningTot,
            Vector2D ForceToAdd) {

        //calculate how much steering force the vehicle has used so far
        double MagnitudeSoFar = RunningTot.Length();

        //calculate how much steering force remains to be used by this vehicle
        double MagnitudeRemaining = m_pVehicle.MaxForce() - MagnitudeSoFar;

        //return false if there is no more force left to use
        if (MagnitudeRemaining <= 0.0) {
            return false;
        }

        //calculate the magnitude of the force we want to add
        double MagnitudeToAdd = ForceToAdd.Length();

        //if the magnitude of the sum of ForceToAdd and the running total
        //does not exceed the maximum force available to this vehicle, just
        //add together. Otherwise add as much of the ForceToAdd vector is
        //possible without going over the max.
        if (MagnitudeToAdd < MagnitudeRemaining) {
            RunningTot.add(ForceToAdd);
        } else {
            //add it to the steering force
            RunningTot.add(mul(Vec2DNormalize(ForceToAdd), MagnitudeRemaining));
        }

        return true;
    }

    /**
     *  Creates the antenna utilized by WallAvoidance
     */
    private void CreateFeelers() {
        m_Feelers.clear();
        //feeler pointing straight in front
        m_Feelers.add(add(m_pVehicle.Pos(), mul(m_dWallDetectionFeelerLength, m_pVehicle.Heading())));

        //feeler to left
        Vector2D temp = new Vector2D(m_pVehicle.Heading());
        Vec2DRotateAroundOrigin(temp, HalfPi * 3.5f);
        m_Feelers.add(add(m_pVehicle.Pos(), mul(m_dWallDetectionFeelerLength / 2.0f, temp)));

        //feeler to right
        temp = new Vector2D(m_pVehicle.Heading());
        Vec2DRotateAroundOrigin(temp, HalfPi * 0.5f);
        m_Feelers.add(add(m_pVehicle.Pos(), mul(m_dWallDetectionFeelerLength / 2.0f, temp)));
    }

/////////////////////////////////////////////////////////////////////////////// START OF BEHAVIORS
    /**
     * Given a target, this behavior returns a steering force which will
     *  direct the agent towards the target
     */
    private Vector2D Seek(Vector2D TargetPos) {
        Vector2D DesiredVelocity = mul(Vec2DNormalize(sub(TargetPos, m_pVehicle.Pos())),
                m_pVehicle.MaxSpeed());

        return sub(DesiredVelocity, m_pVehicle.Velocity());
    }

    /**
     *  Does the opposite of Seek
     */
    private Vector2D Flee(Vector2D TargetPos) {
        //only flee if the target is within 'panic distance'. Work in distance
        //squared space.
 /* const double PanicDistanceSq = 100.0f * 100.0;
        if (Vec2DDistanceSq(m_pVehicle.Pos(), target) > PanicDistanceSq)
        {
        return new Vector2D(0,0);
        }
         */

        Vector2D DesiredVelocity = mul(Vec2DNormalize(sub(m_pVehicle.Pos(), TargetPos)),
                m_pVehicle.MaxSpeed());

        return sub(DesiredVelocity, m_pVehicle.Velocity());
    }

    /**
     * This behavior is similar to seek but it attempts to arrive at the
     *  target with a zero velocity
     */
    private Vector2D Arrive(Vector2D TargetPos, Deceleration deceleration) {
        Vector2D ToTarget = sub(TargetPos, m_pVehicle.Pos());

        //calculate the distance to the target
        double dist = ToTarget.Length();

        if (dist > 0) {
            //because Deceleration is enumerated as an int, this value is required
            //to provide fine tweaking of the deceleration..
            final double DecelerationTweaker = 0.3;

            //calculate the speed required to reach the target given the desired
            //deceleration
            double speed = dist / ((double) deceleration.value() * DecelerationTweaker);

            //make sure the velocity does not exceed the max
            speed = Math.min(speed, m_pVehicle.MaxSpeed());

            //from here proceed just like Seek except we don't need to normalize 
            //the ToTarget vector because we have already gone to the trouble
            //of calculating its length: dist. 
            Vector2D DesiredVelocity = mul(ToTarget, speed / dist);

            return sub(DesiredVelocity, m_pVehicle.Velocity());
        }

        return new Vector2D(0, 0);
    }

    /**
     *  this behavior creates a force that steers the agent towards the 
     *  evader
     */
    private Vector2D Pursuit(final Vehicle evader) {
        //if the evader is ahead and facing the agent then we can just seek
        //for the evader's current position.
        Vector2D ToEvader = sub(evader.Pos(), m_pVehicle.Pos());

        double RelativeHeading = m_pVehicle.Heading().Dot(evader.Heading());

        if ((ToEvader.Dot(m_pVehicle.Heading()) > 0)
                && (RelativeHeading < -0.95)) //acos(0.95)=18 degs
        {
            return Seek(evader.Pos());
        }

        //Not considered ahead so we predict where the evader will be.

        //the lookahead time is propotional to the distance between the evader
        //and the pursuer; and is inversely proportional to the sum of the
        //agent's velocities
        double LookAheadTime = ToEvader.Length()
                / (m_pVehicle.MaxSpeed() + evader.Speed());

        //now seek to the predicted future position of the evader
        return Seek(add(evader.Pos(), mul(evader.Velocity(), LookAheadTime)));
    }

    /**
     *  similar to pursuit except the agent Flees from the estimated future
     *  position of the pursuer
     */
    private Vector2D Evade(final Vehicle pursuer) {
        // Not necessary to include the check for facing direction this time

        Vector2D ToPursuer = sub(pursuer.Pos(), m_pVehicle.Pos());

        //uncomment the following two lines to have Evade only consider pursuers 
        //within a 'threat range'
        final double ThreatRange = 100.0;
        if (ToPursuer.LengthSq() > ThreatRange * ThreatRange) {
            return new Vector2D();
        }

        //the lookahead time is propotional to the distance between the pursuer
        //and the pursuer; and is inversely proportional to the sum of the
        //agents' velocities
        double LookAheadTime = ToPursuer.Length()
                / (m_pVehicle.MaxSpeed() + pursuer.Speed());

        //now flee away from predicted future position of the pursuer
        return Flee(add(pursuer.Pos(), mul(pursuer.Velocity(), LookAheadTime)));
    }

    /**
     * This behavior makes the agent wander about randomly
     */
    private Vector2D Wander() {
        //this behavior is dependent on the update rate, so this line must
        //be included when using time independent framerate.
        double JitterThisTimeSlice = m_dWanderJitter * m_pVehicle.getTimeElapsed();

        //first, add a small random vector to the target's position
        m_vWanderTarget.add(new Vector2D(RandomClamped() * JitterThisTimeSlice,
                RandomClamped() * JitterThisTimeSlice));

        //reproject this new vector back on to a unit circle
        m_vWanderTarget.Normalize();

        //increase the length of the vector to the same as the radius
        //of the wander circle
        m_vWanderTarget.mul(m_dWanderRadius);

        //move the target into a position WanderDist in front of the agent
        Vector2D target = add(m_vWanderTarget, new Vector2D(m_dWanderDistance, 0));

        //project the target into world space
        Vector2D Target = PointToWorldSpace(target,
                m_pVehicle.Heading(),
                m_pVehicle.Side(),
                m_pVehicle.Pos());

        //and steer towards it
        return sub(Target, m_pVehicle.Pos());
    }

    /**
     *  Given a vector of obstacles, this method returns a steering force
     *  that will prevent the agent colliding with the closest obstacle
     */
    private Vector2D ObstacleAvoidance(List<BaseGameEntity> obstacles) {
        //the detection box length is proportional to the agent's velocity
        m_dDBoxLength = Prm.MinDetectionBoxLength
                + (m_pVehicle.Speed() / m_pVehicle.MaxSpeed())
                * Prm.MinDetectionBoxLength;

        //tag all obstacles within range of the box for processing
        m_pVehicle.World().TagObstaclesWithinViewRange(m_pVehicle, m_dDBoxLength);

        //this will keep track of the closest intersecting obstacle (CIB)
        BaseGameEntity ClosestIntersectingObstacle = null;

        //this will be used to track the distance to the CIB
        double DistToClosestIP = MaxDouble;

        //this will record the transformed local coordinates of the CIB
        Vector2D LocalPosOfClosestObstacle = new Vector2D();

        ListIterator<BaseGameEntity> it = obstacles.listIterator();

        while (it.hasNext()) {
            //if the obstacle has been tagged within range proceed
            BaseGameEntity curOb = it.next();
            if (curOb.IsTagged()) {
                //calculate this obstacle's position in local space
                Vector2D LocalPos = PointToLocalSpace(curOb.Pos(),
                        m_pVehicle.Heading(),
                        m_pVehicle.Side(),
                        m_pVehicle.Pos());

                //if the local position has a negative x value then it must lay
                //behind the agent. (in which case it can be ignored)
                if (LocalPos.x >= 0) {
                    //if the distance from the x axis to the object's position is less
                    //than its radius + half the width of the detection box then there
                    //is a potential intersection.
                    double ExpandedRadius = curOb.BRadius() + m_pVehicle.BRadius();

                    if (Math.abs(LocalPos.y) < ExpandedRadius) {
                        //now to do a line/circle intersection test. The center of the 
                        //circle is represented by (cX, cY). The intersection points are 
                        //given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
                        //We only need to look at the smallest positive value of x because
                        //that will be the closest point of intersection.
                        double cX = LocalPos.x;
                        double cY = LocalPos.y;

                        //we only need to calculate the sqrt part of the above equation once
                        double SqrtPart = Math.sqrt(ExpandedRadius * ExpandedRadius - cY * cY);

                        double ip = cX - SqrtPart;

                        if (ip <= 0.0) {
                            ip = cX + SqrtPart;
                        }

                        //test to see if this is the closest so far. If it is keep a
                        //record of the obstacle and its local coordinates
                        if (ip < DistToClosestIP) {
                            DistToClosestIP = ip;

                            ClosestIntersectingObstacle = curOb;

                            LocalPosOfClosestObstacle = LocalPos;
                        }
                    }
                }
            }
        }

        //if we have found an intersecting obstacle, calculate a steering 
        //force away from it
        Vector2D SteeringForce = new Vector2D();

        if (ClosestIntersectingObstacle != null) {
            //the closer the agent is to an object, the stronger the 
            //steering force should be
            double multiplier = 1.0 + (m_dDBoxLength - LocalPosOfClosestObstacle.x)
                    / m_dDBoxLength;

            //calculate the lateral force
            SteeringForce.y = (ClosestIntersectingObstacle.BRadius()
                    - LocalPosOfClosestObstacle.y) * multiplier;

            //apply a braking force proportional to the obstacles distance from
            //the vehicle. 
            final double BrakingWeight = 0.2;

            SteeringForce.x = (ClosestIntersectingObstacle.BRadius()
                    - LocalPosOfClosestObstacle.x)
                    * BrakingWeight;
        }

        //finally, convert the steering vector from local to world space
        return VectorToWorldSpace(SteeringForce,
                m_pVehicle.Heading(),
                m_pVehicle.Side());
    }

    /**
     * This returns a steering force that will keep the agent away from any
     *  walls it may encounter
     */
    private Vector2D WallAvoidance(final List<Wall2D> walls) {
        //the feelers are contained in a std::vector, m_Feelers
        CreateFeelers();

        double DistToThisIP = 0.0;
        double DistToClosestIP = MaxDouble;

        //this will hold an index into the vector of walls
        int ClosestWall = -1;

        Vector2D SteeringForce = new Vector2D(),
                point = new Vector2D(), //used for storing temporary info
                ClosestPoint = new Vector2D();  //holds the closest intersection point

        //examine each feeler in turn
        for (int flr = 0; flr < m_Feelers.size(); ++flr) {
            //run through each wall checking for any intersection points
            DoubleRef DistToThisIPRef = new DoubleRef(DistToThisIP);
            for (int w = 0; w < walls.size(); ++w) {
                if (LineIntersection2D(m_pVehicle.Pos(),
                        m_Feelers.get(flr),
                        walls.get(w).From(),
                        walls.get(w).To(),
                        DistToThisIPRef,
                        point)) {
                    DistToThisIP = DistToThisIPRef.toDouble();
                    //is this the closest found so far? If so keep a record
                    if (DistToThisIP < DistToClosestIP) {
                        DistToClosestIP = DistToThisIP;

                        ClosestWall = w;

                        ClosestPoint = point;
                    }
                }
            }//next wall


            //if an intersection point has been detected, calculate a force  
            //that will direct the agent away
            if (ClosestWall >= 0) {
                //calculate by what distance the projected position of the agent
                //will overshoot the wall
                Vector2D OverShoot = sub(m_Feelers.get(flr), ClosestPoint);

                //create a force in the direction of the wall normal, with a 
                //magnitude of the overshoot
                SteeringForce = mul(walls.get(ClosestWall).Normal(), OverShoot.Length());
            }

        }//next feeler

        return SteeringForce;
    }

    /**
     * this calculates a force repelling from the other neighbors
     */
    Vector2D Separation(final List<Vehicle> neighbors) {
        Vector2D SteeringForce = new Vector2D();

        for (int a = 0; a < neighbors.size(); ++a) {
            //make sure this agent isn't included in the calculations and that
            //the agent being examined is close enough. ***also make sure it doesn't
            //include the evade target ***
            if ((neighbors.get(a) != m_pVehicle) && neighbors.get(a).IsTagged()
                    && (neighbors.get(a) != m_pTargetAgent1)) {
                Vector2D ToAgent = sub(m_pVehicle.Pos(), neighbors.get(a).Pos());

                //scale the force inversely proportional to the agents distance  
                //from its neighbor.
                SteeringForce.add(div(Vec2DNormalize(ToAgent), ToAgent.Length()));
            }
        }

        return SteeringForce;
    }

    /**
     * returns a force that attempts to align this agents heading with that
     * of its neighbors
     */
    private Vector2D Alignment(final List<Vehicle> neighbors) {
        //used to record the average heading of the neighbors
        Vector2D AverageHeading = new Vector2D();

        //used to count the number of vehicles in the neighborhood
        int NeighborCount = 0;

        //iterate through all the tagged vehicles and sum their heading vectors  
        for (int a = 0; a < neighbors.size(); ++a) {
            //make sure *this* agent isn't included in the calculations and that
            //the agent being examined  is close enough ***also make sure it doesn't
            //include any evade target ***
            if ((neighbors.get(a) != m_pVehicle) && neighbors.get(a).IsTagged()
                    && (neighbors.get(a) != m_pTargetAgent1)) {
                AverageHeading.add(neighbors.get(a).Heading());

                ++NeighborCount;
            }
        }

        //if the neighborhood contained one or more vehicles, average their
        //heading vectors.
        if (NeighborCount > 0) {
            AverageHeading.div((double) NeighborCount);
            AverageHeading.sub(m_pVehicle.Heading());
        }

        return AverageHeading;
    }

    /**
     * returns a steering force that attempts to move the agent towards the
     * center of mass of the agents in its immediate area
     */
    private Vector2D Cohesion(final List<Vehicle> neighbors) {
        //first find the center of mass of all the agents
        Vector2D CenterOfMass = new Vector2D(), SteeringForce = new Vector2D();

        int NeighborCount = 0;

        //iterate through the neighbors and sum up all the position vectors
        for (int a = 0; a < neighbors.size(); ++a) {
            //make sure *this* agent isn't included in the calculations and that
            //the agent being examined is close enough ***also make sure it doesn't
            //include the evade target ***
            if ((neighbors.get(a) != m_pVehicle) && neighbors.get(a).IsTagged()
                    && (neighbors.get(a) != m_pTargetAgent1)) {
                CenterOfMass.add(neighbors.get(a).Pos());

                ++NeighborCount;
            }
        }

        if (NeighborCount > 0) {
            //the center of mass is the average of the sum of positions
            CenterOfMass.div((double) NeighborCount);

            //now seek towards that position
            SteeringForce = Seek(CenterOfMass);
        }

        //the magnitude of cohesion is usually much larger than separation or
        //allignment so it usually helps to normalize it.
        return Vec2DNormalize(SteeringForce);
    }

    /* NOTE: the next three behaviors are the same as the above three, except
    that they use a cell-space partition to find the neighbors
     */
    /**
     * this calculates a force repelling from the other neighbors
     *
     * USES SPACIAL PARTITIONING
     */
    private Vector2D SeparationPlus(final List<Vehicle> neighbors) {
        Vector2D SteeringForce = new Vector2D();

        //iterate through the neighbors and sum up all the position vectors
        for (BaseGameEntity pV = m_pVehicle.World().CellSpace().begin();
                !m_pVehicle.World().CellSpace().end();
                pV = m_pVehicle.World().CellSpace().next()) {
            //make sure this agent isn't included in the calculations and that
            //the agent being examined is close enough
            if (pV != m_pVehicle) {
                Vector2D ToAgent = sub(m_pVehicle.Pos(), pV.Pos());

                //scale the force inversely proportional to the agents distance  
                //from its neighbor.
                SteeringForce.add(div(Vec2DNormalize(ToAgent), ToAgent.Length()));
            }

        }

        return SteeringForce;
    }

    /**
     * returns a force that attempts to align this agents heading with that
     * of its neighbors
     *
     *  USES SPACIAL PARTITIONING
     */
    private Vector2D AlignmentPlus(final List<Vehicle> neighbors) {
        //This will record the average heading of the neighbors
        Vector2D AverageHeading = new Vector2D();

        //This count the number of vehicles in the neighborhood
        double NeighborCount = 0.0;

        //iterate through the neighbors and sum up all the position vectors
        for (MovingEntity pV = m_pVehicle.World().CellSpace().begin();
                !m_pVehicle.World().CellSpace().end();
                pV = m_pVehicle.World().CellSpace().next()) {
            //make sure *this* agent isn't included in the calculations and that
            //the agent being examined  is close enough
            if (pV != m_pVehicle) {
                AverageHeading.add(pV.Heading());
                ++NeighborCount;
            }
        }

        //if the neighborhood contained one or more vehicles, average their
        //heading vectors.
        if (NeighborCount > 0.0) {
            AverageHeading.div(NeighborCount);
            AverageHeading.sub(m_pVehicle.Heading());
        }

        return AverageHeading;
    }

    /**
     * returns a steering force that attempts to move the agent towards the
     * center of mass of the agents in its immediate area
     *
     * USES SPACIAL PARTITIONING
     */
    private Vector2D CohesionPlus(final List<Vehicle> neighbors) {
        //first find the center of mass of all the agents
        Vector2D CenterOfMass = new Vector2D(), SteeringForce = new Vector2D();

        int NeighborCount = 0;

        //iterate through the neighbors and sum up all the position vectors
        for (BaseGameEntity pV = m_pVehicle.World().CellSpace().begin();
                !m_pVehicle.World().CellSpace().end();
                pV = m_pVehicle.World().CellSpace().next()) {
            //make sure *this* agent isn't included in the calculations and that
            //the agent being examined is close enough
            if (pV != m_pVehicle) {
                CenterOfMass.add(pV.Pos());

                ++NeighborCount;
            }
        }

        if (NeighborCount > 0) {
            //the center of mass is the average of the sum of positions
            CenterOfMass.div((double) NeighborCount);

            //now seek towards that position
            SteeringForce = Seek(CenterOfMass);
        }

        //the magnitude of cohesion is usually much larger than separation or
        //allignment so it usually helps to normalize it.
        return Vec2DNormalize(SteeringForce);
    }

    /**
     * Given two agents, this method returns a force that attempts to 
     * position the vehicle between them
     */
    private Vector2D Interpose(final Vehicle AgentA, final Vehicle AgentB) {
        //first we need to figure out where the two agents are going to be at 
        //time T in the future. This is approximated by determining the time
        //taken to reach the mid way point at the current time at at max speed.
        Vector2D MidPoint = div(add(AgentA.Pos(), AgentB.Pos()), 2.0);

        double TimeToReachMidPoint = Vec2DDistance(m_pVehicle.Pos(), MidPoint)
                / m_pVehicle.MaxSpeed();

        //now we have T, we assume that agent A and agent B will continue on a
        //straight trajectory and extrapolate to get their future positions
        Vector2D APos = add(AgentA.Pos(), mul(AgentA.Velocity(), TimeToReachMidPoint));
        Vector2D BPos = add(AgentB.Pos(), mul(AgentB.Velocity(), TimeToReachMidPoint));

        //calculate the mid point of these predicted positions
        MidPoint = div(add(APos, BPos), 2.0);

        //then steer to Arrive at it
        return Arrive(MidPoint, Deceleration.fast);
    }

    private Vector2D Hide(final Vehicle hunter, final List<BaseGameEntity> obstacles) {
        double DistToClosest = MaxDouble;
        Vector2D BestHidingSpot = new Vector2D();

        ListIterator<BaseGameEntity> it = obstacles.listIterator();
        BaseGameEntity closest;

        while (it.hasNext()) {
            BaseGameEntity curOb = it.next();
            //calculate the position of the hiding spot for this obstacle
            Vector2D HidingSpot = GetHidingPosition(curOb.Pos(),
                    curOb.BRadius(),
                    hunter.Pos());

            //work in distance-squared space to find the closest hiding
            //spot to the agent
            double dist = Vec2DDistanceSq(HidingSpot, m_pVehicle.Pos());

            if (dist < DistToClosest) {
                DistToClosest = dist;

                BestHidingSpot = HidingSpot;

                closest = curOb;
            }
        }//end while

        //if no suitable obstacles found then Evade the hunter
        if (DistToClosest == MaxFloat) {
            return Evade(hunter);
        }

        //else use Arrive on the hiding spot
        return Arrive(BestHidingSpot, Deceleration.fast);
    }

    /**
     *  Given the position of a hunter, and the position and radius of
     *  an obstacle, this method calculates a position DistanceFromBoundary 
     *  away from its bounding radius and directly opposite the hunter
     */
    private Vector2D GetHidingPosition(final Vector2D posOb,
            final double radiusOb,
            final Vector2D posHunter) {
        //calculate how far away the agent is to be from the chosen obstacle's
        //bounding radius
        final double DistanceFromBoundary = 30.0;
        double DistAway = radiusOb + DistanceFromBoundary;

        //calculate the heading toward the object from the hunter
        Vector2D ToOb = Vec2DNormalize(sub(posOb, posHunter));

        //scale it to size and add to the obstacles position to get
        //the hiding spot.
        return add(mul(ToOb, DistAway), posOb);
    }

    /**
     *  Given a series of Vector2Ds, this method produces a force that will
     *  move the agent along the waypoints in order. The agent uses the
     * 'Seek' behavior to move to the next waypoint - unless it is the last
     *  waypoint, in which case it 'Arrives'
     */
    private Vector2D FollowPath() {
        //move to next target if close enough to current target (working in
        //distance squared space)
        if (Vec2DDistanceSq(m_pPath.CurrentWaypoint(), m_pVehicle.Pos())
                < m_dWaypointSeekDistSq) {
            m_pPath.SetNextWaypoint();
        }

        if (!m_pPath.Finished()) {
            return Seek(m_pPath.CurrentWaypoint());
        } else {
            return Arrive(m_pPath.CurrentWaypoint(), Deceleration.normal);
        }
    }

    /**
     * Produces a steering force that keeps a vehicle at a specified offset
     * from a leader vehicle
     */
    private Vector2D OffsetPursuit(final Vehicle leader,
            final Vector2D offset) {
        //calculate the offset's position in world space
        Vector2D WorldOffsetPos = PointToWorldSpace(offset,
                leader.Heading(),
                leader.Side(),
                leader.Pos());

        Vector2D ToOffset = sub(WorldOffsetPos, m_pVehicle.Pos());

        //the lookahead time is propotional to the distance between the leader
        //and the pursuer; and is inversely proportional to the sum of both
        //agent's velocities
        double LookAheadTime = ToOffset.Length()
                / (m_pVehicle.MaxSpeed() + leader.Speed());

        //now Arrive at the predicted future position of the offset
        return Arrive(add(WorldOffsetPos, mul(leader.Velocity(), LookAheadTime)), Deceleration.fast);
    }

//------------------------- ctor -----------------------------------------
//
//------------------------------------------------------------------------
    public SteeringBehavior(Vehicle agent) {

        m_pVehicle = agent;
        m_iFlags = 0;
        m_dDBoxLength = Prm.MinDetectionBoxLength;
        m_dWeightCohesion = Prm.CohesionWeight;
        m_dWeightAlignment = Prm.AlignmentWeight;
        m_dWeightSeparation = Prm.SeparationWeight;
        m_dWeightObstacleAvoidance = Prm.ObstacleAvoidanceWeight;
        m_dWeightWander = Prm.WanderWeight;
        m_dWeightWallAvoidance = Prm.WallAvoidanceWeight;
        m_dViewDistance = Prm.ViewDistance;
        m_dWallDetectionFeelerLength = Prm.WallDetectionFeelerLength;
        m_Feelers = new ArrayList<Vector2D>(3);
        m_Deceleration = Deceleration.normal;
        m_pTargetAgent1 = null;
        m_pTargetAgent2 = null;
        m_dWanderDistance = WanderDist;
        m_dWanderJitter = WanderJitterPerSec;
        m_dWanderRadius = WanderRad;
        m_dWaypointSeekDistSq = WaypointSeekDist * WaypointSeekDist;
        m_dWeightSeek = Prm.SeekWeight;
        m_dWeightFlee = Prm.FleeWeight;
        m_dWeightArrive = Prm.ArriveWeight;
        m_dWeightPursuit = Prm.PursuitWeight;
        m_dWeightOffsetPursuit = Prm.OffsetPursuitWeight;
        m_dWeightInterpose = Prm.InterposeWeight;
        m_dWeightHide = Prm.HideWeight;
        m_dWeightEvade = Prm.EvadeWeight;
        m_dWeightFollowPath = Prm.FollowPathWeight;
        m_bCellSpaceOn = false;
        m_SummingMethod = summing_method.prioritized;

        //stuff for the wander behavior
        double theta = RandFloat() * TwoPi;

        //create a vector to a target position on the wander circle
        m_vWanderTarget = new Vector2D(m_dWanderRadius * Math.cos(theta),
                m_dWanderRadius * Math.sin(theta));

        //create a Path
        m_pPath = new Path();
        m_pPath.LoopOn();

    }

//---------------------------------dtor ----------------------------------
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }
    //a vertex buffer rqd for drawing the detection box
    static List<Vector2D> box = new ArrayList(4);

    /**
     * renders visual aids and info for seeing how each behavior is
     * calculated
     */
    public void RenderAids() {
        gdi.TransparentText();
        gdi.TextColor(Cgdi.grey);

        int NextSlot = gdi.fontHeight();
        int SlotSize = 20;

        if (KEYDOWN(KeyEvent.VK_INSERT)) {
            m_pVehicle.SetMaxForce(m_pVehicle.MaxForce() + 1000.0f * m_pVehicle.getTimeElapsed());
        }
        if (KEYDOWN(KeyEvent.VK_DELETE)) {
            if (m_pVehicle.MaxForce() > 0.2f) {
                m_pVehicle.SetMaxForce(m_pVehicle.MaxForce() - 1000.0f * m_pVehicle.getTimeElapsed());
            }
        }
        if (KEYDOWN(KeyEvent.VK_HOME)) {
            m_pVehicle.SetMaxSpeed(m_pVehicle.MaxSpeed() + 50.0f * m_pVehicle.getTimeElapsed());
        }
        if (KEYDOWN(KeyEvent.VK_END)) {
            if (m_pVehicle.MaxSpeed() > 0.2f) {
                m_pVehicle.SetMaxSpeed(m_pVehicle.MaxSpeed() - 50.0f * m_pVehicle.getTimeElapsed());
            }
        }

        if (m_pVehicle.MaxForce() < 0) {
            m_pVehicle.SetMaxForce(0.0f);
        }
        if (m_pVehicle.MaxSpeed() < 0) {
            m_pVehicle.SetMaxSpeed(0.0f);
        }
        
        if (m_pVehicle.ID() == 0) {
            gdi.TextAtPos(5, NextSlot, "MaxForce(Ins/Del):");
            gdi.TextAtPos(160, NextSlot, ttos(m_pVehicle.MaxForce() / Prm.SteeringForceTweaker));
            NextSlot += SlotSize;
        }
        if (m_pVehicle.ID() == 0) {
            gdi.TextAtPos(5, NextSlot, "MaxSpeed(Home/End):");
            gdi.TextAtPos(160, NextSlot, ttos(m_pVehicle.MaxSpeed()));
            NextSlot += SlotSize;
        }

        //render the steering force
        if (m_pVehicle.World().RenderSteeringForce()) {
            gdi.RedPen();
            Vector2D F = mul((div(m_vSteeringForce, Prm.SteeringForceTweaker)), Prm.VehicleScale);
            gdi.Line(m_pVehicle.Pos(), add(m_pVehicle.Pos(), F));
        }

        //render wander stuff if relevant
        if (On(behavior_type.wander) && m_pVehicle.World().RenderWanderCircle()) {

            if (KEYDOWN('F')) {
                m_dWanderJitter += 1.0f * m_pVehicle.getTimeElapsed();
                m_dWanderJitter = clamp(m_dWanderJitter, 0.0, 100.0);
            }
            if (KEYDOWN('V')) {
                m_dWanderJitter -= 1.0f * m_pVehicle.getTimeElapsed();
                m_dWanderJitter = clamp(m_dWanderJitter, 0.0, 100.0);
            }
            if (KEYDOWN('G')) {
                m_dWanderDistance += 2.0f * m_pVehicle.getTimeElapsed();
                m_dWanderDistance = clamp(m_dWanderDistance, 0.0, 50.0);
            }
            if (KEYDOWN('B')) {
                m_dWanderDistance -= 2.0f * m_pVehicle.getTimeElapsed();
                m_dWanderDistance = clamp(m_dWanderDistance, 0.0, 50.0);
            }
            if (KEYDOWN('H')) {
                m_dWanderRadius += 2.0f * m_pVehicle.getTimeElapsed();
                m_dWanderRadius = clamp(m_dWanderRadius, 0.0, 100.0);
            }
            if (KEYDOWN('N')) {
                m_dWanderRadius -= 2.0f * m_pVehicle.getTimeElapsed();
                m_dWanderRadius = clamp(m_dWanderRadius, 0.0, 100.0);
            }

            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "Jitter(F/V): ");
                gdi.TextAtPos(160, NextSlot, ttos(m_dWanderJitter));
                NextSlot += SlotSize;
            }
            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "Distance(G/B): ");
                gdi.TextAtPos(160, NextSlot, ttos(m_dWanderDistance));
                NextSlot += SlotSize;
            }
            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "Radius(H/N): ");
                gdi.TextAtPos(160, NextSlot, ttos(m_dWanderRadius));
                NextSlot += SlotSize;
            }


            //calculate the center of the wander circle
            Vector2D m_vTCC = PointToWorldSpace(new Vector2D(m_dWanderDistance * m_pVehicle.BRadius(), 0),
                    m_pVehicle.Heading(),
                    m_pVehicle.Side(),
                    m_pVehicle.Pos());
            //draw the wander circle
            gdi.GreenPen();
            gdi.HollowBrush();
            gdi.Circle(m_vTCC, m_dWanderRadius * m_pVehicle.BRadius());

            //draw the wander target
            gdi.RedPen();
            gdi.Circle(PointToWorldSpace(mul(add(m_vWanderTarget, new Vector2D(m_dWanderDistance, 0)), m_pVehicle.BRadius()),
                    m_pVehicle.Heading(),
                    m_pVehicle.Side(),
                    m_pVehicle.Pos()), 3);
        }

        //render the detection box if relevant
        if (m_pVehicle.World().RenderDetectionBox()) {

            gdi.GreyPen();


            double length = Prm.MinDetectionBoxLength
                    + (m_pVehicle.Speed() / m_pVehicle.MaxSpeed())
                    * Prm.MinDetectionBoxLength;

            //verts for the detection box buffer
            box.clear();
            box.add(new Vector2D(0, m_pVehicle.BRadius()));
            box.add(new Vector2D(length, m_pVehicle.BRadius()));
            box.add(new Vector2D(length, -m_pVehicle.BRadius()));
            box.add(new Vector2D(0, -m_pVehicle.BRadius()));


            if (!m_pVehicle.isSmoothingOn()) {
                box = WorldTransform(box, m_pVehicle.Pos(), m_pVehicle.Heading(), m_pVehicle.Side());
                gdi.ClosedShape(box);
            } else {
                box = WorldTransform(box, m_pVehicle.Pos(), m_pVehicle.SmoothedHeading(), m_pVehicle.SmoothedHeading().Perp());
                gdi.ClosedShape(box);
            }


            //////////////////////////////////////////////////////////////////////////
            //the detection box length is proportional to the agent's velocity
            m_dDBoxLength = Prm.MinDetectionBoxLength
                    + (m_pVehicle.Speed() / m_pVehicle.MaxSpeed())
                    * Prm.MinDetectionBoxLength;

            //tag all obstacles within range of the box for processing
            m_pVehicle.World().TagObstaclesWithinViewRange(m_pVehicle, m_dDBoxLength);

            //this will keep track of the closest intersecting obstacle (CIB)
            BaseGameEntity ClosestIntersectingObstacle = null;

            //this will be used to track the distance to the CIB
            double DistToClosestIP = MaxDouble;

            //this will record the transformed local coordinates of the CIB
            Vector2D LocalPosOfClosestObstacle = new Vector2D();

            ListIterator<BaseGameEntity> it = m_pVehicle.World().Obstacles().listIterator();

            while (it.hasNext()) {
                BaseGameEntity curOb = it.next();
                //if the obstacle has been tagged within range proceed
                if (curOb.IsTagged()) {
                    //calculate this obstacle's position in local space
                    Vector2D LocalPos = PointToLocalSpace(curOb.Pos(),
                            m_pVehicle.Heading(),
                            m_pVehicle.Side(),
                            m_pVehicle.Pos());

                    //if the local position has a negative x value then it must lay
                    //behind the agent. (in which case it can be ignored)
                    if (LocalPos.x >= 0) {
                        //if the distance from the x axis to the object's position is less
                        //than its radius + half the width of the detection box then there
                        //is a potential intersection.
                        if (Math.abs(LocalPos.y) < (curOb.BRadius() + m_pVehicle.BRadius())) {
                            gdi.ThickRedPen();
                            gdi.ClosedShape(box);
                        }
                    }
                }
            }


            /////////////////////////////////////////////////////
        }

        //render the wall avoidnace feelers
        if (On(behavior_type.wall_avoidance) && m_pVehicle.World().RenderFeelers()) {
            gdi.OrangePen();

            for (int flr = 0; flr < m_Feelers.size(); ++flr) {

                gdi.Line(m_pVehicle.Pos(), m_Feelers.get(flr));
            }
        }

        //render path info
        if (On(behavior_type.follow_path) && m_pVehicle.World().RenderPath()) {
            m_pPath.Render();
        }


        if (On(behavior_type.separation)) {
            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "Separation(S/X):");
                gdi.TextAtPos(160, NextSlot, ttos(m_dWeightSeparation / Prm.SteeringForceTweaker));
                NextSlot += SlotSize;
            }
            if (KEYDOWN('S')) {
                m_dWeightSeparation += 200 * m_pVehicle.getTimeElapsed();
                m_dWeightSeparation = clamp(m_dWeightSeparation, 0.0, 50.0 * Prm.SteeringForceTweaker);
            }
            if (KEYDOWN('X')) {
                m_dWeightSeparation -= 200 * m_pVehicle.getTimeElapsed();
                m_dWeightSeparation = clamp(m_dWeightSeparation, 0.0, 50.0 * Prm.SteeringForceTweaker);
            }
        }

        if (On(behavior_type.allignment)) {
            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "Alignment(A/Z):");
                gdi.TextAtPos(160, NextSlot, ttos(m_dWeightAlignment / Prm.SteeringForceTweaker));
                NextSlot += SlotSize;
            }

            if (KEYDOWN('A')) {
                m_dWeightAlignment += 200 * m_pVehicle.getTimeElapsed();
                m_dWeightAlignment = clamp(m_dWeightAlignment, 0.0, 50.0 * Prm.SteeringForceTweaker);
            }
            if (KEYDOWN('Z')) {
                m_dWeightAlignment -= 200 * m_pVehicle.getTimeElapsed();
                m_dWeightAlignment = clamp(m_dWeightAlignment, 0.0, 50.0 * Prm.SteeringForceTweaker);
            }

        }

        if (On(behavior_type.cohesion)) {
            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "Cohesion(D/C):");
                gdi.TextAtPos(160, NextSlot, ttos(m_dWeightCohesion / Prm.SteeringForceTweaker));
                NextSlot += SlotSize;
            }

            if (KEYDOWN('D')) {
                m_dWeightCohesion += 200 * m_pVehicle.getTimeElapsed();
                m_dWeightCohesion = clamp(m_dWeightCohesion, 0.0, 50.0 * Prm.SteeringForceTweaker);
            }
            if (KEYDOWN('C')) {
                m_dWeightCohesion -= 200 * m_pVehicle.getTimeElapsed();
                m_dWeightCohesion = clamp(m_dWeightCohesion, 0.0, 50.0 * Prm.SteeringForceTweaker);
            }
        }

        if (On(behavior_type.follow_path)) {
            double sd = Math.sqrt(m_dWaypointSeekDistSq);
            if (m_pVehicle.ID() == 0) {
                gdi.TextAtPos(5, NextSlot, "SeekDistance(D/C):");
                gdi.TextAtPos(160, NextSlot, ttos(sd));
                NextSlot += SlotSize;
            }

            if (KEYDOWN('D')) {
                m_dWaypointSeekDistSq += 1.0;
            }
            if (KEYDOWN('C')) {
                m_dWaypointSeekDistSq -= 1.0;
                m_dWaypointSeekDistSq = clamp(m_dWaypointSeekDistSq, 0.0, 400.0);
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////// CALCULATE METHODS 
    /**
     * calculates the accumulated steering force according to the method set
     *  in m_SummingMethod
     */
    public Vector2D Calculate() {
        //reset the steering force
        m_vSteeringForce.Zero();

        //use space partitioning to calculate the neighbours of this vehicle
        //if switched on. If not, use the standard tagging system
        if (!isSpacePartitioningOn()) {
            //tag neighbors if any of the following 3 group behaviors are switched on
            if (On(behavior_type.separation) || On(behavior_type.allignment) || On(behavior_type.cohesion)) {
                m_pVehicle.World().TagVehiclesWithinViewRange(m_pVehicle, m_dViewDistance);
            }
        } else {
            //calculate neighbours in cell-space if any of the following 3 group
            //behaviors are switched on
            if (On(behavior_type.separation) || On(behavior_type.allignment) || On(behavior_type.cohesion)) {
                m_pVehicle.World().CellSpace().CalculateNeighbors(m_pVehicle.Pos(), m_dViewDistance);
            }
        }

        switch (m_SummingMethod) {

            case weighted_average:

                m_vSteeringForce = CalculateWeightedSum();
                break;

            case prioritized:

                m_vSteeringForce = CalculatePrioritized();
                break;

            case dithered:

                m_vSteeringForce = CalculateDithered();
                break;

            default:
                m_vSteeringForce = new Vector2D(0, 0);

        }//end switch

        return m_vSteeringForce;
    }

    /**
     * returns the forward component of the steering force
     */
    public double ForwardComponent() {
        return m_pVehicle.Heading().Dot(m_vSteeringForce);
    }

    /**
     * returns the side component of the steering force
     */
    public double SideComponent() {
        return m_pVehicle.Side().Dot(m_vSteeringForce);
    }

    /**
     *  this method calls each active steering behavior in order of priority
     *  and acumulates their forces until the max steering force magnitude
     *  is reached, at which time the function returns the steering force 
     *  accumulated to that  point
     */
    private Vector2D CalculatePrioritized() {
        Vector2D force = new Vector2D();

        if (On(behavior_type.wall_avoidance)) {
            force = mul(WallAvoidance(m_pVehicle.World().Walls()),
                    m_dWeightWallAvoidance);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.obstacle_avoidance)) {
            force = mul(ObstacleAvoidance(m_pVehicle.World().Obstacles()),
                    m_dWeightObstacleAvoidance);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.evade)) {
            assert m_pTargetAgent1 != null : "Evade target not assigned";

            force = mul(Evade(m_pTargetAgent1), m_dWeightEvade);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }


        if (On(behavior_type.flee)) {
            force = mul(Flee(m_pVehicle.World().Crosshair()), m_dWeightFlee);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }


        //these next three can be combined for flocking behavior (wander is
        //also a good behavior to add into this mix)
        if (!isSpacePartitioningOn()) {
            if (On(behavior_type.separation)) {
                force = mul(Separation(m_pVehicle.World().Agents()), m_dWeightSeparation);

                if (!AccumulateForce(m_vSteeringForce, force)) {
                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.allignment)) {
                force = mul(Alignment(m_pVehicle.World().Agents()), m_dWeightAlignment);

                if (!AccumulateForce(m_vSteeringForce, force)) {
                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.cohesion)) {
                force = mul(Cohesion(m_pVehicle.World().Agents()), m_dWeightCohesion);

                if (!AccumulateForce(m_vSteeringForce, force)) {
                    return m_vSteeringForce;
                }
            }
        } else {

            if (On(behavior_type.separation)) {
                force = mul(SeparationPlus(m_pVehicle.World().Agents()), m_dWeightSeparation);

                if (!AccumulateForce(m_vSteeringForce, force)) {
                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.allignment)) {
                force = mul(AlignmentPlus(m_pVehicle.World().Agents()), m_dWeightAlignment);

                if (!AccumulateForce(m_vSteeringForce, force)) {
                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.cohesion)) {
                force = mul(CohesionPlus(m_pVehicle.World().Agents()), m_dWeightCohesion);

                if (!AccumulateForce(m_vSteeringForce, force)) {
                    return m_vSteeringForce;
                }
            }
        }

        if (On(behavior_type.seek)) {
            force = mul(Seek(m_pVehicle.World().Crosshair()), m_dWeightSeek);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }


        if (On(behavior_type.arrive)) {
            force = mul(Arrive(m_pVehicle.World().Crosshair(), m_Deceleration), m_dWeightArrive);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.wander)) {
            force = mul(Wander(), m_dWeightWander);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.pursuit)) {
            assert m_pTargetAgent1 != null : "pursuit target not assigned";

            force = mul(Pursuit(m_pTargetAgent1), m_dWeightPursuit);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.offset_pursuit)) {
            assert m_pTargetAgent1 != null : "pursuit target not assigned";
            assert !m_vOffset.isZero() : "No offset assigned";

            force = OffsetPursuit(m_pTargetAgent1, m_vOffset);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.interpose)) {
            assert m_pTargetAgent1 != null && m_pTargetAgent2 != null : "Interpose agents not assigned";

            force = mul(Interpose(m_pTargetAgent1, m_pTargetAgent2), m_dWeightInterpose);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.hide)) {
            assert m_pTargetAgent1 != null : "Hide target not assigned";

            force = mul(Hide(m_pTargetAgent1, m_pVehicle.World().Obstacles()), m_dWeightHide);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }


        if (On(behavior_type.follow_path)) {
            force = mul(FollowPath(), m_dWeightFollowPath);

            if (!AccumulateForce(m_vSteeringForce, force)) {
                return m_vSteeringForce;
            }
        }

        return m_vSteeringForce;
    }

    /**
     *  this simply sums up all the active behaviors X their weights and 
     *  truncates the result to the max available steering force before 
     *  returning
     */
    private Vector2D CalculateWeightedSum() {
        if (On(behavior_type.wall_avoidance)) {
            m_vSteeringForce.add(mul(WallAvoidance(m_pVehicle.World().Walls()),
                    m_dWeightWallAvoidance));
        }

        if (On(behavior_type.obstacle_avoidance)) {
            m_vSteeringForce.add(mul(ObstacleAvoidance(m_pVehicle.World().Obstacles()),
                    m_dWeightObstacleAvoidance));
        }

        if (On(behavior_type.evade)) {
            assert m_pTargetAgent1 != null : "Evade target not assigned";

            m_vSteeringForce.add(mul(Evade(m_pTargetAgent1), m_dWeightEvade));
        }


        //these next three can be combined for flocking behavior (wander is
        //also a good behavior to add into this mix)
        if (!isSpacePartitioningOn()) {
            if (On(behavior_type.separation)) {
                m_vSteeringForce.add(mul(Separation(m_pVehicle.World().Agents()), m_dWeightSeparation));
            }

            if (On(behavior_type.allignment)) {
                m_vSteeringForce.add(mul(Alignment(m_pVehicle.World().Agents()), m_dWeightAlignment));
            }

            if (On(behavior_type.cohesion)) {
                m_vSteeringForce.add(mul(Cohesion(m_pVehicle.World().Agents()), m_dWeightCohesion));
            }
        } else {
            if (On(behavior_type.separation)) {
                m_vSteeringForce.add(mul(SeparationPlus(m_pVehicle.World().Agents()), m_dWeightSeparation));
            }

            if (On(behavior_type.allignment)) {
                m_vSteeringForce.add(mul(AlignmentPlus(m_pVehicle.World().Agents()), m_dWeightAlignment));
            }

            if (On(behavior_type.cohesion)) {
                m_vSteeringForce.add(mul(CohesionPlus(m_pVehicle.World().Agents()), m_dWeightCohesion));
            }
        }


        if (On(behavior_type.wander)) {
            m_vSteeringForce.add(mul(Wander(), m_dWeightWander));
        }

        if (On(behavior_type.seek)) {
            m_vSteeringForce.add(mul(Seek(m_pVehicle.World().Crosshair()), m_dWeightSeek));
        }

        if (On(behavior_type.flee)) {
            m_vSteeringForce.add(mul(Flee(m_pVehicle.World().Crosshair()), m_dWeightFlee));
        }

        if (On(behavior_type.arrive)) {
            m_vSteeringForce.add(mul(Arrive(m_pVehicle.World().Crosshair(), m_Deceleration), m_dWeightArrive));
        }

        if (On(behavior_type.pursuit)) {
            assert m_pTargetAgent1 != null : "pursuit target not assigned";

            m_vSteeringForce.add(mul(Pursuit(m_pTargetAgent1), m_dWeightPursuit));
        }

        if (On(behavior_type.offset_pursuit)) {
            assert m_pTargetAgent1 != null : "pursuit target not assigned";
            assert !m_vOffset.isZero() : "No offset assigned";

            m_vSteeringForce.add(mul(OffsetPursuit(m_pTargetAgent1, m_vOffset), m_dWeightOffsetPursuit));
        }

        if (On(behavior_type.interpose)) {
            assert m_pTargetAgent1 != null && m_pTargetAgent2 != null : "Interpose agents not assigned";

            m_vSteeringForce.add(mul(Interpose(m_pTargetAgent1, m_pTargetAgent2), m_dWeightInterpose));
        }

        if (On(behavior_type.hide)) {
            assert m_pTargetAgent1 != null : "Hide target not assigned";

            m_vSteeringForce.add(mul(Hide(m_pTargetAgent1, m_pVehicle.World().Obstacles()), m_dWeightHide));
        }

        if (On(behavior_type.follow_path)) {
            m_vSteeringForce.add(mul(FollowPath(), m_dWeightFollowPath));
        }

        m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

        return m_vSteeringForce;
    }

    /**
     *  this method sums up the active behaviors by assigning a probabilty
     *  of being calculated to each behavior. It then tests the first priority
     *  to see if it should be calcukated this simulation-step. If so, it
     *  calculates the steering force resulting from this behavior. If it is
     *  more than zero it returns the force. If zero, or if the behavior is
     *  skipped it continues onto the next priority, and so on.
     *
     *  NOTE: Not all of the behaviors have been implemented in this method,
     *        just a few, so you get the general idea
     */
    private Vector2D CalculateDithered() {
        //reset the steering force
        m_vSteeringForce.Zero();

        if (On(behavior_type.wall_avoidance) && RandFloat() < Prm.prWallAvoidance) {
            m_vSteeringForce = mul(WallAvoidance(m_pVehicle.World().Walls()),
                    m_dWeightWallAvoidance / Prm.prWallAvoidance);

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.obstacle_avoidance) && RandFloat() < Prm.prObstacleAvoidance) {
            m_vSteeringForce.add(mul(ObstacleAvoidance(m_pVehicle.World().Obstacles()),
                    m_dWeightObstacleAvoidance / Prm.prObstacleAvoidance));

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }

        if (!isSpacePartitioningOn()) {
            if (On(behavior_type.separation) && RandFloat() < Prm.prSeparation) {
                m_vSteeringForce.add(mul(Separation(m_pVehicle.World().Agents()),
                        m_dWeightSeparation / Prm.prSeparation));

                if (!m_vSteeringForce.isZero()) {
                    m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                    return m_vSteeringForce;
                }
            }
        } else {
            if (On(behavior_type.separation) && RandFloat() < Prm.prSeparation) {
                m_vSteeringForce.add(mul(SeparationPlus(m_pVehicle.World().Agents()),
                        m_dWeightSeparation / Prm.prSeparation));

                if (!m_vSteeringForce.isZero()) {
                    m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                    return m_vSteeringForce;
                }
            }
        }


        if (On(behavior_type.flee) && RandFloat() < Prm.prFlee) {
            m_vSteeringForce.add(mul(Flee(m_pVehicle.World().Crosshair()), m_dWeightFlee / Prm.prFlee));

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.evade) && RandFloat() < Prm.prEvade) {
            assert m_pTargetAgent1 != null : "Evade target not assigned";

            m_vSteeringForce.add(mul(Evade(m_pTargetAgent1), m_dWeightEvade / Prm.prEvade));

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }


        if (!isSpacePartitioningOn()) {
            if (On(behavior_type.allignment) && RandFloat() < Prm.prAlignment) {
                m_vSteeringForce.add(mul(Alignment(m_pVehicle.World().Agents()),
                        m_dWeightAlignment / Prm.prAlignment));

                if (!m_vSteeringForce.isZero()) {
                    m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.cohesion) && RandFloat() < Prm.prCohesion) {
                m_vSteeringForce.add(mul(Cohesion(m_pVehicle.World().Agents()),
                        m_dWeightCohesion / Prm.prCohesion));

                if (!m_vSteeringForce.isZero()) {
                    m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                    return m_vSteeringForce;
                }
            }
        } else {
            if (On(behavior_type.allignment) && RandFloat() < Prm.prAlignment) {
                m_vSteeringForce.add(mul(AlignmentPlus(m_pVehicle.World().Agents()),
                        m_dWeightAlignment / Prm.prAlignment));

                if (!m_vSteeringForce.isZero()) {
                    m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.cohesion) && RandFloat() < Prm.prCohesion) {
                m_vSteeringForce.add(mul(CohesionPlus(m_pVehicle.World().Agents()),
                        m_dWeightCohesion / Prm.prCohesion));

                if (!m_vSteeringForce.isZero()) {
                    m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                    return m_vSteeringForce;
                }
            }
        }

        if (On(behavior_type.wander) && RandFloat() < Prm.prWander) {
            m_vSteeringForce.add(mul(Wander(), m_dWeightWander / Prm.prWander));

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.seek) && RandFloat() < Prm.prSeek) {
            m_vSteeringForce.add(mul(Seek(m_pVehicle.World().Crosshair()), m_dWeightSeek / Prm.prSeek));

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }

        if (On(behavior_type.arrive) && RandFloat() < Prm.prArrive) {
            m_vSteeringForce.add(mul(Arrive(m_pVehicle.World().Crosshair(), m_Deceleration),
                    m_dWeightArrive / Prm.prArrive));

            if (!m_vSteeringForce.isZero()) {
                m_vSteeringForce.Truncate(m_pVehicle.MaxForce());

                return m_vSteeringForce;
            }
        }

        return m_vSteeringForce;
    }

    public void SetTarget(final Vector2D t) {
        m_vTarget = new Vector2D(t);
    }

    public void SetTargetAgent1(Vehicle Agent) {
        m_pTargetAgent1 = Agent;
    }

    public void SetTargetAgent2(Vehicle Agent) {
        m_pTargetAgent2 = Agent;
    }

    public void SetOffset(final Vector2D offset) {
        m_vOffset = offset;
    }

    public Vector2D GetOffset() {
        return m_vOffset;
    }

    public void SetPath(List<Vector2D> new_path) {
        m_pPath.Set(new_path);
    }

    public void CreateRandomPath(int num_waypoints, int mx, int my, int cx, int cy) {
        m_pPath.CreateRandomPath(num_waypoints, mx, my, cx, cy);
    }

    public Vector2D Force() {
        return m_vSteeringForce;
    }

    public void ToggleSpacePartitioningOnOff() {
        m_bCellSpaceOn = !m_bCellSpaceOn;
    }

    public boolean isSpacePartitioningOn() {
        return m_bCellSpaceOn;
    }

    public void SetSummingMethod(summing_method sm) {
        m_SummingMethod = sm;
    }

    public void FleeOn() {
        m_iFlags |= behavior_type.flee.flag();
    }

    public void SeekOn() {
        m_iFlags |= behavior_type.seek.flag();
    }

    public void ArriveOn() {
        m_iFlags |= behavior_type.arrive.flag();
    }

    public void WanderOn() {
        m_iFlags |= behavior_type.wander.flag();
    }

    public void PursuitOn(Vehicle v) {
        m_iFlags |= behavior_type.pursuit.flag();
        m_pTargetAgent1 = v;
    }

    public void EvadeOn(Vehicle v) {
        m_iFlags |= behavior_type.evade.flag();
        m_pTargetAgent1 = v;
    }

    public void CohesionOn() {
        m_iFlags |= behavior_type.cohesion.flag();
    }

    public void SeparationOn() {
        m_iFlags |= behavior_type.separation.flag();
    }

    public void AlignmentOn() {
        m_iFlags |= behavior_type.allignment.flag();
    }

    public void ObstacleAvoidanceOn() {
        m_iFlags |= behavior_type.obstacle_avoidance.flag();
    }

    public void WallAvoidanceOn() {
        m_iFlags |= behavior_type.wall_avoidance.flag();
    }

    public void FollowPathOn() {
        m_iFlags |= behavior_type.follow_path.flag();
    }

    public void InterposeOn(Vehicle v1, Vehicle v2) {
        m_iFlags |= behavior_type.interpose.flag();
        m_pTargetAgent1 = v1;
        m_pTargetAgent2 = v2;
    }

    public void HideOn(Vehicle v) {
        m_iFlags |= behavior_type.hide.flag();
        m_pTargetAgent1 = v;
    }

    public void OffsetPursuitOn(Vehicle v1, final Vector2D offset) {
        m_iFlags |= behavior_type.offset_pursuit.flag();
        m_vOffset = offset;
        m_pTargetAgent1 = v1;
    }

    public void FlockingOn() {
        CohesionOn();
        AlignmentOn();
        SeparationOn();
        WanderOn();
    }

    public void FleeOff() {
        if (On(behavior_type.flee)) {
            m_iFlags ^= behavior_type.flee.flag();
        }
    }

    public void SeekOff() {
        if (On(behavior_type.seek)) {
            m_iFlags ^= behavior_type.seek.flag();
        }
    }

    public void ArriveOff() {
        if (On(behavior_type.arrive)) {
            m_iFlags ^= behavior_type.arrive.flag();
        }
    }

    public void WanderOff() {
        if (On(behavior_type.wander)) {
            m_iFlags ^= behavior_type.wander.flag();
        }
    }

    public void PursuitOff() {
        if (On(behavior_type.pursuit)) {
            m_iFlags ^= behavior_type.pursuit.flag();
        }
    }

    public void EvadeOff() {
        if (On(behavior_type.evade)) {
            m_iFlags ^= behavior_type.evade.flag();
        }
    }

    public void CohesionOff() {
        if (On(behavior_type.cohesion)) {
            m_iFlags ^= behavior_type.cohesion.flag();
        }
    }

    public void SeparationOff() {
        if (On(behavior_type.separation)) {
            m_iFlags ^= behavior_type.separation.flag();
        }
    }

    public void AlignmentOff() {
        if (On(behavior_type.allignment)) {
            m_iFlags ^= behavior_type.allignment.flag();
        }
    }

    public void ObstacleAvoidanceOff() {
        if (On(behavior_type.obstacle_avoidance)) {
            m_iFlags ^= behavior_type.obstacle_avoidance.flag();
        }
    }

    public void WallAvoidanceOff() {
        if (On(behavior_type.wall_avoidance)) {
            m_iFlags ^= behavior_type.wall_avoidance.flag();
        }
    }

    public void FollowPathOff() {
        if (On(behavior_type.follow_path)) {
            m_iFlags ^= behavior_type.follow_path.flag();
        }
    }

    public void InterposeOff() {
        if (On(behavior_type.interpose)) {
            m_iFlags ^= behavior_type.interpose.flag();
        }
    }

    public void HideOff() {
        if (On(behavior_type.hide)) {
            m_iFlags ^= behavior_type.hide.flag();
        }
    }

    public void OffsetPursuitOff() {
        if (On(behavior_type.offset_pursuit)) {
            m_iFlags ^= behavior_type.offset_pursuit.flag();
        }
    }

    public void FlockingOff() {
        CohesionOff();
        AlignmentOff();
        SeparationOff();
        WanderOff();
    }

    public boolean isFleeOn() {
        return On(behavior_type.flee);
    }

    public boolean isSeekOn() {
        return On(behavior_type.seek);
    }

    public boolean isArriveOn() {
        return On(behavior_type.arrive);
    }

    public boolean isWanderOn() {
        return On(behavior_type.wander);
    }

    public boolean isPursuitOn() {
        return On(behavior_type.pursuit);
    }

    public boolean isEvadeOn() {
        return On(behavior_type.evade);
    }

    public boolean isCohesionOn() {
        return On(behavior_type.cohesion);
    }

    public boolean isSeparationOn() {
        return On(behavior_type.separation);
    }

    public boolean isAlignmentOn() {
        return On(behavior_type.allignment);
    }

    public boolean isObstacleAvoidanceOn() {
        return On(behavior_type.obstacle_avoidance);
    }

    public boolean isWallAvoidanceOn() {
        return On(behavior_type.wall_avoidance);
    }

    public boolean isFollowPathOn() {
        return On(behavior_type.follow_path);
    }

    public boolean isInterposeOn() {
        return On(behavior_type.interpose);
    }

    public boolean isHideOn() {
        return On(behavior_type.hide);
    }

    public boolean isOffsetPursuitOn() {
        return On(behavior_type.offset_pursuit);
    }

    public double DBoxLength() {
        return m_dDBoxLength;
    }

    public List<Vector2D> GetFeelers() {
        return m_Feelers;
    }

    public double WanderJitter() {
        return m_dWanderJitter;
    }

    public double WanderDistance() {
        return m_dWanderDistance;
    }

    public double WanderRadius() {
        return m_dWanderRadius;
    }

    public double SeparationWeight() {
        return m_dWeightSeparation;
    }

    public double AlignmentWeight() {
        return m_dWeightAlignment;
    }

    public double CohesionWeight() {
        return m_dWeightCohesion;
    }
}
