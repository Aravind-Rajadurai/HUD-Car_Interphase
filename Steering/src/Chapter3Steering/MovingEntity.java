/**
 *  Desc:   A base class defining an entity that moves. The entity has 
 *          a local coordinate system and members for defining its
 *          mass and velocity.
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import common.D2.Vector2D;
import common.D2.C2DMatrix;
import static common.D2.Vector2D.*;

//
//  Author: Mat Buckland 2003 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------
public class MovingEntity extends BaseGameEntity {

    protected Vector2D m_vVelocity;
    //a normalized vector pointing in the direction the entity is heading. 
    protected Vector2D m_vHeading;
    //a vector perpendicular to the heading vector
    protected Vector2D m_vSide;
    protected double m_dMass;
    //the maximum speed this entity may travel at.
    protected double m_dMaxSpeed;
    //the maximum force this entity can produce to power itself 
    //(think rockets and thrust)
    protected double m_dMaxForce;
    //the maximum rate (radians per second)this vehicle can rotate         
    protected double m_dMaxTurnRate;

    public MovingEntity(Vector2D position,
            double radius,
            Vector2D velocity,
            double max_speed,
            Vector2D heading,
            double mass,
            Vector2D scale,
            double turn_rate,
            double max_force) {
        super(0, position, radius);
        m_vHeading = new Vector2D(heading);
        m_vVelocity = new Vector2D(velocity);
        m_dMass = mass;
        m_vSide = m_vHeading.Perp();
        m_dMaxSpeed = max_speed;
        m_dMaxTurnRate = turn_rate;
        m_dMaxForce = max_force;
        m_vScale = new Vector2D(scale);
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }

    //accessors
    public Vector2D Velocity() {
        return new Vector2D(m_vVelocity);
    }

    public void SetVelocity(Vector2D NewVel) {
        m_vVelocity = NewVel;
    }

    public double Mass() {
        return m_dMass;
    }

    public Vector2D Side() {
        return m_vSide;
    }

    public double MaxSpeed() {
        return m_dMaxSpeed;
    }

    public void SetMaxSpeed(double new_speed) {
        m_dMaxSpeed = new_speed;
    }

    public double MaxForce() {
        return m_dMaxForce;
    }

    public void SetMaxForce(double mf) {
        m_dMaxForce = mf;
    }

    public boolean IsSpeedMaxedOut() {
        return m_dMaxSpeed * m_dMaxSpeed >= m_vVelocity.LengthSq();
    }

    public double Speed() {
        return m_vVelocity.Length();
    }

    public double SpeedSq() {
        return m_vVelocity.LengthSq();
    }

    public Vector2D Heading() {
        return m_vHeading;
    }

    double MaxTurnRate() {
        return m_dMaxTurnRate;
    }

    void SetMaxTurnRate(double val) {
        m_dMaxTurnRate = val;
    }

    /**
     *  given a target position, this method rotates the entity's heading and
     *  side vectors by an amount not greater than m_dMaxTurnRate until it
     *  directly faces the target.
     *
     *  @return true when the heading is facing in the desired direction
     */
    public boolean RotateHeadingToFacePosition(Vector2D target) {
        Vector2D toTarget = Vec2DNormalize(sub(target, m_vPos));

        //first determine the angle between the heading vector and the target
        double angle = Math.acos(m_vHeading.Dot(toTarget));
        if(Double.isNaN(angle))  {
            angle = 0;
        }

        //return true if the player is facing the target
        if (angle < 0.00001) {
            return true;
        }

        //clamp the amount to turn to the max turn rate
        if (angle > m_dMaxTurnRate) {
            angle = m_dMaxTurnRate;
        }

        //The next few lines use a rotation matrix to rotate the player's heading
        //vector accordingly
        C2DMatrix RotationMatrix = new C2DMatrix();

        //notice how the direction of rotation has to be determined when creating
        //the rotation matrix
        RotationMatrix.Rotate(angle * m_vHeading.Sign(toTarget));
        RotationMatrix.TransformVector2Ds(m_vHeading);
        RotationMatrix.TransformVector2Ds(m_vVelocity);

        //finally recreate m_vSide
        m_vSide = m_vHeading.Perp();

        return false;
    }

    /**
     *  first checks that the given heading is not a vector of zero length. If the
     *  new heading is valid this fumction sets the entity's heading and side 
     *  vectors accordingly
     */
    public void SetHeading(Vector2D new_heading) {
        assert ((new_heading.LengthSq() - 1.0) < 0.00001);

        m_vHeading = new_heading;

        //the side vector must always be perpendicular to the heading
        m_vSide = m_vHeading.Perp();
    }
}