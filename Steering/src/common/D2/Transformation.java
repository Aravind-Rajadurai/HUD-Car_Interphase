/**
 *  Desc:   Functions for converting 2D vectors between World and Local
 *          space.
 *
 * @author Petr (http://www.sallyx.org/)
 */
package common.D2;

import java.util.List;
import static common.D2.Vector2D.*;
import common.misc.CppToJava;
import java.util.ArrayList;

public class Transformation {
//--------------------------- WorldTransform -----------------------------
//
//  given a std::vector of 2D vectors, a position, orientation and scale,
//  this function transforms the 2D vectors into the object's world space
//------------------------------------------------------------------------
    public static List<Vector2D> WorldTransform(List<Vector2D> points,
            Vector2D pos,
            Vector2D forward,
            Vector2D side,
            Vector2D scale) {
        //copy the original vertices into the buffer about to be transformed
        List<Vector2D> TranVector2Ds = CppToJava.clone(points);

        //create a transformation matrix
        C2DMatrix matTransform = new C2DMatrix();

        //scale
        if ((scale.x != 1.0) || (scale.y != 1.0)) {
            matTransform.Scale(scale.x, scale.y);
        }

        //rotate
        matTransform.Rotate(forward, side);

        //and translate
        matTransform.Translate(pos.x, pos.y);

        //now transform the object's vertices
        matTransform.TransformVector2Ds(TranVector2Ds);

        return TranVector2Ds;
    }

/**
 *  given a std::vector of 2D vectors, a position and  orientation
 *  this function transforms the 2D vectors into the object's world space
 */
    public static List<Vector2D> WorldTransform(List<Vector2D> points,
            Vector2D pos,
            Vector2D forward,
            Vector2D side) {
        //copy the original vertices into the buffer about to be transformed
        List<Vector2D> TranVector2Ds = CppToJava.clone(points);
        for(Vector2D v: points) {
            TranVector2Ds.add(v);
        }

        //create a transformation matrix
        C2DMatrix matTransform = new C2DMatrix();

        //rotate
        matTransform.Rotate(forward, side);

        //and translate
        matTransform.Translate(pos.x, pos.y);

        //now transform the object's vertices
        matTransform.TransformVector2Ds(TranVector2Ds);

        return TranVector2Ds;
    }

//--------------------- PointToWorldSpace --------------------------------
//
//  Transforms a point from the agent's local space into world space
//------------------------------------------------------------------------
    public static Vector2D PointToWorldSpace(Vector2D point,
            Vector2D AgentHeading,
            Vector2D AgentSide,
            Vector2D AgentPosition) {
        //make a copy of the point
        Vector2D TransPoint = new Vector2D(point);

        //create a transformation matrix
        C2DMatrix matTransform = new C2DMatrix();

        //rotate
        matTransform.Rotate(AgentHeading, AgentSide);

        //and translate
        matTransform.Translate(AgentPosition.x, AgentPosition.y);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }

//--------------------- VectorToWorldSpace --------------------------------
//
//  Transforms a vector from the agent's local space into world space
//------------------------------------------------------------------------
    public static Vector2D VectorToWorldSpace(Vector2D vec,
            Vector2D AgentHeading,
            Vector2D AgentSide) {
        //make a copy of the point
        Vector2D TransVec = new Vector2D(vec);

        //create a transformation matrix
        C2DMatrix matTransform = new C2DMatrix();

        //rotate
        matTransform.Rotate(AgentHeading, AgentSide);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransVec);

        return TransVec;
    }

//--------------------- PointToLocalSpace --------------------------------
//
//------------------------------------------------------------------------
    public static Vector2D PointToLocalSpace(Vector2D point,
            Vector2D AgentHeading,
            Vector2D AgentSide,
            Vector2D AgentPosition) {

        //make a copy of the point
        Vector2D TransPoint = new Vector2D(point);

        //create a transformation matrix
        C2DMatrix matTransform = new C2DMatrix();

        double Tx = -AgentPosition.Dot(AgentHeading);
        double Ty = -AgentPosition.Dot(AgentSide);

        //create the transformation matrix
        matTransform._11(AgentHeading.x);   matTransform._12(AgentSide.x);
        matTransform._21(AgentHeading.y);   matTransform._22(AgentSide.y);
        matTransform._31(Tx);               matTransform._32(Ty);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }

//--------------------- VectorToLocalSpace --------------------------------
//
//------------------------------------------------------------------------
    public static Vector2D VectorToLocalSpace(Vector2D vec,
            Vector2D AgentHeading,
            Vector2D AgentSide) {

        //make a copy of the point
        Vector2D TransPoint = new Vector2D(vec);

        //create a transformation matrix
        C2DMatrix matTransform = new C2DMatrix();

        //create the transformation matrix
        matTransform._11(AgentHeading.x);   matTransform._12(AgentSide.x);
        matTransform._21(AgentHeading.y);   matTransform._22(AgentSide.y);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }

//-------------------------- Vec2DRotateAroundOrigin --------------------------
//
//  rotates a vector ang rads around the origin
//-----------------------------------------------------------------------------
    public static void Vec2DRotateAroundOrigin(Vector2D v, double ang) {
        //create a transformation matrix
        C2DMatrix mat = new C2DMatrix();

        //rotate
        mat.Rotate(ang);

        //now transform the object's vertices
        mat.TransformVector2Ds(v);
    }

//------------------------ CreateWhiskers ------------------------------------
//
//  given an origin, a facing direction, a 'field of view' describing the 
//  limit of the outer whiskers, a whisker length and the number of whiskers
//  this method returns a vector containing the end positions of a series
//  of whiskers radiating away from the origin and with equal distance between
//  them. (like the spokes of a wheel clipped to a specific segment size)
//----------------------------------------------------------------------------
    public static List<Vector2D> CreateWhiskers(int NumWhiskers,
            double WhiskerLength,
            double fov,
            Vector2D facing,
            Vector2D origin) {
        //this is the magnitude of the angle separating each whisker
        double SectorSize = fov / (double) (NumWhiskers - 1);

        List<Vector2D> whiskers = new ArrayList<Vector2D>(NumWhiskers);
        Vector2D temp;
        double angle = -fov * 0.5;

        for (int w = 0; w < NumWhiskers; ++w) {
            //create the whisker extending outwards at this angle
            temp = facing;
            Vec2DRotateAroundOrigin(temp, angle);
            whiskers.add(add(origin, mul(WhiskerLength, temp)));

            angle += SectorSize;
        }

        return whiskers;
    }
}