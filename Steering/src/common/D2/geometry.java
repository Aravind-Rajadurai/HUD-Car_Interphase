/**
 * Desc:   useful 2D geometry functions
 *
 * @author Petr (http://www.sallyx.org/)
 */
package common.D2;

import java.util.ArrayList;
import static common.D2.Vector2D.*;
import static common.misc.utils.*;
import static common.misc.CppToJava.*;
import static common.D2.Transformation.PointToLocalSpace;

public class geometry {

    /**
     * given a plane and a ray this function determins how far along the ray 
     * an interestion occurs. Returns negative if the ray is parallel
     */
    public static double DistanceToRayPlaneIntersection(Vector2D RayOrigin,
            Vector2D RayHeading,
            Vector2D PlanePoint, //any point on the plane
            Vector2D PlaneNormal) {

        double d = -PlaneNormal.Dot(PlanePoint);
        double numer = PlaneNormal.Dot(RayOrigin) + d;
        double denom = PlaneNormal.Dot(RayHeading);

        // normal is parallel to vector
        if ((denom < 0.000001) && (denom > -0.000001)) {
            return (-1.0);
        }

        return -(numer / denom);
    }

//------------------------- WhereIsPoint --------------------------------------
    public static enum span_type {

        plane_backside, plane_front, on_plane;
    }

    public static span_type WhereIsPoint(Vector2D point,
            Vector2D PointOnPlane, //any point on the plane
            Vector2D PlaneNormal) {
        Vector2D dir = sub(PointOnPlane, point);

        double d = dir.Dot(PlaneNormal);

        if (d < -0.000001) {
            return span_type.plane_front;
        } else if (d > 0.000001) {
            return span_type.plane_backside;
        }

        return span_type.on_plane;
    }
    public static double pi = 3.14159;// Math.PI

    /**
     * GetRayCircleIntersec
     */
    public static double GetRayCircleIntersect(Vector2D RayOrigin,
            Vector2D RayHeading,
            Vector2D CircleOrigin,
            double radius) {
        Vector2D ToCircle = sub(CircleOrigin, RayOrigin);
        double length = ToCircle.Length();
        double v = ToCircle.Dot(RayHeading);
        double d = radius * radius - (length * length - v * v);

        // If there was no intersection, return -1
        if (d < 0.0) {
            return (-1.0);
        }

        // Return the distance to the [first] intersecting point
        return (v - Math.sqrt(d));
    }

    /**
     *  DoRayCircleIntersect
     */
    public static boolean DoRayCircleIntersect(Vector2D RayOrigin,
            Vector2D RayHeading,
            Vector2D CircleOrigin,
            double radius) {

        Vector2D ToCircle = sub(CircleOrigin, RayOrigin);
        double length = ToCircle.Length();
        double v = ToCircle.Dot(RayHeading);
        double d = radius * radius - (length * length - v * v);

        // If there was no intersection, return -1
        return (d < 0.0);
    }

    /**
     *  Given a point P and a circle of radius R centered at C this function
     *  determines the two points on the circle that intersect with the 
     *  tangents from P to the circle. Returns false if P is within the circle.
     *
     *  Thanks to Dave Eberly for this one.
     */
    public static boolean GetTangentPoints(Vector2D C, double R, Vector2D P, Vector2D T1, Vector2D T2) {
        Vector2D PmC = sub(P, C);
        double SqrLen = PmC.LengthSq();
        double RSqr = R * R;
        if (SqrLen <= RSqr) {
            // P is inside or on the circle
            return false;
        }

        double InvSqrLen = 1 / SqrLen;
        double Root = Math.sqrt(Math.abs(SqrLen - RSqr));

        T1.x = C.x + R * (R * PmC.x - PmC.y * Root) * InvSqrLen;
        T1.y = C.y + R * (R * PmC.y + PmC.x * Root) * InvSqrLen;
        T2.x = C.x + R * (R * PmC.x + PmC.y * Root) * InvSqrLen;
        T2.y = C.y + R * (R * PmC.y - PmC.x * Root) * InvSqrLen;

        return true;
    }

    /**
    /* given a line segment AB and a point P, this function calculates the 
    /*  perpendicular distance between them
     */
    public static double DistToLineSegment(Vector2D A,
            Vector2D B,
            Vector2D P) {
        //if the angle is obtuse between PA and AB is obtuse then the closest
        //vertex must be A
        double dotA = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);

        if (dotA <= 0) {
            return Vec2DDistance(A, P);
        }

        //if the angle is obtuse between PB and AB is obtuse then the closest
        //vertex must be B
        double dotB = (P.x - B.x) * (A.x - B.x) + (P.y - B.y) * (A.y - B.y);

        if (dotB <= 0) {
            return Vec2DDistance(B, P);
        }

        //calculate the point along AB that is the closest to P
        //Vector2D Point = A + ((B - A) * dotA)/(dotA + dotB);
        Vector2D Point = add(A, (div(mul(sub(B, A), dotA), (dotA + dotB))));

        //calculate the distance P-Point
        return Vec2DDistance(P, Point);
    }

    /**
     *  as above, but avoiding sqrt
     */
    public static double DistToLineSegmentSq(Vector2D A,
            Vector2D B,
            Vector2D P) {
        //if the angle is obtuse between PA and AB is obtuse then the closest
        //vertex must be A
        double dotA = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);

        if (dotA <= 0) {
            return Vec2DDistanceSq(A, P);
        }

        //if the angle is obtuse between PB and AB is obtuse then the closest
        //vertex must be B
        double dotB = (P.x - B.x) * (A.x - B.x) + (P.y - B.y) * (A.y - B.y);

        if (dotB <= 0) {
            return Vec2DDistanceSq(B, P);
        }

        //calculate the point along AB that is the closest to P
        //Vector2D Point = A + ((B - A) * dotA)/(dotA + dotB);
        Vector2D Point = add(A, (div(mul(sub(B, A), dotA), (dotA + dotB))));

        //calculate the distance P-Point
        return Vec2DDistanceSq(P, Point);
    }

    /**
     *	Given 2 lines in 2D space AB, CD this returns true if an 
     *	intersection occurs.
     */
    public static boolean LineIntersection2D(Vector2D A,
            Vector2D B,
            Vector2D C,
            Vector2D D) {
        double rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
        double sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);

        double Bot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

        if (Bot == 0)//parallel
        {
            return false;
        }

        double invBot = 1.0 / Bot;
        double r = rTop * invBot;
        double s = sTop * invBot;

        if ((r > 0) && (r < 1) && (s > 0) && (s < 1)) {
            //lines intersect
            return true;
        }

        //lines do not intersect
        return false;
    }

    /**
     *  Given 2 lines in 2D space AB, CD this returns true if an 
     *  intersection occurs and sets dist to the distance the intersection
     *  occurs along AB
     */
    public static boolean LineIntersection2D(Vector2D A,
            Vector2D B,
            Vector2D C,
            Vector2D D,
            DoubleRef dist) // double &dist
    {

        double rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
        double sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);

        double Bot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);


        if (Bot == 0)//parallel
        {
            if (isEqual(rTop, 0) && isEqual(sTop, 0)) {
                return true;
            }
            return false;
        }

        double r = rTop / Bot;
        double s = sTop / Bot;

        if ((r > 0) && (r < 1) && (s > 0) && (s < 1)) {
            dist.set(Vec2DDistance(A, B) * r);

            return true;
        } else {
            dist.set(0.0);

            return false;
        }
    }

    /**
     *  Given 2 lines in 2D space AB, CD this returns true if an 
     *  intersection occurs and sets dist to the distance the intersection
     *  occurs along AB. Also sets the 2d vector point to the point of
     *  intersection
     */
    public static boolean LineIntersection2D(Vector2D A,
            Vector2D B,
            Vector2D C,
            Vector2D D,
            DoubleRef dist,
            Vector2D point) {

        double rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
        double rBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

        double sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);
        double sBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

        if ((rBot == 0) || (sBot == 0)) {
            //lines are parallel
            return false;
        }

        double r = rTop / rBot;
        double s = sTop / sBot;

        if ((r > 0) && (r < 1) && (s > 0) && (s < 1)) {
            dist.set(Vec2DDistance(A, B) * r);

            point.set(add(A, mul(r, sub(B, A))));

            return true;
        } else {
            dist.set(0.0);

            return false;
        }
    }

    /**
     *  tests two polygons for intersection. *Does not check for enclosure*
     */
    public static boolean ObjectIntersection2D(ArrayList<Vector2D> object1,
            ArrayList<Vector2D> object2) {
        //test each line segment of object1 against each segment of object2
        for (int r = 0; r < object1.size() - 1; ++r) {
            for (int t = 0; t < object2.size() - 1; ++t) {
                if (LineIntersection2D(object2.get(t),
                        object2.get(t + 1),
                        object1.get(r),
                        object1.get(r + 1))) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     *  tests a line segment against a polygon for intersection
     *  *Does not check for enclosure*
     */
    public static boolean SegmentObjectIntersection2D(final Vector2D A,
            final Vector2D B,
            final ArrayList<Vector2D> object) {
        //test AB against each segment of object
        for (int r = 0; r < object.size() - 1; ++r) {
            if (LineIntersection2D(A, B, object.get(r), object.get(r + 1))) {
                return true;
            }
        }

        return false;
    }

    /**
     *  Returns true if the two circles overlap
     */
    public static boolean TwoCirclesOverlapped(double x1, double y1, double r1,
            double x2, double y2, double r2) {
        double DistBetweenCenters = Math.sqrt((x1 - x2) * (x1 - x2)
                + (y1 - y2) * (y1 - y2));

        if ((DistBetweenCenters < (r1 + r2)) || (DistBetweenCenters < Math.abs(r1 - r2))) {
            return true;
        }

        return false;
    }

    /**
     * Returns true if the two circles overlap
     */
    public static boolean TwoCirclesOverlapped(Vector2D c1, double r1,
            Vector2D c2, double r2) {
        double DistBetweenCenters = Math.sqrt((c1.x - c2.x) * (c1.x - c2.x)
                + (c1.y - c2.y) * (c1.y - c2.y));

        if ((DistBetweenCenters < (r1 + r2)) || (DistBetweenCenters < Math.abs(r1 - r2))) {
            return true;
        }

        return false;
    }

    /**
     *  returns true if one circle encloses the other
     */
    public static boolean TwoCirclesEnclosed(double x1, double y1, double r1,
            double x2, double y2, double r2) {
        double DistBetweenCenters = Math.sqrt((x1 - x2) * (x1 - x2)
                + (y1 - y2) * (y1 - y2));

        if (DistBetweenCenters < Math.abs(r1 - r2)) {
            return true;
        }

        return false;
    }

    /**
     * Given two circles this function calculates the intersection points
     *  of any overlap.
     *
     *  returns false if no overlap found
     *
     * see http://astronomy.swin.edu.au/~pbourke/geometry/2circle/
     */
    public static boolean TwoCirclesIntersectionPoints(double x1, double y1, double r1,
            double x2, double y2, double r2,
            DoubleRef p3X, DoubleRef p3Y,
            DoubleRef p4X, DoubleRef p4Y) {
        //first check to see if they overlap
        if (!TwoCirclesOverlapped(x1, y1, r1, x2, y2, r2)) {
            return false;
        }

        //calculate the distance between the circle centers
        double d = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        //Now calculate the distance from the center of each circle to the center
        //of the line which connects the intersection points.
        double a = (r1 - r2 + (d * d)) / (2 * d);
        double b = (r2 - r1 + (d * d)) / (2 * d);


        //MAYBE A TEST FOR EXACT OVERLAP? 

        //calculate the point P2 which is the center of the line which 
        //connects the intersection points
        double p2X, p2Y;

        p2X = x1 + a * (x2 - x1) / d;
        p2Y = y1 + a * (y2 - y1) / d;

        //calculate first point
        double h1 = Math.sqrt((r1 * r1) - (a * a));

        p3X.set(p2X - h1 * (y2 - y1) / d);
        p3Y.set(p2Y + h1 * (x2 - x1) / d);


        //calculate second point
        double h2 = Math.sqrt((r2 * r2) - (a * a));

        p4X.set(p2X + h2 * (y2 - y1) / d);
        p4Y.set(p2Y - h2 * (x2 - x1) / d);

        return true;

    }

    /**
     *  Tests to see if two circles overlap and if so calculates the area
     *  defined by the union
     *
     * see http://mathforum.org/library/drmath/view/54785.html
     */
    public static double TwoCirclesIntersectionArea(double x1, double y1, double r1,
            double x2, double y2, double r2) {
        //first calculate the intersection points
        double iX1 = 0.0, iY1 = 0.0, iX2 = 0.0, iY2 = 0.0;

        if (!TwoCirclesIntersectionPoints(x1, y1, r1, x2, y2, r2,
                new DoubleRef(iX1), new DoubleRef(iY1), new DoubleRef(iX2), new DoubleRef(iY2))) {
            return 0.0; //no overlap
        }

        //calculate the distance between the circle centers
        double d = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        //find the angles given that A and B are the two circle centers
        //and C and D are the intersection points
        double CBD = 2 * Math.acos((r2 * r2 + d * d - r1 * r1) / (r2 * d * 2));

        double CAD = 2 * Math.acos((r1 * r1 + d * d - r2 * r2) / (r1 * d * 2));


        //Then we find the segment of each of the circles cut off by the 
        //chord CD, by taking the area of the sector of the circle BCD and
        //subtracting the area of triangle BCD. Similarly we find the area
        //of the sector ACD and subtract the area of triangle ACD.

        double area = 0.5f * CBD * r2 * r2 - 0.5f * r2 * r2 * Math.sin(CBD)
                + 0.5f * CAD * r1 * r1 - 0.5f * r1 * r1 * Math.sin(CAD);

        return area;
    }

    /**
     *  given the radius, calculates the area of a circle
     */
    public static double CircleArea(double radius) {
        return pi * radius * radius;
    }

    /**
     *  returns true if the point p is within the radius of the given circle
     */
    public static boolean PointInCircle(Vector2D Pos,
            double radius,
            Vector2D p) {
        double DistFromCenterSquared = (sub(p, Pos)).LengthSq();

        if (DistFromCenterSquared < (radius * radius)) {
            return true;
        }

        return false;
    }

    /**
     * returns true if the line segemnt AB intersects with a circle at
     *  position P with radius radius
     */
    public static boolean LineSegmentCircleIntersection(Vector2D A,
            Vector2D B,
            Vector2D P,
            double radius) {
        //first determine the distance from the center of the circle to
        //the line segment (working in distance squared space)
        double DistToLineSq = DistToLineSegmentSq(A, B, P);

        if (DistToLineSq < radius * radius) {
            return true;
        } else {
            return false;
        }
    }

    /**
     *  given a line segment AB and a circle position and radius, this function
     *  determines if there is an intersection and stores the position of the 
     *  closest intersection in the reference IntersectionPoint
     *
     *  returns false if no intersection point is found
     */
    public static boolean GetLineSegmentCircleClosestIntersectionPoint(Vector2D A,
            Vector2D B,
            Vector2D pos,
            double radius,
            Vector2D IntersectionPoint) {
        Vector2D toBNorm = Vec2DNormalize(sub(B, A));

        //move the circle into the local space defined by the vector B-A with origin
        //at A
        Vector2D LocalPos = PointToLocalSpace(pos, toBNorm, toBNorm.Perp(), A);

        boolean ipFound = false;

        //if the local position + the radius is negative then the circle lays behind
        //point A so there is no intersection possible. If the local x pos minus the 
        //radius is greater than length A-B then the circle cannot intersect the 
        //line segment
        if ((LocalPos.x + radius >= 0)
                && ((LocalPos.x - radius) * (LocalPos.x - radius) <= Vec2DDistanceSq(B, A))) {
            //if the distance from the x axis to the object's position is less
            //than its radius then there is a potential intersection.
            if (Math.abs(LocalPos.y) < radius) {
                //now to do a line/circle intersection test. The center of the 
                //circle is represented by A, B. The intersection points are 
                //given by the formulae x = A +/-sqrt(r^2-B^2), y=0. We only 
                //need to look at the smallest positive value of x.
                double a = LocalPos.x;
                double b = LocalPos.y;

                double ip = a - Math.sqrt(radius * radius - b * b);

                if (ip <= 0) {
                    ip = a + Math.sqrt(radius * radius - b * b);
                }

                ipFound = true;

                IntersectionPoint.set(add(A, mul(toBNorm, ip)));
            }
        }

        return ipFound;
    }
}
