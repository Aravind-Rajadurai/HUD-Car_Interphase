/**
 *   2D vector struct
 *   @author Petr Bilek (http://www.sallyx.org/)
 */
package common.D2;

import common.misc.utils;
import java.util.Scanner;
import java.io.InputStream;
import static common.misc.utils.*;
import common.windows.*;

public class Vector2D {

    public double x;
    public double y;

    public Vector2D() {
        x = 0.0;
        y = 0.0;
    }

    public Vector2D(double a, double b) {
        x = a;
        y = b;
    }

    public Vector2D(Vector2D v) {
        super();
        this.set(v);
    }

    public Vector2D set(Vector2D v) {
        this.x = v.x;
        this.y = v.y;
        return this;
    }
    //sets x and y to zero

    public void Zero() {
        x = 0.0;
        y = 0.0;
    }

    //returns true if both x and y are zero
    public boolean isZero() {
        return (x * x + y * y) < MinDouble;
    }

    /**
     *   returns the length of a 2D vector
     */
    public double Length() {
        return Math.sqrt(x * x + y * y);
    }

    //returns the squared length of the vector (thereby avoiding the sqrt)
    public double LengthSq() {
        return (x * x + y * y);
    }

    /**
     *   normalizes a 2D Vector
     */
    public void Normalize() {
        double vector_length = this.Length();

        if (vector_length > EpsilonDouble) {
            this.x /= vector_length;
            this.y /= vector_length;
        }
    }

    /**
     * calculates the dot product
     * @param v2
     * @return  dot product
     */
    public double Dot(Vector2D v2) {
        return x * v2.x + y * v2.y;
    }
    public static final int clockwise = 1;
    public static final int anticlockwise = -1;

    /**
    /* returns positive if v2 is clockwise of this vector,
    /* negative if anticlockwise (assuming the Y axis is pointing down,
    /* X axis to right like a Window app)
     */
    public int Sign(Vector2D v2) {
        if (y * v2.x > x * v2.y) {
            return anticlockwise;
        } else {
            return clockwise;
        }
    }

    /**
     * returns the vector that is perpendicular to this one.
     */
    public Vector2D Perp() {
        return new Vector2D(-y, x);
    }

    /**
     * adjusts x and y so that the length of the vector does not exceed max
     * truncates a vector so that its length does not exceed max
     * @param max 
     */
    public void Truncate(double max) {
        if (this.Length() > max) {
            this.Normalize();
            this.mul(max);
        }
    }

    /**
     * calculates the euclidean distance between two vectors
     * 
     * @param v2
     * @return the distance between this vector and th one passed as a parameter
     */
    public double Distance(Vector2D v2) {
        double ySeparation = v2.y - y;
        double xSeparation = v2.x - x;

        return Math.sqrt(ySeparation * ySeparation + xSeparation * xSeparation);
    }

    /** 
     * squared version of distance.
     * calculates the euclidean distance squared between two vectors 
     * @param v2
     * @return 
     */
    public double DistanceSq(Vector2D v2) {
        double ySeparation = v2.y - y;
        double xSeparation = v2.x - x;

        return ySeparation * ySeparation + xSeparation * xSeparation;
    }

    /**
     *  given a normalized vector this method reflects the vector it
     *  is operating upon. (like the path of a ball bouncing off a wall)
     * @param norm 
     */
    public void Reflect(Vector2D norm) {
        this.add(norm.GetReverse().mul(2.0 * Dot(norm)));
    }

    /**
     * @return the vector that is the reverse of this vector 
     */
    public Vector2D GetReverse() {
        return new Vector2D(-this.x, -this.y);
    }

    //we need some overloaded operators
    public Vector2D add(Vector2D rhs) {
        x += rhs.x;
        y += rhs.y;

        return this;
    }

    public Vector2D sub(Vector2D rhs) {
        x -= rhs.x;
        y -= rhs.y;

        return this;
    }

    public Vector2D mul(double rhs) {
        x *= rhs;
        y *= rhs;

        return this;
    }

    public Vector2D div(double rhs) {
        x /= rhs;
        y /= rhs;

        return this;
    }

    public boolean isEqual(Vector2D rhs) {
        return (utils.isEqual(x, rhs.x) && utils.isEqual(y, rhs.y));
    }
    // operator !=

    public boolean notEqual(Vector2D rhs) {
        return (x != rhs.x) || (y != rhs.y);
    }

//------------------------------------------------------------------------some more operator overloads
    static public Vector2D mul(Vector2D lhs, double rhs) {
        Vector2D result = new Vector2D(lhs);
        result.mul(rhs);
        return result;
    }

    static public Vector2D mul(double lhs, Vector2D rhs) {
        Vector2D result = new Vector2D(rhs);
        result.mul(lhs);
        return result;
    }

//overload the - operator
    static public Vector2D sub(Vector2D lhs, Vector2D rhs) {
        Vector2D result = new Vector2D(lhs);
        result.x -= rhs.x;
        result.y -= rhs.y;

        return result;
    }

//overload the + operator
    static public Vector2D add(Vector2D lhs, Vector2D rhs) {
        Vector2D result = new Vector2D(lhs);
        result.x += rhs.x;
        result.y += rhs.y;

        return result;
    }

//overload the / operator
    static public Vector2D div(Vector2D lhs, double val) {
        Vector2D result = new Vector2D(lhs);
        result.x /= val;
        result.y /= val;

        return result;
    }

//std::ostream& operator<<(std::ostream& os, const Vector2D& rhs)
    @Override
    public String toString() {
        return " " + this.x + " " + this.y;
    }

//std::ifstream& operator>>(std::ifstream& is, Vector2D& lhs)
    public Vector2D read(InputStream in) {
        Scanner sc = new Scanner(in);

        this.x = sc.nextDouble();
        this.y = sc.nextDouble();
        return this;
    }

//------------------------------------------------------------------------non member functions
    public static Vector2D Vec2DNormalize(Vector2D v) {
        Vector2D vec = new Vector2D(v);

        double vector_length = vec.Length();

        if (vector_length > EpsilonDouble) {
            vec.x /= vector_length;
            vec.y /= vector_length;
        }

        return vec;
    }

    public static double Vec2DDistance(Vector2D v1, Vector2D v2) {

        double ySeparation = v2.y - v1.y;
        double xSeparation = v2.x - v1.x;

        return Math.sqrt(ySeparation * ySeparation + xSeparation * xSeparation);
    }

    public static double Vec2DDistanceSq(Vector2D v1, Vector2D v2) {

        double ySeparation = v2.y - v1.y;
        double xSeparation = v2.x - v1.x;

        return ySeparation * ySeparation + xSeparation * xSeparation;
    }

    public static double Vec2DLength(Vector2D v) {
        return Math.sqrt(v.x * v.x + v.y * v.y);
    }

    public static double Vec2DLengthSq(Vector2D v) {
        return (v.x * v.x + v.y * v.y);
    }

    public static Vector2D POINTStoVector(POINTS p) {
        return new Vector2D(p.x, p.y);
    }

    public static Vector2D POINTtoVector(POINT p) {
        return new Vector2D((double) p.x, (double) p.y);
    }

    public static POINTS VectorToPOINTS(Vector2D v) {
        POINTS p = new POINTS();
        p.x = (short) v.x;
        p.y = (short) v.y;

        return p;
    }

    public static POINT VectorToPOINT(Vector2D v) {
        POINT p = new POINT();
        p.x = (long) v.x;
        p.y = (long) v.y;

        return p;
    }

///////////////////////////////////////////////////////////////////////////////
//treats a window as a toroid
    public static void WrapAround(Vector2D pos, int MaxX, int MaxY) {
        if (pos.x > MaxX) {
            pos.x = 0.0;
        }

        if (pos.x < 0) {
            pos.x = (double) MaxX;
        }

        if (pos.y < 0) {
            pos.y = (double) MaxY;
        }

        if (pos.y > MaxY) {
            pos.y = 0.0;
        }
    }

    /**
     * returns true if the point p is not inside the region defined by top_left
     * and bot_rgt
     * @param p
     * @param top_left
     * @param bot_rgt
     * @return 
     */
    public static boolean NotInsideRegion(Vector2D p,
            Vector2D top_left,
            Vector2D bot_rgt) {
        return (p.x < top_left.x) || (p.x > bot_rgt.x)
                || (p.y < top_left.y) || (p.y > bot_rgt.y);
    }

    public static boolean InsideRegion(Vector2D p,
            Vector2D top_left,
            Vector2D bot_rgt) {
        return !((p.x < top_left.x) || (p.x > bot_rgt.x)
                || (p.y < top_left.y) || (p.y > bot_rgt.y));
    }

    public static boolean InsideRegion(Vector2D p, int left, int top, int right, int bottom) {
        return !((p.x < left) || (p.x > right) || (p.y < top) || (p.y > bottom));
    }

/**
 * @return true if the target position is in the field of view of the entity
 *         positioned at posFirst facing in facingFirst
 */
    public static boolean isSecondInFOVOfFirst(Vector2D posFirst,
            Vector2D facingFirst,
            Vector2D posSecond,
            double fov) {
        Vector2D toTarget = Vec2DNormalize(sub(posSecond, posFirst));
        return facingFirst.Dot(toTarget) >= Math.cos(fov / 2.0);
    }
}