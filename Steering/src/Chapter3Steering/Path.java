/**
 * 
 *  Desc:   class to define, manage, and traverse a path (defined by a series of 2D vectors)
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import java.util.LinkedList;
import common.D2.Vector2D;
import java.util.List;
import java.util.ListIterator;
import static common.misc.utils.*;
import static java.lang.Math.*;
import static common.D2.Transformation.*;
import static common.misc.Cgdi.gdi;

class Path {
    
    private List<Vector2D> m_WayPoints = new LinkedList<Vector2D>();
    //points to the current waypoint
    private ListIterator<Vector2D> curWaypoint;
    private Vector2D cur = null;
    //flag to indicate if the path should be looped
    //(The last waypoint connected to the first)
    boolean m_bLooped;
    
    public Path() {
        m_bLooped = false;
    }

    //constructor for creating a path with initial random waypoints. MinX/Y
    //& MaxX/Y define the bounding box of the path.
    public Path(int NumWaypoints,
            double MinX,
            double MinY,
            double MaxX,
            double MaxY,
            boolean looped) {
        m_bLooped = looped;
        CreateRandomPath(NumWaypoints, MinX, MinY, MaxX, MaxY);
    }

    //returns the current waypoint
    public Vector2D CurrentWaypoint() {
        assert (curWaypoint != null);
        return cur;
    }

    //returns true if the end of the list has been reached
    public boolean Finished() {
        return !(curWaypoint.hasNext());
    }

    //moves the iterator on to the next waypoint in the list
    public void SetNextWaypoint() {
        assert (m_WayPoints.size() > 0);
        
        if (!curWaypoint.hasNext()) {
            if (m_bLooped) {
                curWaypoint = m_WayPoints.listIterator();
            }
        }
        if (curWaypoint.hasNext()) {
            cur = curWaypoint.next();
        }
    }

    //creates a random path which is bound by rectangle described by
    //the min/max values
    public List<Vector2D> CreateRandomPath(int NumWaypoints,
            double MinX,
            double MinY,
            double MaxX,
            double MaxY) {
        m_WayPoints.clear();
        
        double midX = (MaxX + MinX) / 2.0;
        double midY = (MaxY + MinY) / 2.0;
        
        double smaller = min(midX, midY);
        
        double spacing = TwoPi / (double) NumWaypoints;
        
        for (int i = 0; i < NumWaypoints; ++i) {
            double RadialDist = RandInRange(smaller * 0.2f, smaller);
            
            Vector2D temp = new Vector2D(RadialDist, 0.0f);
            
            Vec2DRotateAroundOrigin(temp, i * spacing);
            
            temp.x += midX;
            temp.y += midY;
            
            m_WayPoints.add(temp);
            
        }
        
        curWaypoint = m_WayPoints.listIterator();
        if (curWaypoint.hasNext()) {
            cur = curWaypoint.next();
        }
        
        return m_WayPoints;
    }
    
    public void LoopOn() {
        m_bLooped = true;
    }

    public void LoopOff() {
        m_bLooped = false;
    }

    //adds a waypoint to the end of the path
  /*
    void AddWayPoint(Vector2D new_point) {
    m_WayPoints.add(new_point);
    } */
    //methods for setting the path with either another Path or a list of vectors
    public void Set(List<Vector2D> new_path) {
        m_WayPoints = new_path;
        curWaypoint = m_WayPoints.listIterator();
        cur = curWaypoint.next();
    }

    public void Set(Path path) {
        Set(path.GetPath());
    }
    
    public void Clear() {
        m_WayPoints.clear();
    }
    
    public List<Vector2D> GetPath() {
        return m_WayPoints;
    }

    //renders the path in orange
    public void Render() {
        gdi.OrangePen();
        
        ListIterator<Vector2D> it = m_WayPoints.listIterator();
        
        Vector2D wp = it.next();
        
        while (it.hasNext()) {
            Vector2D n = it.next();
            gdi.Line(wp, n);
            
            wp = n;
        }
        
        if (m_bLooped) {
            gdi.Line(wp, m_WayPoints.get(0));
        }
    }
}
