/**
 * 
 *  Desc:   All the environment data and methods for the Steering
 *          Behavior projects. This class is the root of the project's
 *          update and render calls (excluding main of course)
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

//------------------------------------------------------------------------
import common.D2.InvertedAABBox2D;
import java.awt.event.KeyEvent;
import common.misc.Cgdi;
import java.util.ArrayList;
import common.D2.Vector2D;
import static common.D2.Vector2D.*;
import java.util.List;
import java.util.ListIterator;
import common.D2.Wall2D;
import common.misc.CellSpacePartition;
import static Chapter3Steering.EntityFunctionTemplates.*;
import static Chapter3Steering.ParamLoader.Prm;
import static common.misc.utils.*;
import common.misc.Smoother;
import static common.windows.*;
import static common.D2.geometry.*;
import static common.misc.Cgdi.*;
import static Chapter3Steering.resource.*;
import static common.misc.WindowUtils.*;
import static common.misc.Stream_Utility_function.ttos;
import static Chapter3Steering.SteeringBehavior.summing_method;

public class GameWorld {
    //a container of all the moving entities

    private List<Vehicle> m_Vehicles = new ArrayList<Vehicle>(Prm.NumAgents);
    //any obstacles
    private List<BaseGameEntity> m_Obstacles = new ArrayList<BaseGameEntity>(Prm.NumObstacles);
    //container containing any walls in the environment
    private List<Wall2D> m_Walls = new ArrayList<Wall2D>();
    private CellSpacePartition<Vehicle> m_pCellSpace;
    //any path we may create for the vehicles to follow
    private Path m_pPath;
    //set true to pause the motion
    private boolean m_bPaused;
    //local copy of client window dimensions
    private int m_cxClient,
            m_cyClient;
    //the position of the crosshair
    private Vector2D m_vCrosshair;
    //keeps track of the average FPS
    private double m_dAvFrameTime;
    //flags to turn aids and obstacles etc on/off
    private boolean m_bShowWalls;
    private boolean m_bShowObstacles;
    private boolean m_bShowPath;
    private boolean m_bShowDetectionBox;
    private boolean m_bShowWanderCircle;
    private boolean m_bShowFeelers;
    private boolean m_bShowSteeringForce;
    private boolean m_bShowFPS;
    private boolean m_bRenderNeighbors;
    private boolean m_bViewKeys;
    private boolean m_bShowCellSpaceInfo;

    //private void CreateObstacles();
    //private void CreateWalls();
    //public GameWorld(int cx, int cy);
    //public ~GameWorld();
    // public void  Update(double time_elapsed);
    // public void  Render();
    public void NonPenetrationContraint(Vehicle v) {
        EnforceNonPenetrationConstraint(v, m_Vehicles);
    }

    public void TagVehiclesWithinViewRange(BaseGameEntity pVehicle, double range) {
        TagNeighbors(pVehicle, m_Vehicles, range);
    }

    public void TagObstaclesWithinViewRange(BaseGameEntity pVehicle, double range) {
        TagNeighbors(pVehicle, m_Obstacles, range);
    }

    public List<Wall2D> Walls() {
        return m_Walls;
    }

    public CellSpacePartition<Vehicle> CellSpace() {
        return m_pCellSpace;
    }

    public List<BaseGameEntity> Obstacles() {
        return m_Obstacles;
    }

    public List<Vehicle> Agents() {
        return m_Vehicles;
    }

    //handle WM_COMMAND messages
    //public void  HandleKeyPresses(WPARAM wParam);
    //public void  HandleMenuItems(WPARAM wParam, HWND hwnd);
    public void TogglePause() {
        m_bPaused = !m_bPaused;
    }

    public boolean Paused() {
        return m_bPaused;
    }

    public Vector2D Crosshair() {
        return m_vCrosshair;
    }

    public void SetCrosshair(Vector2D v) {
        m_vCrosshair = v;
    }

    public int cxClient() {
        return m_cxClient;
    }

    public int cyClient() {
        return m_cyClient;
    }

    public boolean RenderWalls() {
        return m_bShowWalls;
    }

    public boolean RenderObstacles() {
        return m_bShowObstacles;
    }

    public boolean RenderPath() {
        return m_bShowPath;
    }

    public boolean RenderDetectionBox() {
        return m_bShowDetectionBox;
    }

    public boolean RenderWanderCircle() {
        return m_bShowWanderCircle;
    }

    public boolean RenderFeelers() {
        return m_bShowFeelers;
    }

    public boolean RenderSteeringForce() {
        return m_bShowSteeringForce;
    }

    public boolean RenderFPS() {
        return m_bShowFPS;
    }

    public void ToggleShowFPS() {
        m_bShowFPS = !m_bShowFPS;
    }

    public void ToggleRenderNeighbors() {
        m_bRenderNeighbors = !m_bRenderNeighbors;
    }

    public boolean RenderNeighbors() {
        return m_bRenderNeighbors;
    }

    public void ToggleViewKeys() {
        m_bViewKeys = !m_bViewKeys;
    }

    public boolean ViewKeys() {
        return m_bViewKeys;
    }

//------------------------------- ctor -----------------------------------
//------------------------------------------------------------------------
    public GameWorld(int cx, int cy) {

        m_cxClient = cx;
        m_cyClient = cy;
        m_bPaused = false;
        m_vCrosshair = new Vector2D(cxClient() / 2.0, cxClient() / 2.0);
        m_bShowWalls = false;
        m_bShowObstacles = false;
        m_bShowPath = false;
        m_bShowWanderCircle = false;
        m_bShowSteeringForce = false;
        m_bShowFeelers = false;
        m_bShowDetectionBox = false;
        m_bShowFPS = true;
        m_dAvFrameTime = 0;
        m_pPath = null;
        m_bRenderNeighbors = false;
        m_bViewKeys = false;
        m_bShowCellSpaceInfo = false;

        //setup the spatial subdivision class
        m_pCellSpace = new CellSpacePartition<Vehicle>((double) cx, (double) cy,
                Prm.NumCellsX, Prm.NumCellsY, Prm.NumAgents);

        double border = 30;
        m_pPath = new Path(5, border, border, cx - border, cy - border, true);

        BaseGameEntity.resetValidID();
        for (int a = 0; a < Prm.NumAgents; ++a) {
            //determine a random starting position
            Vector2D SpawnPos = new Vector2D(cx / 2.0 + RandomClamped() * cx / 2.0,
                    cy / 2.0 + RandomClamped() * cy / 2.0);


            Vehicle pVehicle = new Vehicle(this,
                    SpawnPos, //initial position
                    RandFloat() * TwoPi, //start rotation
                    new Vector2D(0, 0), //velocity
                    Prm.VehicleMass, //mass
                    Prm.MaxSteeringForce, //max force
                    Prm.MaxSpeed, //max velocity
                    Prm.MaxTurnRatePerSecond, //max turn rate
                    Prm.VehicleScale);        //scale

            pVehicle.Steering().FlockingOn();

            m_Vehicles.add(pVehicle);

            //add it to the cell subdivision
            m_pCellSpace.AddEntity(pVehicle);
        }

        final boolean SHOAL = true;
        if (SHOAL) {
            m_Vehicles.get(Prm.NumAgents - 1).Steering().FlockingOff();
            m_Vehicles.get(Prm.NumAgents - 1).SetScale(new Vector2D(10, 10));
            m_Vehicles.get(Prm.NumAgents - 1).Steering().WanderOn();
            m_Vehicles.get(Prm.NumAgents - 1).SetMaxSpeed(70);

            for (int i = 0; i < Prm.NumAgents - 1; ++i) {
                m_Vehicles.get(i).Steering().EvadeOn(m_Vehicles.get(Prm.NumAgents - 1));
            }
        }
        //create any obstacles or walls
        //CreateObstacles();
        //CreateWalls();
    }

//-------------------------------- dtor ----------------------------------
//------------------------------------------------------------------------
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        m_Vehicles.clear();
        m_Obstacles.clear();

        m_pCellSpace = null;
        m_pPath = null;
    }
    final static int SampleRate = 10;
    private static Smoother<Double> FrameRateSmoother = new Smoother<Double>(SampleRate, 0.0);

    /**
     * create a smoother to smooth the framerate
     */
    synchronized public void Update(double time_elapsed) {
        if (m_bPaused) {
            return;
        }

        m_dAvFrameTime = FrameRateSmoother.Update(time_elapsed);

        //update the vehicles
        for (int a = 0; a < m_Vehicles.size(); ++a) {
            m_Vehicles.get(a).Update(time_elapsed);
        }
    }

    /**
     * creates some walls that form an enclosure for the steering agents.
     * used to demonstrate several of the steering behaviors
     */
    private void CreateWalls() {
        //create the walls  
        double bordersize = 20.0;
        double CornerSize = 0.2;
        double vDist = m_cyClient - 2 * bordersize;
        double hDist = m_cxClient - 2 * bordersize;

        Vector2D walls[] = {new Vector2D(hDist * CornerSize + bordersize, bordersize),
            new Vector2D(m_cxClient - bordersize - hDist * CornerSize, bordersize),
            new Vector2D(m_cxClient - bordersize, bordersize + vDist * CornerSize),
            new Vector2D(m_cxClient - bordersize, m_cyClient - bordersize - vDist * CornerSize),
            new Vector2D(m_cxClient - bordersize - hDist * CornerSize, m_cyClient - bordersize),
            new Vector2D(hDist * CornerSize + bordersize, m_cyClient - bordersize),
            new Vector2D(bordersize, m_cyClient - bordersize - vDist * CornerSize),
            new Vector2D(bordersize, bordersize + vDist * CornerSize)};

        final int NumWallVerts = walls.length;

        for (int w = 0; w < NumWallVerts - 1; ++w) {
            m_Walls.add(new Wall2D(walls[w], walls[w + 1]));
        }

        m_Walls.add(new Wall2D(walls[NumWallVerts - 1], walls[0]));
    }

    /**
     * Sets up the vector of obstacles with random positions and sizes. Makes
     *  sure the obstacles do not overlap
     */
    private void CreateObstacles() {
        //create a number of randomly sized tiddlywinks
        for (int o = 0; o < Prm.NumObstacles; ++o) {
            boolean bOverlapped = true;

            //keep creating tiddlywinks until we find one that doesn't overlap
            //any others.Sometimes this can get into an endless loop because the
            //obstacle has nowhere to fit. We test for this case and exit accordingly

            int NumTrys = 0;
            int NumAllowableTrys = 2000;

            while (bOverlapped) {
                NumTrys++;

                if (NumTrys > NumAllowableTrys) {
                    return;
                }
                int radius = RandInt((int) Prm.MinObstacleRadius, (int) Prm.MaxObstacleRadius);
                final int border = 10;
                final int MinGapBetweenObstacles = 20;

                Obstacle ob = new Obstacle(RandInt(radius + border, m_cxClient - radius - border),
                        RandInt(radius + border, m_cyClient - radius - 30 - border),
                        radius);

                if (!Overlapped((BaseGameEntity) ob, m_Obstacles, MinGapBetweenObstacles)) {
                    //its not overlapped so we can add it
                    m_Obstacles.add(ob);
                    bOverlapped = false;
                } else {
                    ob = null;
                }
            }
        }
    }

    /**
     * The user can set the position of the crosshair by right clicking the
     *  mouse. This method makes sure the click is not inside any enabled
     *  Obstacles and sets the position appropriately
     */
    synchronized public void SetCrosshair(POINTS p) {
        Vector2D ProposedPosition = new Vector2D((double) p.x, (double) p.y);

        //make sure it's not inside an obstacle
        ListIterator<BaseGameEntity> it = m_Obstacles.listIterator();
        while (it.hasNext()) {
            BaseGameEntity curOb = it.next();
            if (PointInCircle(curOb.Pos(), curOb.BRadius(), ProposedPosition)) {
                return;
            }

        }
        m_vCrosshair.x = (double) p.x;
        m_vCrosshair.y = (double) p.y;
    }

//------------------------- HandleKeyPresses -----------------------------
    synchronized public void HandleKeyPresses(KeyEvent wParam) {
        switch (wParam.getKeyChar()) {
            case 'u':
            case 'U': {
                m_pPath = null;
                double border = 60;
                m_pPath = new Path(RandInt(3, 7), border, border, cxClient() - border, cyClient() - border, true);
                m_bShowPath = true;
                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).Steering().SetPath(m_pPath.GetPath());
                }
            }
            break;
            case 'p':
            case 'P':

                TogglePause();
                break;

            case 'o':
            case 'O':

                ToggleRenderNeighbors();
                break;

            case 'i':
            case 'I': {
                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).ToggleSmoothing();
                }

            }

            break;
            case 'y':
            case 'Y':

                m_bShowObstacles = !m_bShowObstacles;

                if (!m_bShowObstacles) {
                    m_Obstacles.clear();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_Vehicles.get(i).Steering().ObstacleAvoidanceOff();
                    }
                } else {
                    CreateObstacles();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_Vehicles.get(i).Steering().ObstacleAvoidanceOn();
                    }
                }
                break;

        }//end switch
    }

//-------------------------- HandleMenuItems -----------------------------
    synchronized public void HandleMenuItems(int wParam, Script1.MyMenuBar hwnd) {
        switch (wParam) {
            case ID_OB_OBSTACLES:

                m_bShowObstacles = !m_bShowObstacles;

                if (!m_bShowObstacles) {
                    m_Obstacles.clear();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_Vehicles.get(i).Steering().ObstacleAvoidanceOff();
                    }

                    //uncheck the menu
                    ChangeMenuState(hwnd, ID_OB_OBSTACLES, MFS_UNCHECKED);
                } else {
                    CreateObstacles();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_Vehicles.get(i).Steering().ObstacleAvoidanceOn();
                    }

                    //check the menu
                    ChangeMenuState(hwnd, ID_OB_OBSTACLES, MFS_CHECKED);
                }

                break;

            case ID_OB_WALLS:

                m_bShowWalls = !m_bShowWalls;

                if (m_bShowWalls) {
                    CreateWalls();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_Vehicles.get(i).Steering().WallAvoidanceOn();
                    }

                    //check the menu
                    ChangeMenuState(hwnd, ID_OB_WALLS, MFS_CHECKED);
                } else {
                    m_Walls.clear();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_Vehicles.get(i).Steering().WallAvoidanceOff();
                    }

                    //uncheck the menu
                    ChangeMenuState(hwnd, ID_OB_WALLS, MFS_UNCHECKED);
                }

                break;


            case IDR_PARTITIONING: {
                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).Steering().ToggleSpacePartitioningOnOff();
                }

                //if toggled on, empty the cell space and then re-add all the 
                //vehicles
                if (m_Vehicles.get(0).Steering().isSpacePartitioningOn()) {
                    m_pCellSpace.EmptyCells();

                    for (int i = 0; i < m_Vehicles.size(); ++i) {
                        m_pCellSpace.AddEntity(m_Vehicles.get(i));
                    }

                    ChangeMenuState(hwnd, IDR_PARTITIONING, MFS_CHECKED);
                } else {
                    ChangeMenuState(hwnd, IDR_PARTITIONING, MFS_UNCHECKED);
                    ChangeMenuState(hwnd, IDM_PARTITION_VIEW_NEIGHBORS, MFS_UNCHECKED);
                    m_bShowCellSpaceInfo = false;
                }
            }

            break;

            case IDM_PARTITION_VIEW_NEIGHBORS: {
                m_bShowCellSpaceInfo = !m_bShowCellSpaceInfo;

                if (m_bShowCellSpaceInfo) {
                    ChangeMenuState(hwnd, IDM_PARTITION_VIEW_NEIGHBORS, MFS_CHECKED);

                    if (!m_Vehicles.get(0).Steering().isSpacePartitioningOn()) {
                        SendChangeMenuMessage(hwnd, IDR_PARTITIONING);
                    }
                } else {
                    ChangeMenuState(hwnd, IDM_PARTITION_VIEW_NEIGHBORS, MFS_UNCHECKED);
                }
            }
            break;


            case IDR_WEIGHTED_SUM: {
                ChangeMenuState(hwnd, IDR_WEIGHTED_SUM, MFS_CHECKED);
                ChangeMenuState(hwnd, IDR_PRIORITIZED, MFS_UNCHECKED);
                ChangeMenuState(hwnd, IDR_DITHERED, MFS_UNCHECKED);

                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).Steering().SetSummingMethod(SteeringBehavior.summing_method.weighted_average);
                }
            }

            break;

            case IDR_PRIORITIZED: {
                ChangeMenuState(hwnd, IDR_WEIGHTED_SUM, MFS_UNCHECKED);
                ChangeMenuState(hwnd, IDR_PRIORITIZED, MFS_CHECKED);
                ChangeMenuState(hwnd, IDR_DITHERED, MFS_UNCHECKED);

                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).Steering().SetSummingMethod(summing_method.prioritized);
                }
            }

            break;

            case IDR_DITHERED: {
                ChangeMenuState(hwnd, IDR_WEIGHTED_SUM, MFS_UNCHECKED);
                ChangeMenuState(hwnd, IDR_PRIORITIZED, MFS_UNCHECKED);
                ChangeMenuState(hwnd, IDR_DITHERED, MFS_CHECKED);

                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).Steering().SetSummingMethod(summing_method.dithered);
                }
            }

            break;

            case ID_VIEW_KEYS: {
                ToggleViewKeys();
                CheckMenuItemAppropriately(hwnd, ID_VIEW_KEYS, m_bViewKeys);
            }

            break;

            case ID_VIEW_FPS: {
                ToggleShowFPS();

                CheckMenuItemAppropriately(hwnd, ID_VIEW_FPS, RenderFPS());
            }

            break;

            case ID_MENU_SMOOTHING: {
                for (int i = 0; i < m_Vehicles.size(); ++i) {
                    m_Vehicles.get(i).ToggleSmoothing();
                }

                CheckMenuItemAppropriately(hwnd, ID_MENU_SMOOTHING, m_Vehicles.get(0).isSmoothingOn());
            }

            break;

        }//end switch
    }

//------------------------------ Render ----------------------------------
//------------------------------------------------------------------------
    synchronized public void Render() {
        gdi.TransparentText();

        //render any walls
        gdi.BlackPen();
        for (int w = 0; w < m_Walls.size(); ++w) {
            m_Walls.get(w).Render(true);  //true flag shows normals
        }

        //render any obstacles
        gdi.BlackPen();

        for (int ob = 0; ob < m_Obstacles.size(); ++ob) {
            gdi.Circle(m_Obstacles.get(ob).Pos(), m_Obstacles.get(ob).BRadius());
        }

        //render the agents
        for (int a = 0; a < m_Vehicles.size(); ++a) {
            m_Vehicles.get(a).Render(a == 0);

            //render cell partitioning stuff
            if (m_bShowCellSpaceInfo && a == 0) {
                gdi.HollowBrush();
                InvertedAABBox2D box = new InvertedAABBox2D(sub(m_Vehicles.get(a).Pos(), new Vector2D(Prm.ViewDistance, Prm.ViewDistance)),
                        add(m_Vehicles.get(a).Pos(), new Vector2D(Prm.ViewDistance, Prm.ViewDistance)));
                box.Render();

                gdi.RedPen();

                CellSpace().CalculateNeighbors(m_Vehicles.get(a).Pos(), Prm.ViewDistance);
                for (BaseGameEntity pV = CellSpace().begin(); !CellSpace().end(); pV = CellSpace().next()) {
                    gdi.Circle(pV.Pos(), pV.BRadius());
                }

                gdi.GreenPen();
                gdi.Circle(m_Vehicles.get(a).Pos(), Prm.ViewDistance);
            }
        }

//#define CROSSHAIR
        boolean CROSSHAIR = false;
        if (CROSSHAIR) {
            //and finally the crosshair
            gdi.RedPen();
            gdi.Circle(m_vCrosshair, 4);
            gdi.Line(m_vCrosshair.x - 8, m_vCrosshair.y, m_vCrosshair.x + 8, m_vCrosshair.y);
            gdi.Line(m_vCrosshair.x, m_vCrosshair.y - 8, m_vCrosshair.x, m_vCrosshair.y + 8);
            gdi.TextAtPos(5, cyClient() - 20, "Click to move crosshair");
        }


        //gdi->TextAtPos(cxClient() -120, cyClient() - 20, "Press R to reset");

        gdi.TextColor(Cgdi.grey);
        if (RenderPath()) {
            gdi.TextAtPos((int) (cxClient() / 2.0f - 80), cyClient() - 20, "Press 'U' for random path");

            m_pPath.Render();
        }

        if (RenderFPS()) {
            gdi.TextColor(Cgdi.grey);
            gdi.TextAtPos(5, cyClient() - 20, ttos(1.0 / m_dAvFrameTime));
        }

        if (m_bShowCellSpaceInfo) {
            m_pCellSpace.RenderCells();
        }

    }
}