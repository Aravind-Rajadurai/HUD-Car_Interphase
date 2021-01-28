/**
 * Base class for a game object
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import java.io.InputStream;
import common.D2.Vector2D;
import common.Messaging.Telegram;
import static common.misc.utils.*;

public abstract class BaseGameEntity {

    public static int default_entity_type = -1;
    //each entity has a unique ID
    private int m_ID = 0;
    //every entity has a type associated with it (health, troll, ammo etc)
    int m_EntityType;
    //this is a generic flag. 
    boolean m_bTag;
    //this is the next valid ID. Each time a BaseGameEntity is instantiated
    //this value is updated
    static private int NextID = 0;

    //used by the constructor to give each entity a unique ID
    private int NextValidID() {
        return NextID++;
    }

    /**
     * Use with recreating GameWord
     */
    public static void resetValidID() {
        NextID = 0;
    }
    //its location in the environment
    protected Vector2D m_vPos;
    protected Vector2D m_vScale;
    //the length of this object's bounding radius
    protected double m_dBoundingRadius;

    protected BaseGameEntity() {
        m_ID = NextValidID();
        m_dBoundingRadius = 0.0;
        m_vPos = new Vector2D();
        m_vScale = new Vector2D(1.0, 1.0);
        m_EntityType = default_entity_type;
        m_bTag = false;
    }

    protected BaseGameEntity(int entity_type) {
        m_ID = NextValidID();
        m_dBoundingRadius = 0.0;
        m_vPos = new Vector2D();
        m_vScale = new Vector2D(1.0, 1.0);
        m_EntityType = entity_type;
        m_bTag = false;
    }

    protected BaseGameEntity(int entity_type, Vector2D pos, double r) {
        m_vPos = pos;
        m_dBoundingRadius = r;
        m_ID = NextValidID();
        m_vScale = new Vector2D(1.0, 1.0);
        m_EntityType = entity_type;
        m_bTag = false;

    }

    /**
     * this can be used to create an entity with a 'forced' ID. It can be used
     * when a previously created entity has been removed and deleted from the
     * game for some reason. For example, The Raven map editor uses this ctor 
     * in its undo/redo operations. 
     * USE WITH CAUTION!
     */
    protected BaseGameEntity(int entity_type, int ForcedID) {
        m_ID = ForcedID;
        m_dBoundingRadius = 0.0;
        m_vPos = new Vector2D();
        m_vScale = new Vector2D(1.0, 1.0);
        m_EntityType = entity_type;
        m_bTag = false;
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }

    public void Update(double time_elapsed) {
    }

    public void Render() {
    }

    public boolean HandleMessage(Telegram msg) {
        return false;
    }

    //entities should be able to read/write their data to a stream
    //virtual void Write(std::ostream&  os)const{}
    @Override
    public String toString() {
        return String.format("%f %f", this.m_vPos.x, this.m_vPos.y);
    }

    //virtual void Read (std::ifstream& is){}
    public void read(InputStream in) {
    }

    public Vector2D Pos() {
        return new Vector2D(m_vPos);
    }

    void SetPos(Vector2D new_pos) {
        m_vPos = new Vector2D(new_pos);
    }

    double BRadius() {
        return m_dBoundingRadius;
    }

    void SetBRadius(double r) {
        m_dBoundingRadius = r;
    }

    public int ID() {
        return m_ID;
    }

    boolean IsTagged() {
        return m_bTag;
    }

    void Tag() {
        m_bTag = true;
    }

    void UnTag() {
        m_bTag = false;
    }

    Vector2D Scale() {
        return new Vector2D(m_vScale);
    }

    void SetScale(Vector2D val) {
        m_dBoundingRadius *= MaxOf(val.x, val.y) / MaxOf(m_vScale.x, m_vScale.y);
        m_vScale = new Vector2D(val);
    }

    void SetScale(double val) {
        m_dBoundingRadius *= (val / MaxOf(m_vScale.x, m_vScale.y));
        m_vScale = new Vector2D(val, val);
    }

    int EntityType() {
        return m_EntityType;
    }

    void SetEntityType(int new_type) {
        m_EntityType = new_type;
    }
}