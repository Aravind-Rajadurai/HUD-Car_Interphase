/**
 * 
 *  Desc:   class to create and render 2D walls. Defined as the two 
 *          vectors A - B with a perpendicular normal. 
 *
 * @author Petr (http://www.sallyx.org/)
 */
package common.D2;

import java.util.Scanner;
import java.io.PrintStream;
import java.io.InputStream;
import static common.D2.Vector2D.*;
import static common.misc.Cgdi.gdi;

public class Wall2D {

    protected Vector2D m_vA = new Vector2D(),
            m_vB = new Vector2D(),
            m_vN = new Vector2D();

    protected void CalculateNormal() {
        Vector2D temp = Vec2DNormalize(sub(m_vB, m_vA));

        m_vN.x = -temp.y;
        m_vN.y = temp.x;
    }

    public Wall2D() {
    }

    public Wall2D(Vector2D A, Vector2D B) {
        m_vA = A;
        m_vB = B;
        CalculateNormal();
    }

    public Wall2D(Vector2D A, Vector2D B, Vector2D N) {
        m_vA = A;
        m_vB = B;
        m_vN = N;
    }

    public Wall2D(InputStream in) {
        Read(in);
    }

    public void Render() {
        Render(false);
    }

    public void Render(boolean RenderNormals) {
        gdi.Line(m_vA, m_vB);

        //render the normals if rqd
        if (RenderNormals) {
            int MidX = (int) ((m_vA.x + m_vB.x) / 2);
            int MidY = (int) ((m_vA.y + m_vB.y) / 2);

            gdi.Line(MidX, MidY, (int) (MidX + (m_vN.x * 5)), (int) (MidY + (m_vN.y * 5)));
        }
    }

    public Vector2D From() {
        return m_vA;
    }

    public void SetFrom(Vector2D v) {
        m_vA = v;
        CalculateNormal();
    }

    public Vector2D To() {
        return m_vB;
    }

    public void SetTo(Vector2D v) {
        m_vB = v;
        CalculateNormal();
    }

    public Vector2D Normal() {
        return m_vN;
    }

    public void SetNormal(Vector2D n) {
        m_vN = n;
    }

    public Vector2D Center() {
        return div(add(m_vA, m_vB), 2.0);
    }

    public PrintStream Write(PrintStream os) {
        os.println();
        os.print(From() + ",");
        os.print(To() + ",");
        os.print(Normal());
        return os;
    }

    public void Read(InputStream in) {
        double x, y;
        Scanner br = new Scanner(in);
        x = br.nextDouble();
        y = br.nextDouble();

        SetFrom(new Vector2D(x, y));

        x = br.nextDouble();
        y = br.nextDouble();
        SetTo(new Vector2D(x, y));

        x = br.nextDouble();
        y = br.nextDouble();
        SetNormal(new Vector2D(x, y));
    }
}