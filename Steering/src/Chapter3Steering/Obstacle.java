/**
 * Desc:   Simple obstacle class
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import java.io.PrintStream;
import java.io.OutputStream;
import java.util.Scanner;
import common.D2.Vector2D;
import java.io.InputStream;
import static common.misc.Cgdi.gdi;

class Obstacle extends BaseGameEntity {

    public Obstacle(double x,
            double y,
            double r) {
        super(0, new Vector2D(x, y), r);
    }

    public Obstacle(Vector2D pos, double radius) {
        super(0, pos, radius);
    }

    public Obstacle(InputStream in) {
        Read(in);
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }
    //this is defined as a pure virtual function in BasegameEntity so
    //it must be implemented

    @Override
    public void Update(double time_elapsed) {
    }

    @Override
    public void Render() {
        gdi.BlackPen();
        gdi.Circle(Pos(), BRadius());
    }

    @Override
    public String toString() {
        return EntityType()+","+Pos()+","+BRadius();
    }
    public void Write(OutputStream os) {
        PrintStream print = new PrintStream(os);
        print.println();
        print.print(toString());
    }

    public void Read(InputStream in) {
        double x, y, r;
        Scanner s = new Scanner(in);
        x = s.nextDouble();
        y = s.nextDouble();
        r = s.nextDouble();

        SetPos(new Vector2D(x, y));
        SetBRadius(r);
    }
}
