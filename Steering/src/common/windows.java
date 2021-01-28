/**
 * @author Petr (http://www.sallyx.org/)
 */
package common;

import java.awt.Image;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.geom.Point2D;
import java.net.URL;

/**
 *
 * @author Petr
 */
final public class windows {

    static public int FOREGROUND_BLUE = 0x0001;
    static public int FOREGROUND_GREEN = 0x0002;
    static public int FOREGROUND_RED = 0x0004;
    static public int FOREGROUND_INTENSITY = 0x0008;
    static public int BACKGROUND_RED = 0x0040;

    static public class POINT extends Point2D.Double {

        @Override
        public void setLocation(double x, double y) {
            super.setLocation(Math.round(x), Math.round(y));
        }
    }

    static public class POINTS extends Point {

        public POINTS() {
            this(0, 0);
        }

        public POINTS(int x, int y) {
            super(x, y);
        }

        public POINTS(Point point) {
            super(point);
        }
    }
    final static public long MF_CHECKED = 0x00000008L;
    final static public long MF_UNCHECKED = 0x00000000L;
    final static public long MFS_CHECKED = MF_CHECKED;
    final static public long MFS_UNCHECKED = MF_UNCHECKED;

    static public Image LoadIcon(String file) {
        URL icoURL = windows.class.getResource(file);
        Image img = Toolkit.getDefaultToolkit().createImage(icoURL);
        return img;
    }
}