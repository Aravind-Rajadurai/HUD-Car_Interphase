/**
 *  Desc:   A singleton class to help alleviate the tedium of using the
 *          GDI. Call each method using the #define for gdi->
 *          eg gdi->Line(10, 20, 300, 300)
 *          You must always call gdi->StartDrawing() prior to any 
 *          rendering, and isComplete any rendering with gdi->StopDrawing()
 *
 * @author Petr (http://www.sallyx.org/)
 */
package common.misc;

import common.D2.Vector2D;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.util.List;
import static common.D2.Vector2D.*;

public class Cgdi {

    //------------------------------- define some colors
    final public Color[] colors = {
        new Color(255, 0, 0),
        new Color(0, 0, 255),
        new Color(0, 255, 0),
        new Color(0, 0, 0),
        new Color(255, 200, 200),
        new Color(200, 200, 200), // grey
        new Color(255, 255, 0),
        new Color(255, 170, 0),
        new Color(255, 0, 170),
        new Color(133, 90, 0),
        new Color(255, 255, 255),
        new Color(0, 100, 0), //dark green
        new Color(0, 255, 255), //light blue
        new Color(200, 200, 200), //light grey
        new Color(255, 230, 230) //light pink
    };
    final public int NumColors = colors.length;
    public static final Cgdi gdi = new Cgdi();

    public int NumPenColors() {
        return NumColors;
    }
    private Font m_OldFont;
    //enumerate some colors
    final public static int red = 0;
    final public static int blue = 1;
    final public static int green = 2;
    final public static int black = 3;
    final public static int pink = 4;
    final public static int grey = 5;
    final public static int yellow = 6;
    final public static int orange = 7;
    final public static int purple = 8;
    final public static int brown = 9;
    final public static int white = 10;
    final public static int dark_green = 11;
    final public static int light_blue = 12;
    final public static int light_grey = 13;
    final public static int light_pink = 14;
    final public static int hollow = 15;
    private Color m_OldPen;
    //all the pens
    private Color m_BlackPen;
    private Color m_WhitePen;
    private Color m_RedPen;
    private Color m_GreenPen;
    private Color m_BluePen;
    private Color m_GreyPen;
    private Color m_PinkPen;
    private Color m_OrangePen;
    private Color m_YellowPen;
    private Color m_PurplePen;
    private Color m_BrownPen;
    private Color m_DarkGreenPen;
    private Color m_LightBluePen;
    private Color m_LightGreyPen;
    private Color m_LightPinkPen;
    private Color m_ThickBlackPen;
    private Color m_ThickWhitePen;
    private Color m_ThickRedPen;
    private Color m_ThickGreenPen;
    private Color m_ThickBluePen;

    public void fillRect(Color c, int left, int top, int width, int height) {
        Color old = m_hdc.getColor();
        m_hdc.setColor(c);
        m_hdc.fillRect(left, top, width, height);
        m_hdc.setColor(old);
    }

    public int fontHeight() {
        if (m_hdc == null) {
            return 0;
        }
        return m_hdc.getFontMetrics().getHeight();
    }

    public class Brush extends Color {

        public Brush(int rgb) {
            super(rgb);
        }

        public Brush(Color c) {
            super(c.getRGB());
        }

        public Brush(int r, int g, int b) {
            super(r, g, b);
        }
    }
    private Brush m_OldBrush;
    //all the brushes
    private Brush m_RedBrush;
    private Brush m_GreenBrush;
    private Brush m_BlueBrush;
    private Brush m_GreyBrush;
    private Brush m_BrownBrush;
    private Brush m_YellowBrush;
    private Brush m_OrangeBrush;
    private Brush m_LightBlueBrush;
    private Brush m_DarkGreenBrush;
    private Graphics2D m_hdc;

//constructor is private
    private Cgdi() {
        m_BlackPen = colors[black];
        m_WhitePen = colors[white];
        m_RedPen = colors[red];
        m_GreenPen = colors[green];
        m_BluePen = colors[blue];
        m_GreyPen = colors[grey];
        m_PinkPen = colors[pink];
        m_YellowPen = colors[yellow];
        m_OrangePen = colors[orange];
        m_PurplePen = colors[purple];
        m_BrownPen = colors[brown];

        m_DarkGreenPen = colors[dark_green];

        m_LightBluePen = colors[light_blue];
        m_LightGreyPen = colors[light_grey];
        m_LightPinkPen = colors[light_pink];

        m_ThickBlackPen = colors[black];
        m_ThickWhitePen = colors[white];
        m_ThickRedPen = colors[red];
        m_ThickGreenPen = colors[green];
        m_ThickBluePen = colors[blue];

        m_GreenBrush = new Brush(colors[green]);
        m_RedBrush = new Brush(colors[red]);
        m_BlueBrush = new Brush(colors[blue]);
        m_GreyBrush = new Brush(colors[grey]);
        m_BrownBrush = new Brush(colors[brown]);
        m_YellowBrush = new Brush(colors[yellow]);
        m_LightBlueBrush = new Brush(0, 255, 255);
        m_DarkGreenBrush = new Brush(colors[dark_green]);
        m_OrangeBrush = new Brush(colors[orange]);

        m_hdc = null;
    }

    //copy ctor and assignment should be private
    private Cgdi(Cgdi gdi) {
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException("Cloning not allowed");
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }

    public static Cgdi Instance() {
        return gdi;
    }

    public void BlackPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_BlackPen);
        }
    }

    public void WhitePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_WhitePen);
        }
    }

    public void RedPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_RedPen);
        }
    }

    public void GreenPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_GreenPen);
        }
    }

    public void BluePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_BluePen);
        }
    }

    public void GreyPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_GreyPen);
        }
    }

    public void PinkPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_PinkPen);
        }
    }

    public void YellowPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_YellowPen);
        }
    }

    public void OrangePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_OrangePen);
        }
    }

    public void PurplePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_PurplePen);
        }
    }

    public void BrownPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_BrownPen);
        }
    }

    public void DarkGreenPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_DarkGreenPen);
        }
    }

    public void LightBluePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_LightBluePen);
        }
    }

    public void LightGreyPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_LightGreyPen);
        }
    }

    public void LightPinkPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_LightPinkPen);
        }
    }

    public void ThickBlackPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_ThickBlackPen);
        }
    }

    public void ThickWhitePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_ThickWhitePen);
        }
    }

    public void ThickRedPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_ThickRedPen);
        }
    }

    public void ThickGreenPen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_ThickGreenPen);
        }
    }

    public void ThickBluePen() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_ThickBluePen);
        }
    }

    public void BlackBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, new Brush(Color.BLACK));
        }
    }

    public void WhiteBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, new Brush(Color.WHITE));
        }
    }

    public void HollowBrush() {
        if (m_hdc != null) {
            BrushColor = null;
        }
    }

    public void GreenBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_GreenBrush);
        }
    }

    public void RedBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_RedBrush);
        }
    }

    public void BlueBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_BlueBrush);
        }
    }

    public void GreyBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_GreyBrush);
        }
    }

    public void BrownBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_BrownBrush);
        }
    }

    public void YellowBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_YellowBrush);
        }
    }

    public void LightBlueBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_LightBlueBrush);
        }
    }

    public void DarkGreenBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_DarkGreenBrush);
        }
    }

    public void OrangeBrush() {
        if (m_hdc != null) {
            SelectObject(m_hdc, m_OrangeBrush);
        }
    }
    private Color PenColor = Color.BLACK;
    private Color BrushColor = null;

    private void SelectObject(Graphics2D m_hdc, Color color) {
        if (color instanceof Brush) {
            BrushColor = color;
        } else {
            PenColor = color;
        }
    }

    //ALWAYS call this before drawing
    public void StartDrawing(Graphics2D hdc) {
        assert (m_hdc == null);

        m_hdc = hdc;

        //get the current pen
        m_OldPen = hdc.getColor();
        m_OldBrush = new Brush(hdc.getBackground());
        m_OldFont = hdc.getFont();
        hdc.setFont(new Font(m_OldFont.getFontName(), Font.BOLD, 12));
        m_hdc.setRenderingHint(RenderingHints.KEY_ANTIALIASING, 
                RenderingHints.VALUE_ANTIALIAS_ON);
    }

    //ALWAYS call this after drawing
    public void StopDrawing(Graphics2D hdc) {
        assert (hdc != null);

        hdc.setColor(m_OldPen);
        hdc.setBackground(m_OldBrush);
        hdc.setFont(m_OldFont);
        m_hdc = null;
    }
    
   
    //---------------------------Text
    private boolean opaque = false;
    private Color textColor = Color.BLACK;
    private Color bg = Color.WHITE;

    public void TextAtPos(int x, int y, String s) {
        Color back = m_hdc.getColor();
        y += fontHeight() - 2;
        if (opaque) {
            FontMetrics fm = m_hdc.getFontMetrics();
            int lineHeight = fm.getHeight();
            m_hdc.setColor(bg);
            m_hdc.fillRect(x, y - fm.getAscent() + fm.getDescent(),
                    fm.stringWidth(s), fm.getAscent());
        }
        m_hdc.setColor(textColor);
        m_hdc.drawString(s, x, y);
        m_hdc.setColor(back);
    }

    public void TextAtPos(double x, double y, String s) {
        TextAtPos((int) x, (int) y, s);
    }

    public void TextAtPos(Vector2D pos, String s) {
        TextAtPos((int) pos.x, (int) pos.y, s);
    }

    public void TransparentText() {
        opaque = false;
    }

    public void OpaqueText() {
        opaque = true;
    }

    public void TextColor(int color) {
        assert (color < NumColors);
        textColor = colors[color];
    }

    public void TextColor(int r, int g, int b) {
        textColor = new Color(r, g, b);
    }

    //----------------------------pixels
    public void DrawDot(Vector2D pos, Color color) {
        DrawDot((int) pos.x, (int) pos.y, color);
    }

    public void DrawDot(int x, int y, Color color) {
        m_hdc.setColor(BrushColor);
        m_hdc.fillRect(x, y, 0, 0);
    }

    //-------------------------Line Drawing
    public void Line(Vector2D from, Vector2D to) {
        Line(from.x, from.y, to.x, to.y);
    }

    public void Line(int a, int b, int x, int y) {
        m_hdc.setColor(PenColor);
        m_hdc.drawLine(a, b, x, y);
    }

    public void Line(double a, double b, double x, double y) {
        Line((int) a, (int) b, (int) x, (int) y);
    }

    public void PolyLine(List<Vector2D> points) {
        //make sure we have at least 2 points
        if (points.size() < 2) {
            return;
        }
        Polygon p = new Polygon();

        for (Vector2D v : points) {
            p.addPoint((int) v.x, (int) v.y);
        }
        m_hdc.setColor(PenColor);
        m_hdc.drawPolygon(p);
    }

    public void LineWithArrow(Vector2D from, Vector2D to, double size) {
        Vector2D norm = Vec2DNormalize(sub(to, from));

        //calculate where the arrow is attached
        Vector2D CrossingPoint = sub(to, mul(norm, size));

        //calculate the two extra points required to make the arrowhead
        Vector2D ArrowPoint1 = add(CrossingPoint, (mul(norm.Perp(), 0.4f * size)));
        Vector2D ArrowPoint2 = add(CrossingPoint, (mul(norm.Perp(), 0.4f * size)));

        //draw the line
        m_hdc.setColor(PenColor);
        m_hdc.drawLine((int) from.x, (int) from.y, (int) CrossingPoint.x, (int) CrossingPoint.y);

        //draw the arrowhead (filled with the currently selected brush)
        Polygon p = new Polygon();

        p.addPoint((int) ArrowPoint1.x, (int) ArrowPoint1.y);
        p.addPoint((int) ArrowPoint2.x, (int) ArrowPoint2.y);
        p.addPoint((int) to.x, (int) to.y);

        //SetPolyFillMode(m_hdc, WINDING);
        if(BrushColor != null) {
            m_hdc.setColor(BrushColor);
            m_hdc.fillPolygon(p);
        }
    }

    public void Cross(Vector2D pos, int diameter) {
        Line((int) pos.x - diameter, (int) pos.y - diameter, (int) pos.x + diameter, (int) pos.y + diameter);
        Line((int) pos.x - diameter, (int) pos.y + diameter, (int) pos.x + diameter, (int) pos.y - diameter);
    }

    //---------------------Geometry drawing methods
    public void Rect(int left, int top, int right, int bot) {
        if (left > right) {
            int tmp = right;
            right = left;
            left = tmp;
        }
        m_hdc.setColor(PenColor);
        m_hdc.drawRect(left, top, right - left, bot - top);
        if (BrushColor != null) {
            m_hdc.setColor(BrushColor);
            m_hdc.fillRect(left, top, right - left, bot - top);
        }

    }

    public void Rect(double left, double top, double right, double bot) {
        Rect((int) left, (int) top, (int) right, (int) bot);
    }

    public void ClosedShape(List<Vector2D> points) {
        Polygon pol = new Polygon();

        for (Vector2D p : points) {
            pol.addPoint((int) p.x, (int) p.y);
        }
        m_hdc.setColor(PenColor);
        m_hdc.drawPolygon(pol);
        if(BrushColor != null) {
            //m_hdc.setColor(BrushColor);
            //m_hdc.fillPolygon(pol);
        }
    }

    public void Circle(Vector2D pos, double radius) {
        Circle(pos.x, pos.y, radius);
    }

    public void Circle(double x, double y, double radius) {
        m_hdc.setColor(PenColor);
        m_hdc.drawOval(
                (int) (x - radius),
                (int) (y - radius),
                (int) (radius * 2),
                (int) (radius * 2));
        if (BrushColor != null) {
            m_hdc.setColor(BrushColor);
            m_hdc.fillOval(
                    (int) (x - radius+1),
                    (int) (y - radius+1),
                    (int) (radius * 2-1),
                    (int) (radius * 2-1));
        }
    }

    public void Circle(int x, int y, double radius) {
        Circle((double) x, (double) y, radius);
    }

    public void SetPenColor(int color) {
        assert (color < NumColors);

        switch (color) {
            case black:
                BlackPen();
                return;

            case white:
                WhitePen();
                return;
            case red:
                RedPen();
                return;
            case green:
                GreenPen();
                return;
            case blue:
                BluePen();
                return;
            case pink:
                PinkPen();
                return;
            case grey:
                GreyPen();
                return;
            case yellow:
                YellowPen();
                return;
            case orange:
                OrangePen();
                return;
            case purple:
                PurplePen();
                return;
            case brown:
                BrownPen();
                return;
            case light_blue:
                LightBluePen();
                return;
            case light_grey:
                LightGreyPen();
                return;
            case light_pink:
                LightPinkPen();
                return;
        }//end switch
    }
}