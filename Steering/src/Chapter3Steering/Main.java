/**
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import javax.swing.JMenuBar;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.Lock;
import common.misc.CppToJava;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.Graphics2D;
import common.Time.PrecisionTimer;
import java.awt.Color;
import java.awt.Cursor;
import static common.misc.Cgdi.gdi;
import java.awt.Graphics;
import javax.swing.JPanel;
import java.awt.image.BufferedImage;
import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Point;
import javax.swing.JFrame;
import static Chapter3Steering.constants.*;
import static common.misc.WindowUtils.*;
import static Chapter3Steering.resource.*;
import static common.windows.*;
import static Chapter3Steering.resource.IDR_MENU1;
import static common.misc.WindowUtils.ChangeMenuState;

public class Main {
//--------------------------------- Globals ------------------------------
//
//------------------------------------------------------------------------

    static String g_szApplicationName = "Steering Behaviors - Another Big Shoal";
    // static String g_szWindowClassName = "MyWindowClass";
    static GameWorld g_GameWorld;
    static Lock GameWorldLock = new ReentrantLock(); // bacause of restart (g_GameWorld could be null for a while)

    /**
     *	The entry point of the windows program
     */
    public static void main(String[] args) {
        final Window window = new Window(g_szApplicationName);
        window.setIconImage(LoadIcon("/Chapter3Steering/example.png"));
        window.setCursor(Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR));
        final BufferedImage buffer = new BufferedImage(constWindowWidth, constWindowHeight, BufferedImage.TYPE_INT_RGB);
        final Graphics2D hdcBackBuffer = buffer.createGraphics();
        //these hold the dimensions of the client window area
        final int cxClient = buffer.getWidth();
        final int cyClient = buffer.getHeight();
        //seed random number generator
        common.misc.utils.setSeed(0);

        window.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        Point center = GraphicsEnvironment.getLocalGraphicsEnvironment().getCenterPoint();
        //Dimension screen = Toolkit.getDefaultToolkit().getScreenSize();

        window.setResizable(false);

        int y = center.y - window.getHeight() / 2;
        window.setLocation(center.x - window.getWidth() / 2, y >= 0 ? y : 0);
        g_GameWorld = new GameWorld(cxClient, cyClient);

        window.setJMenuBar(Script1.createMenu(IDR_MENU1, g_GameWorld));

        final JPanel panel = new JPanel() {

            @Override
            public void paint(Graphics g) {
                super.paint(g);
                hdcBackBuffer.setPaint(Color.RED);
                gdi.StartDrawing(hdcBackBuffer);
                //fill our backbuffer with white
                gdi.fillRect(Color.WHITE, 0, 0, constWindowWidth, constWindowHeight);
                GameWorldLock.lock();
                g_GameWorld.Render();
                GameWorldLock.unlock();
                gdi.StopDrawing(hdcBackBuffer);


                g.drawImage(buffer, 0, 0, null);
            }
        };
        panel.setSize(constWindowWidth, constWindowHeight);
        panel.setPreferredSize(new Dimension(constWindowWidth, constWindowHeight));
        window.add(panel);
        window.pack();

        ChangeMenuState(window.getMenu(), IDR_PRIORITIZED, MFS_CHECKED);
        ChangeMenuState(window.getMenu(), ID_VIEW_FPS, MFS_CHECKED);

        window.addMouseListener(new MouseAdapter() {

            @Override
            public void mouseClicked(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1) {
                    GameWorldLock.lock();
                    g_GameWorld.SetCrosshair(new POINTS(e.getPoint()));
                    GameWorldLock.unlock();
                }
            }
        });

        window.addKeyListener(new KeyAdapter() {

            @Override
            public void keyReleased(KeyEvent e) {
                CppToJava.keyCache.released(e);
                switch (e.getKeyChar()) {
                    case KeyEvent.VK_ESCAPE: {
                        System.exit(0);
                    }
                    break;
                    case 'r':
                    case 'R': {
                        GameWorldLock.lock();
                        g_GameWorld = null;
                        g_GameWorld = new GameWorld(cxClient, cyClient);
                        JMenuBar bar = Script1.createMenu(IDR_MENU1, g_GameWorld);
                        window.setJMenuBar(bar);
                        bar.revalidate();
                        GameWorldLock.unlock();
                    }
                    break;
                }//end switch

                //handle any others
                GameWorldLock.lock();
                g_GameWorld.HandleKeyPresses(e);
                GameWorldLock.unlock();
            }

            @Override
            public void keyPressed(KeyEvent e) {
                CppToJava.keyCache.pressed(e);
            }
        });

        //make the window visible
        window.setVisible(true);

        //create a timer
        final PrecisionTimer timer = new PrecisionTimer();

        timer.SmoothUpdatesOn();

        //start the timer
        timer.Start();

        while (true) {
            //update
            GameWorldLock.lock();
            g_GameWorld.Update(timer.TimeElapsed());
            GameWorldLock.unlock();
            //render
            //panel.revalidate();
            panel.repaint();

            try {
                //System.out.println(timer.TimeElapsed());
                Thread.sleep(2);
            } catch (InterruptedException ex) {
            }
        }//end while
    }
}