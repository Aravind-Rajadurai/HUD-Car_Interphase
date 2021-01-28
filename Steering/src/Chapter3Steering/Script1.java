/**
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

import java.util.HashMap;
import java.util.Map;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JMenuItem;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JCheckBoxMenuItem;
import static Chapter3Steering.resource.*;
import javax.swing.MenuElement;
import static common.windows.*;

public class Script1 {

    public static class MyMenuBar extends JMenuBar {

        final private GameWorld gameWorld;
        final private ActionListener al;
        private Map<Integer, MyCheckBoxMenuItem> items = new HashMap<Integer, MyCheckBoxMenuItem>();

        public MyMenuBar(GameWorld gw) {
            super();
            gameWorld = gw;
            al = new ActionListener() {

                @Override
                public void actionPerformed(ActionEvent e) {
                    MyCheckBoxMenuItem source = (MyCheckBoxMenuItem) e.getSource();
                    gameWorld.HandleMenuItems(source.getID(), MyMenuBar.this);
                }
            };
        }

        @Override
        public JMenu add(JMenu c) {
            for (MenuElement elm : c.getSubElements()) {
                for (MenuElement comp : elm.getSubElements()) {
                    if (comp instanceof MyCheckBoxMenuItem) {
                        MyCheckBoxMenuItem myelm = (MyCheckBoxMenuItem) comp;
                        this.items.put(myelm.getID(), myelm);
                    }
                }
            }
            return super.add(c);
        }

        private ActionListener getActionListener() {
            return al;
        }

        /**
         * Swap menu state and do call actionEvent
         * 
         * @param MenuItem ID of MyCheckBoxMenuItem
         */
        public void changeMenuState(int MenuItem) {
            MyCheckBoxMenuItem item = this.items.get(MenuItem);
            if (item != null) {
                item.doClick();
            }
        }

        /**
         * Set menu state and do not call actionEvent
         * 
         * @param MenuItem ID of MyCheckBoxMenuItem
         * @param state New state (MFS_CHECKED or MFS_UNCHECKED)
         */
        public void setMenuState(int MenuItem, final long state) {
            MyCheckBoxMenuItem item = this.items.get(MenuItem);
            if (item == null) {
                return;
            }
            if (state == MFS_CHECKED) {
                item.setSelected(true);
            } else if (state == MFS_UNCHECKED) {
                item.setSelected(false);
            } else {
                throw new UnsupportedOperationException("Not yet implemented");
            }
        }
    }

    public static class MyCheckBoxMenuItem extends JCheckBoxMenuItem {

        private final int id;

        public MyCheckBoxMenuItem(String title, int id, ActionListener al) {
            this(title, id, al, false);
        }

        public MyCheckBoxMenuItem(String title, int id, ActionListener al, boolean checked) {
            super(title, checked);
            this.id = id;
            this.addActionListener(al);
        }

        public int getID() {
            return id;
        }
    }

    public static MyMenuBar createMenu(final int id_menu, final GameWorld gameWorld) {
        MyMenuBar menu = new MyMenuBar(gameWorld);
        ActionListener al = menu.getActionListener();
        JMenu objects = new JMenu("Objects");
        JMenuItem walls = new MyCheckBoxMenuItem("Walls On/Off", ID_OB_WALLS, al);
        JMenuItem obstacles = new MyCheckBoxMenuItem("Obstacles On/Off", ID_OB_OBSTACLES, al);
        objects.add(walls);
        objects.add(obstacles);
        menu.add(objects);

        JMenu spacePartitioning = new JMenu("Space Partitioning");
        JMenuItem onOff = new MyCheckBoxMenuItem("On/Off", IDR_PARTITIONING, al);
        JMenuItem view = new MyCheckBoxMenuItem("View Neighbors", IDM_PARTITION_VIEW_NEIGHBORS, al);
        spacePartitioning.add(onOff);
        spacePartitioning.add(view);
        menu.add(spacePartitioning);

        JMenu calculate = new JMenu("Calculate Method");
        JMenuItem weightSum = new MyCheckBoxMenuItem("Weighted Sum", IDR_WEIGHTED_SUM, al);
        JMenuItem prioritized = new MyCheckBoxMenuItem("Prioritized", IDR_PRIORITIZED, al);
        JMenuItem dithered = new MyCheckBoxMenuItem("Dithered", IDR_DITHERED, al);
        calculate.add(weightSum);
        calculate.add(prioritized);
        calculate.add(dithered);
        menu.add(calculate);

        JMenu info = new JMenu("Info");
        JMenuItem keys = new MyCheckBoxMenuItem("Keys", ID_VIEW_KEYS, al);
        JMenuItem fps = new MyCheckBoxMenuItem("FPS", ID_VIEW_FPS, al);
        info.add(keys);
        info.add(fps);
        menu.add(info);

        JMenu smoothing = new JMenu("Smoothing");
        JMenuItem sOnOff = new MyCheckBoxMenuItem("On/Off", ID_MENU_SMOOTHING, al);
        smoothing.add(sOnOff);
        menu.add(smoothing);

        return menu;
    }
}