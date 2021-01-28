/**
 * @author Petr (http://www.sallyx.org/)
 */
package common.misc;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import javax.swing.JDialog;
import Chapter3Steering.Script1.MyMenuBar;
import Chapter3Steering.Script1;
import javax.swing.JFrame;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;
import static common.windows.*;

public class WindowUtils {
    //JFrame + Menu

    public static class Window extends JDialog {

        JMenuBar menu;

        public Window(String title) {
            super((JFrame) null, title);
            this.addWindowListener(new WindowAdapter() {

                @Override
                public void windowClosing(WindowEvent e) {
                    System.exit(0);
                }
            });
        }

        public MyMenuBar getMenu() {
            JMenuBar bar = this.getJMenuBar();
            if (bar == null) {
                return null;
            }
            return (MyMenuBar) bar;
        }

        @Override
        public void setJMenuBar(JMenuBar menu) {
            assert (menu instanceof MyMenuBar);
            super.setJMenuBar(menu);
        }
    }

    /**
     *  Changes the state of a menu item given the item identifier, the 
     *  desired state and the HWND of the menu owner
     */
    public static void ChangeMenuState(Script1.MyMenuBar hwnd, int MenuItem, long state) {
        //hwnd.setMenuState(MenuItem,state);
        hwnd.setMenuState(MenuItem, state);
    }

    /**
     * Instead of SendMessage(hwnd, WM_COMMAND, MenuItem, NULL);
     */
    public static void SendChangeMenuMessage(Script1.MyMenuBar hwnd, int MenuItem) {
        hwnd.changeMenuState(MenuItem);
    }

    /**
     * if b is true MenuItem is checked, otherwise it is unchecked
     */
    public static void CheckMenuItemAppropriately(Script1.MyMenuBar hwnd, int MenuItem, boolean b) {
        if (b) {
            ChangeMenuState(hwnd, MenuItem, MFS_CHECKED);
        } else {
            ChangeMenuState(hwnd, MenuItem, MFS_UNCHECKED);
        }
    }

    /**
     *  this is a replacement for the StringCchLength function found in the 
     *  platform SDK. See MSDN for details. Only ever used for checking toolbar
     *  strings
     */
    public static boolean CheckBufferLength(String buff, int MaxLength, int BufferLength) {
        return true;
    }

    public static void ErrorBox(String msg) {
        JOptionPane.showMessageDialog(null, msg, "Error", JOptionPane.ERROR_MESSAGE);
    }
}
