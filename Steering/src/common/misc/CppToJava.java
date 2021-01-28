/**
 * @author Petr (http://www.sallyx.org/)
 */
package common.misc;

import java.awt.event.KeyEvent;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

final public class CppToJava {

    final public static KeyCache keyCache = new KeyCache();

    public static class KeyCache {
        private Map<Integer,Boolean> keys;
        private KeyCache() {
            keys = new HashMap<Integer,Boolean>();
        }
        //copy ctor and assignment should be private

        private KeyCache(KeyCache kc) {
        }

        @Override
        protected Object clone() throws CloneNotSupportedException {
            throw new CloneNotSupportedException("Cloning not allowed");
        }

        public void released(KeyEvent e) {
            keys.put(e.getKeyCode(), Boolean.FALSE);
        }

        public void pressed(KeyEvent e) {
            keys.put(e.getKeyCode(), Boolean.TRUE);
        }
        
        public boolean keyDown(int key) {
            Boolean b = keys.get(key);
            if(b == null) return false;
            return b;
        }
    }
    
    public static boolean KEYDOWN(int c) {
        return keyCache.keyDown(c)
                || keyCache.keyDown(Character.toLowerCase(c)) || 
                keyCache.keyDown(Character.toUpperCase(c));
    }

    public static class DoubleRef extends AtomicReference<Double> {

        public DoubleRef(double ref) {
            super(ref);
        }

        public double toDouble() {
            return this.get();
        }
    }

    public static <T extends Object> List<T> clone(List<T> list) {
        try {
            List<T> c = list.getClass().newInstance();
            for (T t : list) {
                T copy = (T) t.getClass().getDeclaredConstructor(t.getClass()).newInstance(t);
                c.add(copy);
            }
            return c;
        } catch (Exception e) {
            throw new RuntimeException("List cloning unsupported", e);
        }
    }
}
