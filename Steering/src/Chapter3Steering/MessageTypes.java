/**
 * @author Petr (http://www.sallyx.org/)
 */
package Chapter3Steering;

public enum MessageTypes {

    Msg_Empty(0);
    
    final public int id;

    MessageTypes(int id) {
        this.id = id;
    }

    @Override
    public String toString() {
        return MsgToStr(this.id);
    }
    public static String MsgToStr(MessageTypes msg) {
        return MsgToStr(msg.id);
    
    }
    public static String MsgToStr(int msg) {
        switch (msg) {
            case 0:
                return "HiHoneyImHome";
            case 1:
                return "StewReady";
            default:
                return "Not recognized!";
        }
    }
}