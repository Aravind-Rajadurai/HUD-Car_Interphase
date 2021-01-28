/**
 * @author Petr (http://www.sallyx.org/)
 */
package common.Messaging;

import Chapter3Steering.MessageTypes;

public class Telegram implements Comparable {
    //the entity that sent this telegram

    public int Sender;
    //the entity that is to receive this telegram
    public int Receiver;
    //the message itself. These are all enumerated in the file
    //"MessageTypes.h"
    public MessageTypes Msg;
    //messages can be dispatched immediately or delayed for a specified amount
    //of time. If a delay is necessary this field is stamped with the time 
    //the message should be dispatched.
    public double DispatchTime;
    //any additional information that may accompany the message
    public Object ExtraInfo;

    public Telegram() {
        DispatchTime = -1;
        Sender = -1;
        Receiver = -1;
        Msg = null;
    }

    public Telegram(double time,
            int sender,
            int receiver,
            MessageTypes msg) {
        this(time, sender, receiver, msg, null);
    }

    public Telegram(double time,
            int sender,
            int receiver,
            MessageTypes msg,
            Object info) {
        DispatchTime = time;
        Sender = sender;
        Receiver = receiver;
        Msg = msg;
        ExtraInfo = info;
    }
//these telegrams will be stored in a priority queue. Therefore the >
//operator needs to be overloaded so that the PQ can sort the telegrams
//by time priority. Note how the times must be smaller than
//SmallestDelay apart before two Telegrams are considered unique.
    public final static double SmallestDelay = 0.25;

    /**
     *  "overloads" == operaotr
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Telegram)) {
            return false;
        }
        Telegram t1 = this;
        Telegram t2 = (Telegram) o;
        return (Math.abs(t1.DispatchTime - t2.DispatchTime) < SmallestDelay)
                && (t1.Sender == t2.Sender)
                && (t1.Receiver == t2.Receiver)
                && (t1.Msg == t2.Msg);
    }

    /**
     *  It is generally necessary to override the hashCode method 
     *  whenever equals method is overridden, so as to maintain the 
     *  general contract for the hashCode method, which states that 
     *  equal objects must have equal hash codes.
     */
    @Override
    public int hashCode() {
        int hash = 3;
        hash = 89 * hash + this.Sender;
        hash = 89 * hash + this.Receiver;
        hash = 89 * hash + this.Msg.id;
        return hash;
    }
    /**
     * "overloads" < and > operators
     */
    @Override
    public int compareTo(Object o2) {
        Telegram t1 = this;
        Telegram t2 = (Telegram) o2;
        if (t1 == t2) {
            return 0;
        } else {
            return (t1.DispatchTime < t2.DispatchTime) ? -1 : 1;
        }
    }

    @Override
    public String toString() {
        return "time: " + DispatchTime + "  Sender: " + Sender
                + "   Receiver: " + Receiver + "   Msg: " + Msg;
    }

//handy helper function for dereferencing the ExtraInfo field of the Telegram 
//to the required type.
    public static <T> T DereferenceToType(Object p) {
        return (T) (p);
    }
}