/**
 * @author Petr (http://www.sallyx.org/)
 */
package common.Time;

public class CrudeTimer {

    final public static CrudeTimer Clock = new CrudeTimer();
    //set to the time (in seconds) when class is instantiated
    final private double m_dStartTime;

    //set the start time
    private CrudeTimer() {
        m_dStartTime = System.currentTimeMillis() * 0.001;
    }

    //copy ctor and assignment should be private
    private CrudeTimer(CrudeTimer c) {
	    m_dStartTime = c.m_dStartTime;
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException("Cloning not allowed");
    }

    public CrudeTimer Instance() {
        return Clock;
    }

    //returns how much time has elapsed since the timer was started
    public double GetCurrentTime() {
        //return System.currentTimeMillis() * 0.001 - m_dStartTime;
        // The truncation of the results was added to produce results similar to what is produced by the C++ code
        // Improved by A.Rick Anderson
        return ((double) (Math.round((System.currentTimeMillis() * 0.001 - m_dStartTime) * 1000))) / 1000;
    }
}
