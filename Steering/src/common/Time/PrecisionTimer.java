/**
 *
 *  Desc: Windows timer class.
 *
 *        nb. this only uses the high performance timer. There is no
 *        support for ancient computers. I know, I know, I should add
 *        support, but hey, I have shares in AMD and Intel... Go upgrade ;o)
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package common.Time;

public class PrecisionTimer {

    private Long m_CurrentTime,
            m_LastTime,
            m_LastTimeInTimeElapsed,
            m_NextTime,
            m_StartTime,
            m_FrameTime,
            m_PerfCountFreq;
    private double m_TimeElapsed,
            m_LastTimeElapsed,
            m_TimeScale;
    private double m_NormalFPS;
    private double m_SlowFPS;
    private boolean m_bStarted;
    //if true a call to TimeElapsed() will return 0 if the current
    //time elapsed is much smaller than the previous. Used to counter
    //the problems associated with the user using menus/resizing/moving 
    //a window etc
    private boolean m_bSmoothUpdates;

    /**
     * default constructor
     */
    public PrecisionTimer() {
        m_NormalFPS = 0.0;
        m_SlowFPS = 1.0;
        m_TimeElapsed = 0.0;
        m_FrameTime = 0L;
        m_LastTime = 0L;
        m_LastTimeInTimeElapsed = 0L;
        m_PerfCountFreq = 0L;
        m_bStarted = false;
        m_StartTime = 0L;
        m_LastTimeElapsed = 0.0;
        m_bSmoothUpdates = false;

        //how many ticks per sec do we get
        //QueryPerformanceFrequency((LARGE_INTEGER *) & m_PerfCountFreq);
        //using System.nanoSecond() it is obviously 1 000 000 000 nano second per second
        m_PerfCountFreq = 1000000000L;

        m_TimeScale = 1.0 / m_PerfCountFreq;
    }

    /**
    /* use to specify FPS
     */
    public PrecisionTimer(double fps) {
        m_NormalFPS = fps;
        m_SlowFPS = 1.0;
        m_TimeElapsed = 0.0;
        m_FrameTime = 0L;
        m_LastTime = 0L;
        m_LastTimeInTimeElapsed = 0L;
        m_PerfCountFreq = 0L;
        m_bStarted = false;
        m_StartTime = 0L;
        m_LastTimeElapsed = 0.0;
        m_bSmoothUpdates = false;

        //how many ticks per sec do we get
        //QueryPerformanceFrequency((LARGE_INTEGER *) & m_PerfCountFreq);
        //using System.nanoSecond() it is obviously 1 000 000 000 nano second per second
        m_PerfCountFreq = 1000000000L;

        m_TimeScale = 1.0 / m_PerfCountFreq;

        //calculate ticks per frame
        m_FrameTime = (long) (m_PerfCountFreq / m_NormalFPS);
    }

    /**
     *  whatdayaknow, this starts the timer
     *  call this immediately prior to game loop. Starts the timer (obviously!)
     *
     */
    public void Start() {
        m_bStarted = true;

        m_TimeElapsed = 0.0;

        //get the time
        //QueryPerformanceCounter((LARGE_INTEGER *) & m_LastTime);
        m_LastTime = System.nanoTime();

        //keep a record of when the timer was started
        m_StartTime = m_LastTimeInTimeElapsed = m_LastTime;

        //update time to render next frame
        m_NextTime = m_LastTime + m_FrameTime;

        return;
    }

    //determines if enough time has passed to move onto next frame
    //public boolean    ReadyForNextFrame();
    //only use this after a call to the above.
    //double  GetTimeElapsed(){return m_TimeElapsed;}
    //public double  TimeElapsed();

    /**
     * Get time elapsed from Start()
     * @return
     */
    public double CurrentTime() {
        //QueryPerformanceCounter((LARGE_INTEGER *) & m_CurrentTime);
        m_CurrentTime = System.nanoTime();

        return (m_CurrentTime - m_StartTime) * m_TimeScale;
    }

    public boolean Started() {
        return m_bStarted;
    }

    public void SmoothUpdatesOn() {
        m_bSmoothUpdates = true;
    }

    public void SmoothUpdatesOff() {
        m_bSmoothUpdates = false;
    }

    /**
     *  returns true if it is time to move on to the next frame step. To be used if
     *  FPS is set.
     */
    public boolean ReadyForNextFrame() {
        assert m_NormalFPS != 0 : "PrecisionTimer::ReadyForNextFrame<No FPS set in timer>";

        //QueryPerformanceCounter((LARGE_INTEGER *) & m_CurrentTime);
        m_CurrentTime = System.nanoTime();

        if (m_CurrentTime > m_NextTime) {

            m_TimeElapsed = (m_CurrentTime - m_LastTime) * m_TimeScale;
            m_LastTime = m_CurrentTime;

            //update time to render next frame
            m_NextTime = m_CurrentTime + m_FrameTime;

            return true;
        }

        return false;
    }

    /**
     *  returns time elapsed since last call to this function.
     */
    public double TimeElapsed() {
        m_LastTimeElapsed = m_TimeElapsed;

        //QueryPerformanceCounter((LARGE_INTEGER *) & m_CurrentTime);
        m_CurrentTime = System.nanoTime();

        m_TimeElapsed = (m_CurrentTime - m_LastTimeInTimeElapsed) * m_TimeScale;

        m_LastTimeInTimeElapsed = m_CurrentTime;

        final double Smoothness = 5.0;

        if (m_bSmoothUpdates) {
            if (m_TimeElapsed < (m_LastTimeElapsed * Smoothness)) {
                return m_TimeElapsed;
            } else {
                return 0.0;
            }
        } else {
            return m_TimeElapsed;
        }
    }
}