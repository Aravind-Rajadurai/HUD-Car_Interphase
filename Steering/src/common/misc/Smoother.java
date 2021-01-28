/**
 * 
 *  Desc: Template class to help calculate the average value of a history
 *        of values. This can only be used with types that have a 'zero'
 *        value and that have the += and / operators overloaded.
 *
 *        Example: Used to smooth frame rate calculations.
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package common.misc;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

public class Smoother<T extends Number> {
    //this holds the history

    private List<T> m_History;
    private int m_iNextUpdateSlot;
    //an example of the 'zero' value of the type to be smoothed. This
    //would be something like Vector2D(0,0)
    private T m_ZeroValue;

    /**
     * to instantiate a Smoother pass it the number of samples you want
     * to use in the smoothing, and an exampe of a 'zero' type
     */
    public Smoother(int SampleSize, T ZeroValue) {
        m_History = new ArrayList<T>(SampleSize);
        for(int i = 0; i < SampleSize; i++)
            m_History.add(ZeroValue);
        m_ZeroValue = ZeroValue;
        m_iNextUpdateSlot = 0;
    }

    //each time you want to get a new average, feed it the most recent value
    //and this method will return an average over the last SampleSize updates
    public T Update(T MostRecentValue) {
        //overwrite the oldest value with the newest
        m_History.set(m_iNextUpdateSlot++,MostRecentValue);

        //make sure m_iNextUpdateSlot wraps around. 
        if (m_iNextUpdateSlot == m_History.size()) {
            m_iNextUpdateSlot = 0;
        }

        //now to calculate the average of the history list
        T sum = null;
        try {
            sum = (T) m_ZeroValue.getClass().getConstructor(Double.TYPE).newInstance(0.0);
        }catch(Exception e) {
            e.printStackTrace();
            System.exit(1);
        }

        ListIterator<T> it = m_History.listIterator();

        while (it.hasNext()) {
            sum = add(sum, it.next());
        }

        sum = div(sum, (double) m_History.size());
        return sum;
    }

    public <X extends Number> X add(X a, X b) {
        return (X) new Double(a.doubleValue() + b.doubleValue());
    }

    public <X extends Number> X div(X a, double b) {
        return (X) new Double(a.doubleValue() / b);
    }
}
