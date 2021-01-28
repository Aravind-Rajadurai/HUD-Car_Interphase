/**
 * 
 *  Desc:   various useful functions that operate on or with streams
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package common.misc;

public class Stream_Utility_function {
/**
 * convert a type to a string
*/
public static <T extends Number> String ttos(final T t) {
    return ttos(t,2);
}
public static <T extends Number> String ttos(final T t, int precision)
{
   double multipicationFactor = Math.pow(10, precision);
   double interestedInZeroDPs = t.doubleValue() * multipicationFactor;
   return "" + (Math.round(interestedInZeroDPs) / multipicationFactor);
}

/**
 *  convert a bool to a string
*/
public static String btos(boolean b)
{
  if (b) return "true";
  return "false";
}

/**
*  grabs a value of the specified type from an input stream
*/
/*
public static <T>
 T GetValueFromStream(InputStream stream)
{

}
*/

/**
/* writes the value as a binary string of bits
*/
/*
template <typename T>
void WriteBitsToStream(std::ostream& stream, const T& val)
{
  int iNumBits = sizeof(T) * 8;

  while (--iNumBits >= 0)
  {
    if ((iNumBits+1) % 8 == 0) stream << " ";
    unsigned long mask = 1 << iNumBits;
    if (val & mask) stream << "1";
    else stream << "0";
  }
} */ 
}
