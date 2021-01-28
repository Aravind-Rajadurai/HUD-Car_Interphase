/**
 *  Desc: Inherit from this to create a class capable of reading
 *        parameters from an ascii file
 *
 *        instantiate this class with the name of the parameter file. Then
 *        call the helper functions to retrieve the data. 
 * 
 * @author Petr (http://www.sallyx.org/)
 */
package common.misc;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class iniFileLoaderBase {

    //the file the parameters are stored in
    private BufferedReader file;
    private String CurrentLine = "";

//-------------------------- GetParameterValueAsString ------------------------
//
// given a line of text this function removes the parameter description
// and returns just the parameter as a String
//-----------------------------------------------------------------------------
    private String GetParameterValueAsString(String line) {
        //find beginning of parameter description
        int begIdx;
        int endIdx;

        //define some delimiters
        final String delims = "[ ;=,]";
        final Pattern pattern = Pattern.compile(delims);
        String[] s = pattern.split(line);
        if (s.length > 0) {
            return s[s.length - 1];
        }
        return "";
    }

    //----------------------- GetNextParameter ------------------------------------
//
//  searches the text file for the next valid parameter. Discards any comments
//  and returns the value as a string
//-----------------------------------------------------------------------------
    String GetNextParameter() throws IOException {

        //this will be the string that holds the bext parameter
        String line;

        line = file.readLine();

        line = RemoveCommentingFromLine(line);

        //if the line is of zero length, get the next line from
        //the file
        if (line.length() == 0) {
            return GetNextParameter();
        }

        line = GetParameterValueAsString(line);
        return line;
    }

//--------------------------- GetNextToken ------------------------------------
//
//  ignores any commenting and gets the next delimited string
//-----------------------------------------------------------------------------
    String GetNextToken() throws IOException {
        //strip the line of any commenting
        while (CurrentLine.equals("")) {
            CurrentLine = file.readLine();
            CurrentLine = RemoveCommentingFromLine(CurrentLine);
        }

        //find beginning of parameter description
        int begIdx = CurrentLine.length();
        int endIdx = CurrentLine.length();

        //define some delimiters
        final String delims = "[ ;=,]+";
        final Pattern pattern = Pattern.compile(delims);
        Matcher matcher = pattern.matcher(CurrentLine);

        //begIdx = CurrentLine.find_first_not_of(delims);

        //find the end of the parameter description
        if (matcher.find()) {
            begIdx = matcher.end();
            if (matcher.find()) {
                endIdx = matcher.start();
            } else {
                endIdx = CurrentLine.length();
            }
        }

        String s = CurrentLine.substring(begIdx, endIdx);

        if (endIdx != CurrentLine.length()) {
            //strip the token from the line
            CurrentLine = CurrentLine.substring(endIdx + 1, CurrentLine.length());
        } else {
            CurrentLine = "";
        }

        return s;

    }
//this is set to true if the file specified by the user is valid
    private boolean m_bGoodFile;

    //helper methods. They convert the next parameter value found into the 
    //relevant type
    public double GetNextParameterDouble() throws IOException {
        if (m_bGoodFile) {
            return Double.valueOf(GetNextParameter());
        }
        throw new RuntimeException("bad file");
    }

    public float GetNextParameterFloat() throws IOException {
        if (m_bGoodFile) {
            return Float.valueOf(GetNextParameter());
        }
        throw new RuntimeException("bad file");
    }

    public int GetNextParameterInt() throws IOException {
        if (m_bGoodFile) {
            return Integer.valueOf(GetNextParameter());
        }
        throw new RuntimeException("bad file");
    }

    public boolean GetNextParameterBool() throws IOException {
        if (m_bGoodFile) {
            return 0 != Integer.valueOf(GetNextParameter());
        }
        throw new RuntimeException("bad file");
    }

    public double GetNextTokenAsDouble() throws IOException {
        if (m_bGoodFile) {
            return Double.valueOf(GetNextToken());
        }
        throw new RuntimeException("bad file");
    }

    public float GetNextTokenAsFloat() throws IOException {
        if (m_bGoodFile) {
            return Float.valueOf(GetNextToken());
        }
        throw new RuntimeException("bad file");
    }

    public int GetNextTokenAsInt() throws IOException {
        if (m_bGoodFile) {
            return Integer.valueOf(GetNextToken());
        }
        throw new RuntimeException("bad file");
    }

    public String GetNextTokenAsString() throws IOException {
        if (m_bGoodFile) {
            return GetNextToken();
        }
        throw new RuntimeException("bad file");
    }

    public boolean eof() throws IOException {
        if (m_bGoodFile) {
            return !file.ready();
        }
        throw new RuntimeException("bad file");
    }

    public boolean FileIsGood() {
        return m_bGoodFile;
    }

    public iniFileLoaderBase(URL filename) {
        CurrentLine = "";
        m_bGoodFile = true;
        try {
            file = new BufferedReader(new FileReader(new File(filename.toURI())));
        } catch (FileNotFoundException ex) {
            m_bGoodFile = false;
        } catch(URISyntaxException ex) {
            m_bGoodFile = false;
        }
    }

    //removes any commenting from a line of text
    public static String RemoveCommentingFromLine(String line) {
        //search for any comment and remove
        int idx = line.indexOf("//");

        if (idx != -1) {
            //cut out the comment
            return line.substring(0, idx);
        }
        return line;
    }
}