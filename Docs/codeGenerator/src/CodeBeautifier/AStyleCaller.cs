using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace astyle
{
    public class AStyleCaller
    {
        // thanks to https://astyle.sourceforge.net/develop/sharp.html
        public delegate void showInfo_delegate(string info);
        public static string beautify(string cppText, showInfo_delegate showInfo)
        {
            // options to pass to AStyle
            // mode=cs is required for C# files
            string options = "-A1tOP";

            // create an object
            AStyleInterface AStyle = new AStyleInterface();

            // get Artistic Style version
            // does not need to terminate on an error
            string version = AStyle.GetVersion();
            if (version == String.Empty)
                throw new Exception("Cannot get the version of AStyle");

            if (showInfo != null)
                showInfo("AStyle " + version);

            string textOut = AStyle.FormatSource(cppText, options);

            if (textOut == String.Empty)
                throw new Exception("Cannot format cpp text");

            if (showInfo != null)
                showInfo("Formatted cpp text");

            return textOut;
        }
    }
}
