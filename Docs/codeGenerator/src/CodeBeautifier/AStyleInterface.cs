// AStyleInterface.cs

using System;
using System.Runtime.InteropServices;

/// AStyleInterface contains methods to call the Artistic Style formatter.
public class AStyleInterface
{   // Dll name
// Windows will NOT attach an extension to a filename containing dots (.).
#if (DEBUG)
    private const string dllName = "astyle32d.dll";
#else
    private const string dllName = "astyle32.dll";
#endif

    /// AStyleGetVersion DllImport.
    /// Cannot use string as a return value because Mono runtime will attempt to
    /// free the returned pointer resulting in a runtime crash.
    /// NOTE: CharSet.Unicode is NOT used here.
    [DllImport(dllName)]
    private static extern IntPtr AStyleGetVersion();

    /// AStyleMainUtf16 DllImport.
    /// Cannot use string as a return value because Mono runtime will attempt to
    /// free the returned pointer resulting in a runtime crash.
    /// NOTE: CharSet.Unicode and wide strings ARE used here.
    [DllImport(dllName, CharSet = CharSet.Unicode)]
    private static extern IntPtr AStyleMainUtf16(
        [MarshalAs(UnmanagedType.LPWStr)] string textIn,
        [MarshalAs(UnmanagedType.LPWStr)] string options,
        AStyleErrorDelgate AStyleError,
        AStyleMemAllocDelgate AStyleMemAlloc
    );

    /// AStyleMainUtf16 callbacks.
    /// NOTE: Wide strings are NOT used here.
    private delegate void AStyleErrorDelgate(
        int errorNum,
        [MarshalAs(UnmanagedType.LPStr)] string error
    );
    private delegate IntPtr AStyleMemAllocDelgate(int size);

    /// Error handler to abort the program.
    private void Error(string message)
    {
        Console.WriteLine(message);
        Console.WriteLine("The program has terminated!");
        Environment.Exit(1);
    }

    /// Call the AStyleMainUtf16 function in Artistic Style.
    /// An empty string is returned on error.
    public string FormatSource(string textIn, string options)
    {   // Return the allocated string
        // Memory space is allocated by AStyleMemAlloc, a callback function
        string sTextOut = String.Empty;
        try
        {
            IntPtr pText = AStyleMainUtf16(textIn, options,
                                           AStyleError, AStyleMemAlloc);
            if (pText != IntPtr.Zero)
            {
                sTextOut = Marshal.PtrToStringUni(pText);
                Marshal.FreeHGlobal(pText);
            }
        }
        catch (BadImageFormatException e)
        {
            Console.WriteLine(e.ToString());
            Error("You may be mixing 32 and 64 bit code!");
        }
        catch (DllNotFoundException)
        {   //Console.WriteLine(e.ToString());
            Error("Cannot load native library: " + dllName);
        }
        catch (Exception e)
        {
            Error(e.ToString());
        }
        return sTextOut;
    }

    /// Get the Artistic Style version number.
    public string GetVersion()
    {
        string sVersion = String.Empty;
        try
        {
            IntPtr pVersion = AStyleGetVersion();
            if (pVersion != IntPtr.Zero)
            {
                sVersion = Marshal.PtrToStringAnsi(pVersion);
            }
        }
        catch (BadImageFormatException e)
        {
            Console.WriteLine(e.ToString());
            Error("You may be mixing 32 and 64 bit code!");
        }
        catch (DllNotFoundException)
        {   //Console.WriteLine(e.ToString());
            Error("Cannot load native library: " + dllName);
        }
        catch (Exception e)
        {
            Error(e.ToString());
        }
        return sVersion;
    }

    /// AStyleMainUtf16 callback to allocate memory for the return string.
    private IntPtr AStyleMemAlloc(int size)
    {
        return Marshal.AllocHGlobal(size);
    }

    /// AStyleMainUtf16 callback to display errors from Artistic Style.
    private void AStyleError(int errorNumber, string error)
    {
        Console.WriteLine("AStyle error " + errorNumber + "\n" + error);
    }

}   // class AStyleInterface
