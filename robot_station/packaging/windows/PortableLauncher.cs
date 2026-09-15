// Private-runtime launcher. Process-local environment only.
// Does not write PATH, registry, or the customer's Python.
using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Windows.Forms;

internal static class PortableLauncher
{
    const string Title = "FAFU 机械臂站控";

    [STAThread]
    static int Main()
    {
        string logPath = Path.Combine(Path.GetTempPath(), "fafu-station-app.log");
        try
        {
            string root = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(
                Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            Log(logPath, "portable launcher " + DateTime.Now.ToString("s"));
            Log(logPath, "root=" + root);

            string dest = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                "FAFUArmStation");
            if (!SameDir(root, dest))
            {
                Log(logPath, "first-run copy to " + dest);
                if (InstallTo(root, dest, logPath) && File.Exists(Path.Combine(dest, "FAFUArmStation.exe")))
                {
                    ProcessStartInfo next = new ProcessStartInfo();
                    next.FileName = Path.Combine(dest, "FAFUArmStation.exe");
                    next.WorkingDirectory = dest;
                    next.UseShellExecute = true;
                    Process.Start(next);
                    return 0;
                }
                Log(logPath, "install copy failed, starting from zip folder");
            }

            string python = Path.Combine(root, "runtime", "python310", "python.exe");
            string appDir = Path.Combine(root, "app");
            string desktop = Path.Combine(appDir, "station_desktop.py");
            if (!File.Exists(python) || !File.Exists(desktop))
            {
                MessageBox.Show(
                    "便携包不完整：找不到私有 Python 或 app\\station_desktop.py。\n"
                    + "请用 packaging\\windows\\build_portable.ps1 重新打包。\n"
                    + "日志: " + logPath,
                    Title);
                return 1;
            }

            string sdk = Path.Combine(appDir, "vendor", "fafu_arm_sdk");
            if (!Directory.Exists(Path.Combine(sdk, "fafu_robot_python")))
                sdk = Path.Combine(root, "vendor", "fafu_arm_sdk");

            ProcessStartInfo psi = new ProcessStartInfo();
            psi.FileName = python;
            psi.Arguments = "-u \"" + desktop + "\"";
            psi.WorkingDirectory = appDir;
            psi.UseShellExecute = false;
            psi.CreateNoWindow = true;

            psi.EnvironmentVariables["STATION_PORTABLE"] = "1";
            psi.EnvironmentVariables["STATION_INSTALL_ROOT"] = root;
            psi.EnvironmentVariables["STATION_ALLOW_LIVE_ARM"] = "1";
            psi.EnvironmentVariables["PYTHONNOUSERSITE"] = "1";
            psi.EnvironmentVariables["PIP_USER"] = "0";
            psi.EnvironmentVariables["PYTHONPATH"] = appDir;
            psi.EnvironmentVariables.Remove("PYTHONSTARTUP");
            psi.EnvironmentVariables.Remove("PYTHONHOME");
            psi.EnvironmentVariables.Remove("PYTHONUSERBASE");
            if (Directory.Exists(Path.Combine(sdk, "fafu_robot_python")))
                psi.EnvironmentVariables["FAFU_ARM_SDK"] = sdk;

            string pyDir = Path.GetDirectoryName(python);
            string scripts = Path.Combine(pyDir, "Scripts");
            string libBin = Path.Combine(pyDir, "Library", "bin");
            string path = psi.EnvironmentVariables["PATH"] ?? "";
            string prefix = pyDir + ";" + scripts;
            if (Directory.Exists(libBin))
            {
                prefix = libBin + ";" + prefix;
                psi.EnvironmentVariables["PINOCCHIO_WINDOWS_DLL_PATH"] = libBin;
            }
            psi.EnvironmentVariables["PATH"] = prefix + ";" + path;

            Log(logPath, "start " + python + " " + psi.Arguments);
            using (Process proc = Process.Start(psi))
            {
                if (proc == null)
                {
                    MessageBox.Show("无法启动私有 Python。\n日志: " + logPath, Title);
                    return 1;
                }
                proc.WaitForExit();
                Log(logPath, "exit " + proc.ExitCode);
                return proc.ExitCode;
            }
        }
        catch (Exception ex)
        {
            try { File.AppendAllText(logPath, ex.ToString() + "\r\n", Encoding.UTF8); } catch { }
            MessageBox.Show("无法启动站控。\n日志: " + logPath + "\n\n" + ex.Message, Title);
            return 1;
        }
    }

    static bool SameDir(string a, string b)
    {
        try
        {
            return string.Equals(
                Path.GetFullPath(a).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar),
                Path.GetFullPath(b).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar),
                StringComparison.OrdinalIgnoreCase);
        }
        catch
        {
            return false;
        }
    }

    static bool InstallTo(string root, string dest, string logPath)
    {
        try
        {
            Directory.CreateDirectory(dest);
            ProcessStartInfo copy = new ProcessStartInfo();
            copy.FileName = "robocopy";
            copy.Arguments = "\"" + root + "\" \"" + dest + "\" /E /XD cache /NFL /NDL /NJH /NJS /nc /ns /np";
            copy.UseShellExecute = false;
            copy.CreateNoWindow = true;
            using (Process p = Process.Start(copy))
            {
                if (p == null)
                    return false;
                p.WaitForExit();
                if (p.ExitCode >= 8)
                {
                    Log(logPath, "robocopy " + p.ExitCode);
                    return false;
                }
            }
            string vbs = Path.Combine(dest, "install_shortcut.vbs");
            if (File.Exists(vbs))
            {
                ProcessStartInfo sc = new ProcessStartInfo();
                sc.FileName = "cscript";
                sc.Arguments = "//nologo \"" + vbs + "\" \"" + dest + "\"";
                sc.UseShellExecute = false;
                sc.CreateNoWindow = true;
                using (Process p = Process.Start(sc))
                {
                    if (p != null)
                        p.WaitForExit(8000);
                }
            }
            return true;
        }
        catch (Exception ex)
        {
            Log(logPath, "install " + ex.Message);
            return false;
        }
    }

    static void Log(string path, string line)
    {
        try { File.AppendAllText(path, DateTime.Now.ToString("HH:mm:ss") + " " + line + "\r\n", Encoding.UTF8); } catch { }
    }
}
