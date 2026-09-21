// Private-runtime launcher. Process-local environment only.
// Runs in place next to this exe. Does not copy the tree, write PATH,
// or touch the customer's Python. Optional Install.bat still copies
// to %LOCALAPPDATA%\FAFUArmStation for users who want to delete the zip.
using System;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Windows.Forms;
using Microsoft.Win32;

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

            string python = Path.Combine(root, "runtime", "python310", "python.exe");
            string appDir = Path.Combine(root, "app");
            string desktop = Path.Combine(appDir, "station_desktop.py");
            if (!File.Exists(python) || !File.Exists(desktop))
            {
                MessageBox.Show(
                    "便携包不完整：找不到私有 Python 或 app\\station_desktop.py。\n"
                    + "请把整个 FAFUArmStation 文件夹放在一起，不要只拷 exe。\n"
                    + "日志: " + logPath,
                    Title);
                return 1;
            }

            if (!RunPreflight(root, python, logPath))
                return 1;

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
            string urdf = Path.Combine(appDir, "urdf");
            if (Directory.Exists(urdf))
                psi.EnvironmentVariables["STATION_URDF_DIR"] = urdf;

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
                EnsureShortcut(root, logPath);
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

    static bool RunPreflight(string root, string python, string logPath)
    {
        string pyDir = Path.GetDirectoryName(python) ?? "";
        bool webview = HasWebView2();
        bool vcruntime = HasVcRuntime(pyDir);
        string[] ports = ComPorts();
        Log(logPath, "preflight webview2=" + webview + " vcruntime=" + vcruntime
            + " com=" + (ports.Length == 0 ? "(none)" : string.Join(",", ports)));

        if (!webview)
        {
            DialogResult go = MessageBox.Show(
                "未检测到 Microsoft Edge WebView2。独立控制窗口可能打不开。\n"
                + "请安装 WebView2 运行时后再试：\n"
                + "https://go.microsoft.com/fwlink/p/?LinkId=2124703\n\n"
                + "日志: " + logPath + "\n\n仍要继续启动吗？",
                Title,
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);
            if (go != DialogResult.Yes)
                return false;
        }
        if (!vcruntime)
        {
            DialogResult go = MessageBox.Show(
                "未找到 VC++ 运行库（vcruntime140.dll）。\n"
                + "真机 USB 模块可能无法加载。请安装 Microsoft Visual C++ Redistributable (x64)。\n\n"
                + "日志: " + logPath + "\n\n仍要继续启动吗？",
                Title,
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);
            if (go != DialogResult.Yes)
                return false;
        }
        if (ports.Length == 0)
        {
            MessageBox.Show(
                "未检测到串口。若已插机械臂 USB，请在设备管理器确认 COM 口，"
                + "并安装调试板驱动（CH340 / CP210x / FTDI）。\n"
                + "软件仍会打开，可先用仿真臂。\n\n日志: " + logPath,
                Title,
                MessageBoxButtons.OK,
                MessageBoxIcon.Information);
        }
        return true;
    }

    static bool HasWebView2()
    {
        string guid = @"{F3017226-FE2A-4295-8BDF-00C3A9A7E4C5}";
        string[] keys =
        {
            @"SOFTWARE\WOW6432Node\Microsoft\EdgeUpdate\Clients\" + guid,
            @"SOFTWARE\Microsoft\EdgeUpdate\Clients\" + guid,
            @"SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Microsoft EdgeWebView",
            @"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Microsoft EdgeWebView",
        };
        foreach (string k in keys)
        {
            try
            {
                using (RegistryKey hk = Registry.LocalMachine.OpenSubKey(k))
                {
                    if (hk != null) return true;
                }
                using (RegistryKey hk = Registry.CurrentUser.OpenSubKey(k))
                {
                    if (hk != null) return true;
                }
            }
            catch { }
        }
        try
        {
            string x86 = Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86);
            if (!string.IsNullOrEmpty(x86)
                && Directory.Exists(Path.Combine(x86, "Microsoft", "EdgeWebView", "Application")))
                return true;
        }
        catch { }
        return false;
    }

    static bool HasVcRuntime(string pyDir)
    {
        string[] names = { "vcruntime140.dll", "VCRUNTIME140.dll" };
        string[] dirs =
        {
            pyDir ?? "",
            Environment.SystemDirectory ?? "",
        };
        foreach (string d in dirs)
        {
            if (string.IsNullOrEmpty(d)) continue;
            foreach (string n in names)
            {
                if (File.Exists(Path.Combine(d, n)))
                    return true;
            }
        }
        return false;
    }

    static string[] ComPorts()
    {
        try
        {
            return SerialPort.GetPortNames() ?? new string[0];
        }
        catch
        {
            return new string[0];
        }
    }

    static void EnsureShortcut(string root, string logPath)
    {
        try
        {
            string vbs = Path.Combine(root, "install_shortcut.vbs");
            if (!File.Exists(vbs))
                return;
            ProcessStartInfo sc = new ProcessStartInfo();
            sc.FileName = "cscript";
            sc.Arguments = "//nologo \"" + vbs + "\" \"" + root + "\"";
            sc.UseShellExecute = false;
            sc.CreateNoWindow = true;
            using (Process p = Process.Start(sc))
            {
                if (p != null)
                    p.WaitForExit(2500);
            }
        }
        catch (Exception ex)
        {
            Log(logPath, "shortcut " + ex.Message);
        }
    }

    static void Log(string path, string line)
    {
        try { File.AppendAllText(path, DateTime.Now.ToString("HH:mm:ss") + " " + line + "\r\n", Encoding.UTF8); } catch { }
    }
}
