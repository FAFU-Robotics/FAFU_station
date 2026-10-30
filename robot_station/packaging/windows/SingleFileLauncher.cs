// Single-file customer launcher. This exe carries the portable tree as a zip
// overlay (see pack_single_exe.ps1). First run unpacks to
// %LOCALAPPDATA%\FAFUArmStation then starts the thin FAFUArmStation.exe there.
// The user never opens a zip or looks for a .bat.
//
// Footer (little-endian), 48 bytes at EOF:
//   [zip payload]
//   [32-byte SHA-256 of the zip]
//   [8-byte int64 zip length]
//   [8-byte magic "FAFUEXE1"]
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Windows.Forms;

internal static class SingleFileLauncher
{
    const string Title = "FAFU 机械臂站控";
    const string Magic = "FAFUEXE1";
    const int FooterSize = 48;
    const string StampName = ".bundle_sha256";

    [STAThread]
    static int Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);

        string logPath = Path.Combine(Path.GetTempPath(), "fafu-station-setup.log");
        string dest = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "FAFUArmStation");
        string err = null;
        bool unpacked = false;

        Mutex mutex = null;
        bool created = false;
        try
        {
            mutex = new Mutex(true, @"Local\FAFUArmStation-SingleExe", out created);
            if (!created)
            {
                if (!mutex.WaitOne(180000))
                {
                    MessageBox.Show("已有一份安装正在进行，请稍后再试。", Title);
                    return 1;
                }
            }

            SplashForm splash = new SplashForm();
            splash.Show();
            Application.DoEvents();

            try
            {
                unpacked = Prepare(dest, logPath, splash);
            }
            catch (Exception ex)
            {
                err = ex.Message;
                Log(logPath, ex.ToString());
            }

            try { splash.Close(); } catch { }
            try { splash.Dispose(); } catch { }
            Application.DoEvents();
        }
        finally
        {
            if (mutex != null)
            {
                try { mutex.ReleaseMutex(); } catch { }
                mutex.Close();
            }
        }

        if (err != null)
        {
            MessageBox.Show(
                "无法准备站控。\n" + err + "\n\n日志: " + logPath,
                Title);
            return 1;
        }

        string inner = Path.Combine(dest, "FAFUArmStation.exe");
        if (!File.Exists(inner))
        {
            MessageBox.Show(
                "安装后找不到 FAFUArmStation.exe。\n请重新下载安装文件。\n日志: " + logPath,
                Title);
            return 1;
        }

        EnsureShortcut(dest, logPath);
        Log(logPath, unpacked ? "launch after unpack" : "launch existing " + inner);

        try
        {
            ProcessStartNoWindow(inner, dest);
        }
        catch (Exception ex)
        {
            Log(logPath, ex.ToString());
            MessageBox.Show("无法启动站控。\n日志: " + logPath + "\n\n" + ex.Message, Title);
            return 1;
        }
        return 0;
    }

    static bool Prepare(string dest, string logPath, SplashForm splash)
    {
        string self = Application.ExecutablePath;
        Log(logPath, "single-file launcher " + DateTime.Now.ToString("s"));
        Log(logPath, "self=" + self);
        Log(logPath, "dest=" + dest);

        byte[] hash;
        long zipLen;
        long zipOffset;
        ParseFooter(self, out hash, out zipLen, out zipOffset);
        Log(logPath, "payload offset=" + zipOffset + " len=" + zipLen);

        string stampPath = Path.Combine(dest, StampName);
        string hex = Hex(hash);
        string inner = Path.Combine(dest, "FAFUArmStation.exe");
        string python = Path.Combine(dest, "runtime", "python310", "python.exe");
        if (File.Exists(inner) && File.Exists(python) && File.Exists(stampPath))
        {
            string prev = "";
            try { prev = File.ReadAllText(stampPath, Encoding.ASCII).Trim(); } catch { }
            if (string.Equals(prev, hex, StringComparison.OrdinalIgnoreCase))
            {
                splash.SetStatus("正在启动机械臂站控…");
                Application.DoEvents();
                Log(logPath, "bundle stamp matches, skip unpack");
                return false;
            }
        }

        splash.SetStatus("正在安装机械臂站控（只需这一次）…");
        Application.DoEvents();
        EnsureInstallAvailable(dest);
        VerifyPayload(self, zipOffset, zipLen, hash);

        string staging = Path.Combine(Path.GetTempPath(), "FAFUArmStation-unpack-" + Guid.NewGuid().ToString("N"));
        string tmpZip = staging + ".zip";
        Directory.CreateDirectory(staging);
        try
        {
            CopyPayload(self, zipOffset, zipLen, tmpZip);
            ExtractZip(tmpZip, staging);
            if (!File.Exists(Path.Combine(staging, "FAFUArmStation.exe")))
                throw new InvalidOperationException("安装包内没有 FAFUArmStation.exe");
            File.WriteAllText(Path.Combine(staging, StampName), hex + "\r\n", Encoding.ASCII);
            ReplaceInstall(staging, dest);
            Log(logPath, "unpacked " + zipLen + " bytes sha256=" + hex);
        }
        catch (IOException ex)
        {
            throw new InvalidOperationException(
                "无法写入安装目录。请先关闭已打开的机械臂站控再运行此文件。\n" + ex.Message,
                ex);
        }
        catch (UnauthorizedAccessException ex)
        {
            throw new InvalidOperationException(
                "安装文件正在使用或不可写。请关闭旧版站控及其后台进程后重试。\n" + ex.Message,
                ex);
        }
        finally
        {
            try { if (File.Exists(tmpZip)) File.Delete(tmpZip); } catch { }
            try { if (Directory.Exists(staging)) Directory.Delete(staging, true); } catch { }
        }
        return true;
    }

    static void ParseFooter(string self, out byte[] hash, out long zipLen, out long zipOffset)
    {
        hash = null;
        zipLen = 0;
        zipOffset = 0;
        long fileLen = new FileInfo(self).Length;
        if (fileLen < FooterSize + 64)
            throw new InvalidOperationException("这不是完整的站控安装文件（文件过小）。请重新下载。");

        byte[] footer = new byte[FooterSize];
        using (FileStream fs = File.OpenRead(self))
        {
            fs.Seek(fileLen - FooterSize, SeekOrigin.Begin);
            ReadExact(fs, footer, FooterSize);
        }

        string magic = Encoding.ASCII.GetString(footer, 40, 8);
        if (magic != Magic)
            throw new InvalidOperationException("这不是完整的站控安装文件。请发送打包生成的单个 FAFUArmStation.exe，不要只拷便携目录里的薄启动器。");

        zipLen = BitConverter.ToInt64(footer, 32);
        if (zipLen < 64 || zipLen > fileLen - FooterSize)
            throw new InvalidOperationException("安装包载荷长度无效。");

        zipOffset = fileLen - FooterSize - zipLen;
        hash = new byte[32];
        Buffer.BlockCopy(footer, 0, hash, 0, 32);
    }

    static void VerifyPayload(string self, long zipOffset, long zipLen, byte[] expected)
    {
        using (FileStream fs = File.OpenRead(self))
        {
            fs.Seek(zipOffset, SeekOrigin.Begin);
            using (SHA256 sha = SHA256.Create())
            {
                byte[] actual = HashStream(sha, fs, zipLen);
                if (!HashesEqual(expected, actual))
                    throw new InvalidOperationException("安装包已损坏（校验失败）。请重新下载。");
            }
        }
    }

    static byte[] HashStream(SHA256 sha, Stream src, long count)
    {
        byte[] buf = new byte[1024 * 1024];
        long left = count;
        while (left > 0)
        {
            int n = (int)Math.Min(buf.Length, left);
            int got = src.Read(buf, 0, n);
            if (got <= 0)
                throw new EndOfStreamException("安装包读取中断");
            sha.TransformBlock(buf, 0, got, null, 0);
            left -= got;
        }
        sha.TransformFinalBlock(new byte[0], 0, 0);
        return sha.Hash;
    }

    static void CopyPayload(string self, long offset, long length, string destZip)
    {
        using (FileStream src = File.OpenRead(self))
        using (FileStream dst = File.Create(destZip))
        {
            src.Seek(offset, SeekOrigin.Begin);
            byte[] buf = new byte[1024 * 1024];
            long left = length;
            while (left > 0)
            {
                int n = (int)Math.Min(buf.Length, left);
                int got = src.Read(buf, 0, n);
                if (got <= 0)
                    throw new EndOfStreamException("安装包读取中断");
                dst.Write(buf, 0, got);
                left -= got;
            }
        }
    }

    static void ExtractZip(string zipPath, string dest)
    {
        string root = Path.GetFullPath(dest);
        if (!root.EndsWith(Path.DirectorySeparatorChar.ToString()))
            root = root + Path.DirectorySeparatorChar;
        using (ZipArchive zip = ZipFile.OpenRead(zipPath))
        {
            foreach (ZipArchiveEntry entry in zip.Entries)
            {
                string full = Path.GetFullPath(Path.Combine(dest, entry.FullName.Replace('/', Path.DirectorySeparatorChar)));
                if (!full.StartsWith(root, StringComparison.OrdinalIgnoreCase))
                    throw new InvalidOperationException("安装包内含非法路径");
                if (string.IsNullOrEmpty(entry.Name))
                {
                    Directory.CreateDirectory(full);
                    continue;
                }
                string dir = Path.GetDirectoryName(full);
                if (!string.IsNullOrEmpty(dir))
                    Directory.CreateDirectory(dir);
                entry.ExtractToFile(full, true);
            }
        }
    }

    static void ReplaceInstall(string staging, string dest)
    {
        staging = Path.GetFullPath(staging).TrimEnd(Path.DirectorySeparatorChar);
        dest = Path.GetFullPath(dest).TrimEnd(Path.DirectorySeparatorChar);
        string parent = Path.GetDirectoryName(dest);
        if (string.IsNullOrEmpty(parent) || IsWithin(staging, dest) || IsWithin(dest, staging))
            throw new InvalidOperationException("安装暂存目录与目标目录不能重叠。");
        // A payload must never replace user programs, waypoints or trajectories.
        if (Directory.Exists(Path.Combine(staging, "recordings")) || File.Exists(Path.Combine(staging, "recordings")))
            throw new InvalidOperationException("安装包不能包含 recordings 用户数据目录。");
        EnsureInstallAvailable(dest);
        Directory.CreateDirectory(dest);
        string backup = Path.Combine(parent, Path.GetFileName(dest) + "-previous-" + Guid.NewGuid().ToString("N"));
        List<string> moved = new List<string>();
        List<string> copied = new List<string>();
        Directory.CreateDirectory(backup);
        try
        {
            foreach (string child in InstallEntries(dest))
            {
                string name = Path.GetFileName(child);
                MoveEntry(child, Path.Combine(backup, name));
                moved.Add(name);
            }
            foreach (string child in Directory.GetFileSystemEntries(staging))
            {
                string name = Path.GetFileName(child);
                copied.Add(name); // Track partial copies as well.
                string target = Path.Combine(dest, name);
                if (Directory.Exists(child))
                    CopyTree(child, target);
                else
                    File.Copy(child, target, false);
            }
        }
        catch (Exception installError)
        {
            try
            {
                foreach (string name in copied)
                    DeleteEntry(Path.Combine(dest, name));
                foreach (string name in moved)
                    MoveEntry(Path.Combine(backup, name), Path.Combine(dest, name));
                Directory.Delete(backup);
            }
            catch (Exception rollbackError)
            {
                throw new InvalidOperationException(
                    "更新失败，旧版备份保留在：\n" + backup + "\n用户录制目录未改动。\n" +
                    installError.Message + "\n恢复失败：" + rollbackError.Message, installError);
            }
            throw new InvalidOperationException(
                "更新失败，旧版文件已恢复，用户录制目录未改动。\n" + installError.Message, installError);
        }
        // The new runtime is complete, including its stamp. A delayed antivirus
        // handle on the backup must not turn a successful update into a failure.
        try { Directory.Delete(backup, true); }
        catch (Exception ex)
        {
            Log(Path.Combine(Path.GetTempPath(), "fafu-station-setup.log"), "old backup cleanup " + backup + ": " + ex.Message);
        }
    }

    static bool IsWithin(string path, string root)
    {
        return string.Equals(path, root, StringComparison.OrdinalIgnoreCase) ||
            path.StartsWith(root + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase);
    }

    static IEnumerable<string> InstallEntries(string root)
    {
        foreach (string child in Directory.GetFileSystemEntries(root))
            if (!string.Equals(Path.GetFileName(child), "recordings", StringComparison.OrdinalIgnoreCase))
                yield return child;
    }

    static IEnumerable<string> InstallFiles(string entry)
    {
        if ((File.GetAttributes(entry) & FileAttributes.ReparsePoint) != 0)
            throw new InvalidOperationException("安装目录中存在链接，不能自动替换：" + entry);
        if (!Directory.Exists(entry))
        {
            yield return entry;
            yield break;
        }
        foreach (string child in Directory.GetFileSystemEntries(entry))
            foreach (string file in InstallFiles(child))
                yield return file;
    }

    static void EnsureInstallAvailable(string dest)
    {
        if (!Directory.Exists(dest)) return;
        string root = Path.GetFullPath(dest).TrimEnd(Path.DirectorySeparatorChar);
        List<string> running = new List<string>();
        foreach (Process process in Process.GetProcesses())
        {
            using (process)
            {
                try
                {
                    string exe = process.MainModule.FileName;
                    if (IsWithin(Path.GetFullPath(exe), root))
                        running.Add(process.ProcessName + "（PID " + process.Id + "）");
                }
                catch { } // Other users' protected processes need not be inspected.
            }
        }
        if (running.Count > 0)
            throw new InvalidOperationException(
                "旧版站控后台仍在运行，尚未修改安装文件。\n" + string.Join("、", running.ToArray()) +
                "\n\n关闭窗口不会结束后台服务。请确认机械臂停止并可靠支撑，关闭旧版后台后重试。" +
                "\n安装程序不会自动结束机械臂控制进程。");
        foreach (string entry in InstallEntries(root))
        {
            foreach (string file in InstallFiles(entry))
            {
                try
                {
                    // Opening without writing detects locked native modules and
                    // read-only files before any installed file can be moved.
                    using (FileStream probe = new FileStream(file, FileMode.Open, FileAccess.ReadWrite, FileShare.None)) { }
                }
                catch (Exception ex)
                {
                    if (!(ex is IOException) && !(ex is UnauthorizedAccessException)) throw;
                    throw new InvalidOperationException(
                        "安装文件正在使用或不可写，尚未修改安装文件：\n" + file +
                        "\n请关闭旧版站控及其后台进程后重试。", ex);
                }
            }
        }
    }

    static void MoveEntry(string source, string target)
    {
        if (Directory.Exists(source)) Directory.Move(source, target);
        else File.Move(source, target);
    }

    static void DeleteEntry(string path)
    {
        if (Directory.Exists(path)) Directory.Delete(path, true);
        else if (File.Exists(path)) File.Delete(path);
    }

    static void CopyTree(string src, string dest)
    {
        Directory.CreateDirectory(dest);
        foreach (string dir in Directory.GetDirectories(src))
            CopyTree(dir, Path.Combine(dest, Path.GetFileName(dir)));
        foreach (string file in Directory.GetFiles(src))
        {
            string to = Path.Combine(dest, Path.GetFileName(file));
            File.Copy(file, to, true);
        }
    }

    static void ProcessStartNoWindow(string exe, string workDir)
    {
        System.Diagnostics.ProcessStartInfo psi = new System.Diagnostics.ProcessStartInfo();
        psi.FileName = exe;
        psi.WorkingDirectory = workDir;
        psi.UseShellExecute = false;
        psi.CreateNoWindow = true;
        System.Diagnostics.Process proc = System.Diagnostics.Process.Start(psi);
        if (proc == null)
            throw new InvalidOperationException("无法启动 " + exe);
    }

    static void EnsureShortcut(string root, string logPath)
    {
        try
        {
            string vbs = Path.Combine(root, "install_shortcut.vbs");
            if (!File.Exists(vbs))
                return;
            System.Diagnostics.ProcessStartInfo sc = new System.Diagnostics.ProcessStartInfo();
            sc.FileName = "cscript";
            sc.Arguments = "//nologo \"" + vbs + "\" \"" + root + "\"";
            sc.UseShellExecute = false;
            sc.CreateNoWindow = true;
            using (System.Diagnostics.Process p = System.Diagnostics.Process.Start(sc))
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

    static void ReadExact(Stream src, byte[] buf, int n)
    {
        int off = 0;
        while (off < n)
        {
            int got = src.Read(buf, off, n - off);
            if (got <= 0)
                throw new EndOfStreamException("安装包读取中断");
            off += got;
        }
    }

    static bool HashesEqual(byte[] a, byte[] b)
    {
        if (a == null || b == null || a.Length != b.Length)
            return false;
        int diff = 0;
        for (int i = 0; i < a.Length; i++)
            diff |= a[i] ^ b[i];
        return diff == 0;
    }

    static string Hex(byte[] hash)
    {
        StringBuilder sb = new StringBuilder(hash.Length * 2);
        for (int i = 0; i < hash.Length; i++)
            sb.Append(hash[i].ToString("X2"));
        return sb.ToString();
    }

    static void Log(string path, string line)
    {
        try
        {
            File.AppendAllText(path, DateTime.Now.ToString("HH:mm:ss") + " " + line + "\r\n", Encoding.UTF8);
        }
        catch { }
    }
}

internal sealed class SplashForm : Form
{
    readonly Label _label;
    readonly ProgressBar _bar;

    public SplashForm()
    {
        Text = "FAFU 机械臂站控";
        FormBorderStyle = FormBorderStyle.FixedDialog;
        StartPosition = FormStartPosition.CenterScreen;
        MinimizeBox = false;
        MaximizeBox = false;
        ShowInTaskbar = true;
        ClientSize = new Size(460, 150);
        BackColor = Color.FromArgb(20, 29, 39);
        ForeColor = Color.FromArgb(232, 238, 245);

        _label = new Label();
        _label.AutoSize = false;
        _label.SetBounds(24, 28, 412, 48);
        _label.Font = new Font("Segoe UI", 11f);
        _label.ForeColor = Color.FromArgb(232, 238, 245);
        _label.Text = "正在启动机械臂站控…";
        Controls.Add(_label);

        _bar = new ProgressBar();
        _bar.SetBounds(24, 90, 412, 22);
        _bar.Style = ProgressBarStyle.Marquee;
        _bar.MarqueeAnimationSpeed = 30;
        Controls.Add(_bar);
    }

    public void SetStatus(string text)
    {
        if (IsDisposed)
            return;
        if (InvokeRequired)
        {
            BeginInvoke(new Action<string>(SetStatus), text);
            return;
        }
        _label.Text = text;
    }
}
