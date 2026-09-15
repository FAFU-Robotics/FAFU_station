Option Explicit
Dim sh, fso, folder, desktop, sc, target, ico, edge
Set sh = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
folder = fso.GetParentFolderName(WScript.ScriptFullName)
If WScript.Arguments.Count > 0 Then
  folder = WScript.Arguments(0)
End If
folder = fso.GetAbsolutePathName(folder)
desktop = sh.SpecialFolders("Desktop")
If desktop = "" Then
  desktop = sh.ExpandEnvironmentStrings("%USERPROFILE%") & "\Desktop"
End If
If Not fso.FolderExists(desktop) Then
  WScript.Echo "Desktop folder not found"
  WScript.Quit 1
End If

target = folder & "\FAFUArmStation.exe"
If Not fso.FileExists(target) Then
  WScript.Echo "FAFUArmStation.exe not found in " & folder
  WScript.Quit 1
End If

ico = folder & "\FAFUArmStation.ico"
If Not fso.FileExists(ico) Then
  ico = target & ",0"
End If

Set sc = sh.CreateShortcut(desktop & "\FAFUArmStation.lnk")
sc.TargetPath = target
sc.Arguments = ""
sc.WorkingDirectory = folder
sc.WindowStyle = 1
sc.Description = "FAFU Arm Station (private runtime)"
sc.IconLocation = ico
sc.Save
WScript.Echo "Desktop shortcut: " & desktop & "\FAFUArmStation.lnk"
