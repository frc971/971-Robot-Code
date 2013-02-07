Dim WshShell, oExec, re, Revision
Set WshShell = CreateObject("WScript.Shell")
'Get the svn revision of WPILib that is being installed
Set oExec = WshShell.Exec("svnversion -n ../WPILib")

'We wait for the end of process
Do While oExec.Status = 0
    WScript.Sleep 100
Loop

'Read the command output 
Do While oExec.StdOut.AtEndOfStream <> True 
    Revision = oExec.StdOut.ReadLine
Loop

'Check for partial sync (:) or modified files (M)
Set re = new regexp
re.Pattern = "[M:]"
If re.Test(Revision) <> true Then
	Wscript.echo(Year(Date()) & _
        Right("0" & Month(date()), 2) & _
        Right("0" & Day(date()), 2) & _
		"rev" & Revision)
Else
	Wscript.echo("BadSVNSync")
End If