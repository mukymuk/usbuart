@echo off
"%CROSSWORKS%\bin\crossbuild.exe" %1 -config "THUMB Debug" ..\cw\max32620\usbuart.hzp
