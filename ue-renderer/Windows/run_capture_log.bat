@echo off
cd /d "%~dp0testwl\Binaries\Win64"
testwl-Win64-Shipping.exe -renderoffscreen -nosound -minimalviewport -saveimage -log 2>&1 > "%~dp0capture.log"
