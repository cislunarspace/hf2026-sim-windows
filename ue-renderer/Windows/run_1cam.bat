@echo off
cd /d "%~dp0testwl\Binaries\Win64"
testwl-Win64-Shipping.exe -renderoffscreen -nosound -minimalviewport -saveimage -log
