@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo 🚀 Iniciando Sistema ROS2 Completo desde Windows hacia WSL
echo ================================================================
echo.

echo 🔍 Verificando Python...
python --version >nul 2>nul
if %errorlevel% neq 0 (
    echo ❌ Python no está instalado o no está en el PATH
    echo    Por favor instala Python desde https://python.org/
    echo    Asegúrate de marcar "Add to PATH" durante la instalación
    pause
    exit /b 1
)

echo ✅ Python encontrado
echo.

echo 🔍 Verificando WSL...
wsl --list --verbose >nul 2>nul
if %errorlevel% neq 0 (
    echo ❌ WSL no está disponible
    echo    Instala WSL desde: https://docs.microsoft.com/en-us/windows/wsl/install
    pause
    exit /b 1
)

echo ✅ WSL disponible
echo.

echo 🚀 Iniciando sistema ROS2 completo en WSL...
echo.
echo 💡 Este script inicia el sistema ROS2 completo en WSL Ubuntu
echo    desde Windows, proporcionando funcionalidad completa
echo.
echo 🔗 Sistema ROS2: Ejecutándose en WSL Ubuntu
echo 🌐 Interfaz web: http://localhost:3000
echo.
echo 💡 Para detener: Ctrl+C
echo ================================================================
echo.

REM Ejecutar el script Python
python start_system.py

echo.
echo 🛑 Script completado
pause
