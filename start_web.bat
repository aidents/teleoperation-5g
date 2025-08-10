@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo 🚀 Iniciando SOLO la Interfaz Web de Teleoperación de Dron (Windows)
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

echo 🔍 Verificando directorio web...
if not exist "src\remote_interface\web" (
    echo ❌ Directorio web no encontrado
    echo    Asegúrate de ejecutar este script desde el directorio raíz del proyecto
    pause
    exit /b 1
)

echo ✅ Directorio web encontrado
echo.

echo 🌐 Iniciando SOLO la interfaz web...
echo.
echo 💡 Este script inicia SOLO la interfaz web Next.js usando Python
echo    Para funcionalidad completa con ROS2, necesitas WSL Ubuntu
echo.
echo 📱 Interfaz web: http://localhost:3000
echo.
echo 💡 Para detener: Ctrl+C
echo ================================================================
echo.

REM Ejecutar el script Python
python start_web.py

echo.
echo 🛑 Script completado
pause
