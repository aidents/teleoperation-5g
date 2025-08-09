@echo off
echo ===============================================
echo 🚁 Iniciando Servidor Web - Teleoperacion Dron
echo ===============================================
echo.

:: Cambiar al directorio web
cd "install\remote_interface\share\remote_interface\web"

if not exist "index.html" (
    echo ❌ Error: No se encontraron los archivos web
    echo Verifica que estás en el directorio correcto del proyecto
    pause
    exit /b 1
)

echo 📁 Directorio web encontrado
echo 📋 Archivos disponibles:
dir /b *.html *.js *.css

echo.
echo 🌐 Iniciando servidor HTTP en puerto 8080...
echo 🔗 La interfaz se abrirá automáticamente en: http://localhost:8080
echo ⏹️  Presiona Ctrl+C para detener el servidor
echo.

:: Abrir navegador después de 3 segundos
start "" timeout /t 3 >nul && start http://localhost:8080

:: Iniciar servidor
python -m http.server 8080

pause
