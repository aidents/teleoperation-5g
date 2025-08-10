#!/usr/bin/env python3
"""
Iniciador SOLO de Interfaz Web para Windows
Este script inicia únicamente la interfaz web Next.js en Windows
"""

import os
import sys
import time
import signal
import subprocess
import threading
from pathlib import Path

class WebOnlyLauncher:
    def __init__(self):
        self.project_root = Path(__file__).parent
        self.web_process = None
        self.running = True
        
        # Configuración
        self.web_port = 3000
        
        # Detectar npm para Windows
        self.npm_command = self.detect_npm()
        
        # Configurar manejo de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def detect_npm(self):
        """Detecta la ubicación de npm en Windows"""
        # Rutas comunes de npm en Windows
        npm_paths = [
            r"C:\Program Files\nodejs\npm.cmd",
            r"C:\Program Files (x86)\nodejs\npm.cmd",
            os.path.expanduser(r"~\AppData\Roaming\npm\npm.cmd")
        ]
        
        for path in npm_paths:
            if os.path.exists(path):
                print(f"✅ npm encontrado en: {path}")
                return f'"{path}"'
        
        # Si no se encuentra en rutas comunes, usar PATH
        try:
            result = subprocess.run(['where', 'npm'], capture_output=True, text=True, shell=True)
            if result.returncode == 0:
                npm_path = result.stdout.strip().split('\n')[0]
                print(f"✅ npm encontrado en PATH: {npm_path}")
                return f'"{npm_path}"'
        except:
            pass
        
        print("⚠️  npm no encontrado en rutas comunes, usando PATH")
        return "npm"
    
    def signal_handler(self, signum, frame):
        """Maneja señales de terminación"""
        print(f"\n🛑 Señal recibida ({signum}), deteniendo...")
        self.stop_web()
        sys.exit(0)
    
    def start_web_interface(self):
        """Inicia la interfaz web Next.js"""
        print("🌐 Iniciando interfaz web...")
        
        web_dir = self.project_root / "src" / "remote_interface" / "web"
        if not web_dir.exists():
            print("❌ Directorio web no encontrado")
            return False
        
        try:
            # Verificar si node_modules existe
            if not (web_dir / "node_modules").exists():
                print("📦 Instalando dependencias web...")
                install_cmd = f"{self.npm_command} install --legacy-peer-deps"
                print(f"Ejecutando: {install_cmd}")
                
                result = subprocess.run(install_cmd, shell=True, cwd=web_dir, check=True)
                if result.returncode != 0:
                    print("❌ Error instalando dependencias")
                    return False
                
                print("✅ Dependencias instaladas")
            
            # Iniciar servidor de desarrollo
            print("🚀 Iniciando servidor de desarrollo...")
            dev_cmd = f"{self.npm_command} run dev"
            print(f"Ejecutando: {dev_cmd}")
            
            self.web_process = subprocess.Popen(
                dev_cmd, 
                shell=True, 
                cwd=web_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print(f"✅ Interfaz web iniciada (PID: {self.web_process.pid})")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"❌ Error iniciando interfaz web: {e}")
            return False
        except Exception as e:
            print(f"❌ Error inesperado: {e}")
            return False
    
    def wait_for_web_interface(self, timeout=60):
        """Espera a que la interfaz web esté disponible"""
        print(f"⏳ Esperando interfaz web (timeout: {timeout}s)...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                import requests
                response = requests.get(f"http://localhost:{self.web_port}", timeout=5)
                if response.status_code == 200:
                    print(f"✅ Interfaz web disponible en http://localhost:{self.web_port}")
                    return True
            except:
                pass
            
            time.sleep(2)
        
        print(f"❌ Timeout esperando interfaz web")
        return False
    
    def stop_web(self):
        """Detiene la interfaz web"""
        if self.web_process:
            try:
                print("🛑 Deteniendo interfaz web...")
                self.web_process.terminate()
                self.web_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print("⚠️  Forzando terminación...")
                self.web_process.kill()
            except Exception as e:
                print(f"❌ Error deteniendo: {e}")
            
            self.web_process = None
    
    def run(self):
        """Ejecuta solo la interfaz web"""
        print("🚀 Iniciando SOLO la Interfaz Web de Teleoperación de Dron")
        print("=" * 60)
        print("💡 Este script inicia SOLO la interfaz web Next.js")
        print("   Para funcionalidad completa con ROS2, necesitas WSL Ubuntu")
        print("=" * 60)
        
        # Iniciar interfaz web
        if not self.start_web_interface():
            print("❌ No se pudo iniciar la interfaz web")
            return False
        
        # Esperar a que esté disponible
        web_ready = self.wait_for_web_interface()
        
        if web_ready:
            print("\n🎯 Interfaz web iniciada correctamente!")
            print(f"🌐 URL: http://localhost:{self.web_port}")
            print("\n💡 Para detener: Ctrl+C")
            
            try:
                # Mantener el script ejecutándose
                while self.running and self.web_process and self.web_process.poll() is None:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n🛑 Interrupción del usuario")
            finally:
                self.stop_web()
        else:
            print("❌ La interfaz web no se pudo iniciar correctamente")
            self.stop_web()
            return False
        
        return True

def main():
    launcher = WebOnlyLauncher()
    success = launcher.run()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
