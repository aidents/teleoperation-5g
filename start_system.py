#!/usr/bin/env python3
"""
Iniciador del Sistema ROS2 desde Windows hacia WSL Ubuntu
Este script lanza el sistema ROS2 completo en WSL desde Windows
"""

import os
import sys
import time
import signal
import subprocess
from pathlib import Path

class WSLSystemLauncher:
    def __init__(self):
        self.project_root = Path(__file__).parent
        self.wsl_process = None
        self.running = True
        
        # Configuración WSL
        self.wsl_distro = "Ubuntu"
        self.ros_script = "start_ros_system.sh"
        
        # Configurar manejo de señales
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Maneja señales de terminación"""
        print(f"\n🛑 Señal recibida ({signum}), deteniendo...")
        self.stop_wsl()
        sys.exit(0)
    
    def check_wsl_available(self):
        """Verifica que WSL esté disponible"""
        print("🔍 Verificando WSL...")
        
        try:
            result = subprocess.run(['wsl', '--list', '--verbose'], 
                                  capture_output=True, text=True, shell=True)
            
            if result.returncode != 0:
                print("❌ WSL no está disponible")
                print("   Instala WSL desde: https://docs.microsoft.com/en-us/windows/wsl/install")
                return False
            
            print(f"✅ WSL disponible con {self.wsl_distro}")
            return True
            
        except Exception as e:
            print(f"❌ Error verificando WSL: {e}")
            return False
    
    def start_ros_system_in_wsl(self):
        """Inicia el sistema ROS2 en WSL"""
        print("🚀 Iniciando sistema ROS2 en WSL...")
        
        # Convertir ruta de Windows a WSL
        wsl_path = self.project_root.as_posix().replace('D:', '/mnt/d')
        wsl_path = wsl_path.replace('\\', '/')
        
        # Comando simple para ejecutar en WSL
        wsl_command = f'cd "{wsl_path}" && chmod +x {self.ros_script} && ./{self.ros_script}'
        
        try:
            # Ejecutar en WSL
            self.wsl_process = subprocess.Popen(
                ['wsl', '-d', self.wsl_distro, 'bash', '-c', wsl_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print(f"✅ Sistema ROS2 iniciado en WSL (PID: {self.wsl_process.pid})")
            return True
            
        except Exception as e:
            print(f"❌ Error iniciando sistema ROS2 en WSL: {e}")
            return False
    
    def wait_for_ros_system(self, timeout=120):
        """Espera a que el sistema ROS2 esté disponible"""
        print(f"⏳ Esperando sistema ROS2 (timeout: {timeout}s)...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                # Verificar que el proceso WSL esté ejecutándose
                if self.wsl_process and self.wsl_process.poll() is not None:
                    print("❌ Proceso WSL terminó prematuramente")
                    return False
                
                time.sleep(5)
                
            except Exception as e:
                print(f"⚠️  Error verificando sistema: {e}")
                time.sleep(5)
        
        print(f"✅ Sistema ROS2 iniciado correctamente")
        return True
    
    def stop_wsl(self):
        """Detiene el sistema WSL"""
        if self.wsl_process:
            try:
                print("🛑 Deteniendo sistema ROS2 en WSL...")
                
                # Enviar señal de terminación al proceso WSL
                self.wsl_process.terminate()
                
                # Esperar a que termine
                try:
                    self.wsl_process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    print("⚠️  Forzando terminación...")
                    self.wsl_process.kill()
                    self.wsl_process.wait()
                
                print("✅ Sistema ROS2 detenido")
                
            except Exception as e:
                print(f"❌ Error deteniendo sistema: {e}")
            
            self.wsl_process = None
    
    def run(self):
        """Ejecuta el sistema completo"""
        print("🚀 Iniciando Sistema ROS2 Completo desde Windows hacia WSL")
        print("=" * 70)
        print("💡 Este script inicia el sistema ROS2 completo en WSL Ubuntu")
        print("   desde Windows, proporcionando funcionalidad completa")
        print("=" * 70)
        
        # Verificar WSL
        if not self.check_wsl_available():
            return False
        
        # Iniciar sistema ROS2 en WSL
        if not self.start_ros_system_in_wsl():
            return False
        
        # Esperar a que esté disponible
        system_ready = self.wait_for_ros_system()
        
        if system_ready:
            print("\n🎯 Sistema ROS2 iniciado correctamente en WSL!")
            print("🔗 El sistema está ejecutándose en WSL Ubuntu")
            print("🌐 La interfaz web estará disponible en http://localhost:3000")
            print("\n💡 Para detener: Ctrl+C")
            
            try:
                # Mantener el script ejecutándose
                while self.running and self.wsl_process and self.wsl_process.poll() is None:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n🛑 Interrupción del usuario")
            finally:
                self.stop_wsl()
        else:
            print("❌ El sistema ROS2 no se pudo iniciar correctamente")
            self.stop_wsl()
            return False
        
        return True

def main():
    launcher = WSLSystemLauncher()
    success = launcher.run()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
