#!/usr/bin/env python3
"""
Test del Sistema de Teleoperación de Dron
Script de prueba para verificar el funcionamiento básico
"""

import os
import sys
import time
import subprocess
from pathlib import Path

def test_python_environment():
    """Testea el entorno Python"""
    print("🐍 Testeando entorno Python...")
    
    try:
        import requests
        print("✅ requests instalado")
    except ImportError:
        print("❌ requests no instalado")
        return False
    
    try:
        import websocket
        print("✅ websocket-client instalado")
    except ImportError:
        print("⚠️  websocket-client no instalado (opcional)")
    
    print("✅ Entorno Python OK")
    return True

def test_node_environment():
    """Testea el entorno Node.js"""
    print("\n🟢 Testeando entorno Node.js...")
    
    try:
        result = subprocess.run(['node', '--version'], 
                              capture_output=True, text=True, check=True)
        print(f"✅ Node.js: {result.stdout.strip()}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ Node.js no encontrado")
        return False
    
    try:
        result = subprocess.run(['npm', '--version'], 
                              capture_output=True, text=True, check=True)
        print(f"✅ npm: {result.stdout.strip()}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ npm no encontrado")
        return False
    
    print("✅ Entorno Node.js OK")
    return True

def test_project_structure():
    """Testea la estructura del proyecto"""
    print("\n📁 Testeando estructura del proyecto...")
    
    project_root = Path(__file__).parent.parent
    required_dirs = [
        "src/remote_interface/web",
        "src/remote_interface/remote_interface",
        "src/drone_controller/drone_controller",
        "launch"
    ]
    
    required_files = [
        "src/remote_interface/web/package.json",
        "src/remote_interface/remote_interface/ros2_web_bridge.py",
        "src/drone_controller/drone_controller/drone_controller.py",
        "start_complete_system.py",
        "start_web_only_windows.py"
    ]
    
    # Verificar directorios
    for dir_path in required_dirs:
        full_path = project_root / dir_path
        if full_path.exists():
            print(f"✅ Directorio: {dir_path}")
        else:
            print(f"❌ Directorio faltante: {dir_path}")
            return False
    
    # Verificar archivos
    for file_path in required_files:
        full_path = project_root / file_path
        if full_path.exists():
            print(f"✅ Archivo: {file_path}")
        else:
            print(f"❌ Archivo faltante: {file_path}")
            return False
    
    print("✅ Estructura del proyecto OK")
    return True

def test_web_dependencies():
    """Testea las dependencias web"""
    print("\n🌐 Testeando dependencias web...")
    
    web_dir = Path(__file__).parent.parent / "src" / "remote_interface" / "web"
    
    if not web_dir.exists():
        print("❌ Directorio web no encontrado")
        return False
    
    package_json = web_dir / "package.json"
    if not package_json.exists():
        print("❌ package.json no encontrado")
        return False
    
    node_modules = web_dir / "node_modules"
    if node_modules.exists():
        print("✅ Dependencias web instaladas")
        return True
    else:
        print("⚠️  Dependencias web no instaladas")
        print("   Ejecuta: cd src/remote_interface/web && npm install --legacy-peer-deps")
        return False

def test_ros2_environment():
    """Testea el entorno ROS2"""
    print("\n🤖 Testeando entorno ROS2...")
    
    try:
        # Verificar si estamos en WSL/Linux
        if os.name == 'nt':  # Windows
            print("⚠️  Windows detectado - ROS2 requiere WSL")
            
            # Verificar si WSL está disponible
            try:
                result = subprocess.run(['wsl', '--list', '--verbose'], 
                                      capture_output=True, text=True, check=True)
                if 'Ubuntu' in result.stdout:
                    print("✅ WSL Ubuntu disponible")
                    return True
                else:
                    print("❌ WSL Ubuntu no encontrado")
                    return False
            except:
                print("❌ WSL no disponible")
                return False
        else:
            # Linux/WSL
            try:
                result = subprocess.run(['ros2', '--version'], 
                                      capture_output=True, text=True, check=True)
                print(f"✅ ROS2: {result.stdout.strip()}")
                return True
            except (subprocess.CalledProcessError, FileNotFoundError):
                print("❌ ROS2 no encontrado")
                return False
    
    except Exception as e:
        print(f"❌ Error verificando ROS2: {e}")
        return False

def run_all_tests():
    """Ejecuta todos los tests"""
    print("🧪 EJECUTANDO TESTS DEL SISTEMA")
    print("=" * 50)
    
    tests = [
        ("Entorno Python", test_python_environment),
        ("Entorno Node.js", test_node_environment),
        ("Estructura del Proyecto", test_project_structure),
        ("Dependencias Web", test_web_dependencies),
        ("Entorno ROS2", test_ros2_environment)
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"❌ Error en test {test_name}: {e}")
            results[test_name] = False
    
    # Resumen
    print("\n" + "=" * 50)
    print("📊 RESULTADOS DE LOS TESTS")
    print("=" * 50)
    
    total_tests = len(results)
    passed_tests = sum(1 for result in results.values() if result)
    
    for test_name, result in results.items():
        status = "✅ PASÓ" if result else "❌ FALLÓ"
        print(f"{test_name}: {status}")
    
    print(f"\n📈 Resumen: {passed_tests}/{total_tests} tests pasaron")
    
    if passed_tests == total_tests:
        print("🎉 ¡Todos los tests pasaron! El sistema está listo.")
    else:
        print("⚠️  Algunos tests fallaron. Revisa los errores arriba.")
    
    print("=" * 50)
    
    return results

def main():
    results = run_all_tests()
    
    # Código de salida
    if all(results.values()):
        sys.exit(0)  # Éxito
    else:
        sys.exit(1)  # Fallo

if __name__ == "__main__":
    main()
