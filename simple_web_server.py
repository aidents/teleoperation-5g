#!/usr/bin/env python3

import http.server
import socketserver
import os
import sys
from pathlib import Path
import threading
import webbrowser
import time

def main():
    # Cambiar al directorio web
    web_dir = Path("install/remote_interface/share/remote_interface/web")
    if not web_dir.exists():
        print(f"❌ Directorio web no encontrado: {web_dir.absolute()}")
        return False

    os.chdir(web_dir)
    print(f"📁 Sirviendo archivos desde: {web_dir.absolute()}")

    PORT = 8080
    
    class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            super().end_headers()
        
        def log_message(self, format, *args):
            print(f"🌐 {self.address_string()} - {format % args}")

    try:
        with socketserver.TCPServer(("0.0.0.0", PORT), CustomHTTPRequestHandler) as httpd:
            print(f"✅ Servidor web iniciado exitosamente en puerto {PORT}")
            print(f"🔗 Acceder desde Windows: http://localhost:{PORT}")
            print(f"🔗 Acceder desde WSL: http://0.0.0.0:{PORT}")
            print(f"📱 Interfaz principal: http://localhost:{PORT}/index.html")
            print()
            print("📁 Archivos disponibles:")
            for item in sorted(Path(".").glob("*")):
                if item.is_file():
                    size = item.stat().st_size
                    print(f"  📄 {item.name} ({size} bytes)")
            print()
            print("⏹️  Presiona Ctrl+C para detener el servidor")
            print("=" * 60)
            
            # Función para abrir navegador automáticamente
            def open_browser():
                time.sleep(2)
                try:
                    webbrowser.open(f'http://localhost:{PORT}')
                    print("🌐 Navegador abierto automáticamente")
                except:
                    pass
            
            # Abrir navegador en un hilo separado
            browser_thread = threading.Thread(target=open_browser, daemon=True)
            browser_thread.start()
            
            httpd.serve_forever()
            
    except OSError as e:
        if "Address already in use" in str(e):
            print(f"❌ Puerto {PORT} ya está en uso")
            print("💡 Intenta detener otros servidores o usar otro puerto")
            return False
        else:
            print(f"❌ Error iniciando servidor: {e}")
            return False
    except KeyboardInterrupt:
        print("\n🛑 Servidor detenido por el usuario")
        return True
    except Exception as e:
        print(f"❌ Error inesperado: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

