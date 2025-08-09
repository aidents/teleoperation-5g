#!/usr/bin/env python3
import http.server
import socketserver
import os
import webbrowser
import time
import threading

def main():
    # Cambiar al directorio web
    web_dir = r"install\remote_interface\share\remote_interface\web"
    if os.path.exists(web_dir):
        os.chdir(web_dir)
        print(f"📁 Sirviendo desde: {os.path.abspath('.')}")
    else:
        print(f"❌ Directorio no encontrado: {web_dir}")
        print("Archivos disponibles:")
        for file in os.listdir("."):
            if file.endswith(".html"):
                print(f"  📄 {file}")
        return

    PORT = 8080
    
    class CustomHandler(http.server.SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            super().end_headers()

    def open_browser():
        time.sleep(2)
        webbrowser.open(f'http://localhost:{PORT}')
        print(f"🌐 Navegador abierto: http://localhost:{PORT}")

    try:
        with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
            print(f"✅ Servidor iniciado en http://localhost:{PORT}")
            print("📁 Archivos disponibles:")
            for file in os.listdir("."):
                if os.path.isfile(file):
                    size = os.path.getsize(file)
                    print(f"  📄 {file} ({size} bytes)")
            
            print(f"\n🔗 Acceder desde: http://localhost:{PORT}")
            print("⏹️  Presiona Ctrl+C para detener\n")
            
            # Abrir navegador automáticamente
            browser_thread = threading.Thread(target=open_browser, daemon=True)
            browser_thread.start()
            
            httpd.serve_forever()
            
    except KeyboardInterrupt:
        print("\n🛑 Servidor detenido")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
