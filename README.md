# Teleoperación de Dron con ROS 2 y 5G

Este proyecto implementa un sistema de teleoperación de dron usando ROS 2 con simulación de red 5G para control remoto.

## ⚡ Inicio Rápido

**¿Solo quieres ver el sistema funcionando?**

1. **Windows**: Ejecuta `start_web_server.bat` → Se abre la interfaz web
2. **WSL**: Ejecuta `./start_ros_system.sh` → Inicia el sistema ROS 2  
3. **Listo**: ¡Ya puedes controlar el dron desde http://localhost:8080!

## 🚁 Características

- **Control remoto de dron**: Interfaz web para controlar el dron desde cualquier dispositivo
- **Simulación de red 5G**: Configuración de QoS para simular latencia y ancho de banda de red 5G
- **ROS 2 Topics y Servicios**: Comunicación robusta entre componentes
- **Interfaz web responsive**: Control intuitivo con joystick virtual y botones
- **Monitoreo en tiempo real**: Visualización de telemetría del dron

## 📁 Estructura del Proyecto

```
drone_teleop_5g/
├── src/
│   ├── drone_controller/          # Controlador del dron
│   ├── remote_interface/          # Interfaz web remota
│   ├── network_simulator/         # Simulador de red 5G
│   └── drone_simulator/           # Simulador del dron
├── config/                        # Configuraciones QoS y red
├── launch/                        # Archivos de lanzamiento
├── docs/                          # Documentación
└── scripts/                       # Scripts de instalación y utilidades
```

## 🛠️ Requisitos

- **ROS 2 Rolling** (instalado en WSL Ubuntu)
- **Python 3.8+** (tanto en Windows como en WSL)
- **WSL 2** con Ubuntu 
- **Navegador web moderno**

## 🚀 Instalación

1. **Clonar el repositorio:**
```bash
git clone <repository-url>
cd "Teleoperacion con dron"
```

2. **Configurar entorno WSL:**
```bash
# En WSL Ubuntu
cd "/mnt/d/Documentos/Programacion/ROS 2/Teleoperacion con dron"
source /opt/ros/rolling/setup.bash
source install/setup.bash
```

3. **El workspace ya está compilado y listo para usar**

## 🎮 Uso Rápido

### **PASO 1: Iniciar Interfaz Web (Windows)**

**Opción A - Script Automático (RECOMENDADO):**
```cmd
# Doble clic en el archivo o ejecutar desde CMD:
start_web_server.bat
```

**Opción B - Manual:**
En **PowerShell o CMD** desde el directorio del proyecto:

```powershell
# Navegar al directorio web
cd "install\remote_interface\share\remote_interface\web"

# Iniciar servidor HTTP
python -m http.server 8080
```

**Resultado:** Se abrirá automáticamente **http://localhost:8080**

### **PASO 2: Iniciar Sistema ROS 2 (WSL)**

**Opción A - Script Automático (RECOMENDADO):**
```bash
# En terminal WSL Ubuntu
cd "/mnt/d/Documentos/Programacion/ROS 2/Teleoperacion con dron"
./start_ros_system.sh
```

**Opción B - Manual:**
```bash
# Configurar entorno
cd "/mnt/d/Documentos/Programacion/ROS 2/Teleoperacion con dron"
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Iniciar controlador del dron
ros2 run drone_controller drone_controller
```

### **PASO 3: Controlar el Dron**

**Desde la interfaz web:**
- Usar joystick virtual para controlar el dron
- Botones de armado/desarmado
- Monitoreo de telemetría en tiempo real

**Desde terminal (WSL):**
```bash
# Armar el dron
ros2 topic pub --once /drone/arm std_msgs/Bool "{data: true}"

# Controlar movimiento
ros2 topic pub --once /drone/control geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}"

# Ver telemetría
ros2 topic echo /drone/telemetry
```

## 📊 QoS y Configuración 5G

El proyecto incluye configuraciones QoS optimizadas para:
- **Baja latencia**: < 10ms para control crítico
- **Alta confiabilidad**: Mensajes garantizados
- **Ancho de banda eficiente**: Compresión de datos

## 🔧 Métodos Alternativos

### **Método A: Sistema Completo con Launch Files**

```bash
# En WSL - Sistema completo automático
ros2 launch launch/full_system.launch.py
```

### **Método B: Componentes Individuales**

**Terminal 1 (WSL) - Controlador:**
```bash
ros2 run drone_controller drone_controller
```

**Terminal 2 (WSL) - Simulador de Red:**
```bash
ros2 run network_simulator network_simulator
```

**Terminal 3 (WSL) - Servidor WebSocket:**
```bash
ros2 run remote_interface web_server
```

### **Método C: Acceso Directo a Archivos**

Si hay problemas de conectividad:
1. Abre directamente: `install\remote_interface\share\remote_interface\web\index.html`
2. O usa: `demo_interface.html` para una versión offline

## 🛠️ Solución de Problemas

### **No se conecta la interfaz web:**
```powershell
# Windows - Verificar puerto
netstat -an | findstr 8080

# Usar servidor alternativo
python -m http.server 8080 --bind 0.0.0.0
```

### **Error "conexión rechazada":**
```bash
# WSL - Verificar nodos ROS
ros2 node list

# Reiniciar servicios
pkill -f "ros2\|drone\|web"
ros2 run drone_controller drone_controller
```

### **WSL no conecta con Windows:**
```powershell
# PowerShell como Administrador
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=172.30.159.225
```

## 📊 Comandos Útiles

```bash
# Ver estado del sistema
ros2 node list
ros2 topic list | grep drone

# Verificar telemetría
ros2 topic hz /drone/telemetry
ros2 topic echo /drone/telemetry --once

# Control manual
ros2 topic pub --once /drone/arm std_msgs/Bool "{data: true}"
ros2 topic pub /drone/control geometry_msgs/Twist "{linear: {x: 1.0}}" --rate 10
```

## 📝 Licencia

MIT License - ver LICENSE para detalles.

## 🤝 Contribuciones

¡Las contribuciones son bienvenidas! Por favor lee CONTRIBUTING.md antes de enviar un PR. 