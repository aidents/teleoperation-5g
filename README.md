# Teleoperación de Dron con ROS 2 y 5G

Este proyecto implementa un sistema de teleoperación de dron usando ROS 2 con simulación de red 5G para control remoto.

## ⚡ Inicio Rápido

**¿Solo quieres ver el sistema funcionando?**

### **Opción 1: Solo Interfaz Web (Windows)**

1. **Windows**: Ejecuta `start_web.bat` → Se abre la interfaz web moderna
2. **Listo**: ¡Ya puedes ver la interfaz en http://localhost:3000!

### **Opción 2: Sistema Completo (RECOMENDADO)**

1. **Windows**: Ejecuta `start_system.bat` → Inicia el sistema ROS 2 completo en WSL
2. **WSL**: Ejecuta `./start_ros_system.sh` → Inicia el sistema ROS 2 completo
3. **Listo**: ¡Ya puedes controlar el dron desde http://localhost:3000!

### **Opción 3: Componentes Individuales**

1. **Windows**: `start_web.bat` → Solo interfaz web
2. **WSL**: `./start_ros_system.sh` → Solo sistema ROS 2
3. **Manual**: `python start_web.py` → Interfaz web manual

## 🚁 Características

- **Control remoto de dron**: Interfaz web moderna con Next.js para controlar el dron desde cualquier dispositivo
- **Simulación de red 5G**: Configuración de QoS para simular latencia y ancho de banda de red 5G
- **ROS 2 Topics y Servicios**: Comunicación robusta entre componentes
- **Interfaz web responsive**: Control intuitivo con joystick virtual y botones
- **Monitoreo en tiempo real**: Visualización de telemetría del dron
- **Arquitectura moderna**: Next.js 15 + TypeScript + Tailwind CSS
- **Bridge ROS2-Web**: Comunicación en tiempo real entre ROS2 y la interfaz web

## 📁 Estructura del Proyecto

```
drone_teleop_5g/
├── src/
│   ├── drone_controller/          # Controlador del dron
│   ├── remote_interface/          # Interfaz web remota moderna
│   │   └── web/                   # Aplicación Next.js + TypeScript
│   ├── network_simulator/         # Simulador de red 5G
│   └── drone_simulator/           # Simulador del dron
├── config/                        # Configuraciones QoS y red
├── launch/                        # Archivos de lanzamiento
├── docs/                          # Documentación
└── scripts/                       # Scripts de instalación y utilidades
```

## 🛠️ Requisitos

- **ROS 2 Rolling** (instalado en WSL Ubuntu)
- **Node.js 18+** (para la interfaz web moderna)
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

### **PASO 1: Iniciar Interfaz Web Moderna (Windows)**

**Opción A - Script Simple (RECOMENDADO):**
```cmd
# Doble clic en el archivo o ejecutar desde CMD:
start_web.bat
```

**Opción B - Script Python (Alternativa):**
```cmd
# Ejecutar desde CMD:
python start_web.py
```

**Opción C - Desde Git Bash:**
```bash
# Si estás usando Git Bash:
./start_web_gitbash.sh
```

**Opción D - Manual:**
```cmd
cd "src\remote_interface\web"
npm install --legacy-peer-deps
npm run dev
```

**Opción E - Manual desde Git Bash:**
```bash
# Navegar al directorio web
cd "src/remote_interface/web"

# Instalar dependencias (solo la primera vez)
npm install --legacy-peer-deps

# Iniciar servidor de desarrollo
npm run dev
```

**Resultado:** Se abrirá automáticamente **http://localhost:3000**

### **PASO 2: Iniciar Sistema ROS 2 (WSL)**

**Opción A - Script Automático (RECOMENDADO):**
```cmd
# Doble clic en el archivo o ejecutar desde CMD:
start_system.bat
```

**Opción B - Script Python (Alternativa):**
```cmd
# Ejecutar desde CMD:
python start_system.py
```

**Opción C - Desde Git Bash:**
```bash
# Si estás usando Git Bash:
./start_system_gitbash.sh
```

**Opción D - Script Bash Directo en WSL:**
```bash
# En terminal WSL Ubuntu
cd "/mnt/d/Documentos/Programacion/ROS 2/Teleoperacion con dron"
./start_ros_system.sh
```

**Opción E - Manual en WSL:**
```bash
# Configurar entorno
cd "/mnt/d/Documentos/Programacion/ROS 2/Teleoperacion con dron"
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Iniciar controlador del dron
ros2 run drone_controller drone_controller

# En otra terminal, iniciar bridge ROS2-Web
ros2 run remote_interface ros2_web_bridge
```

## 🔌 Integración ROS2-Web

El sistema ahora incluye un **bridge ROS2-Web** que permite:

- **Comunicación en tiempo real** entre ROS2 y la interfaz web
- **Telemetría en vivo** del dron (posición, velocidad, batería)
- **Control remoto** desde la interfaz web hacia ROS2
- **Monitoreo de latencia** de la red 5G simulada
- **Estado del sistema** sincronizado entre componentes

### **Flujo de Datos:**
```
[Interfaz Web] ←→ [Bridge ROS2-Web] ←→ [Sistema ROS2]
    3000             8080                    ROS2 Topics
```

## 🌐 Interfaz Web Moderna

La nueva interfaz web utiliza tecnologías modernas:

- **Next.js 15** - Framework de React para aplicaciones web
- **TypeScript** - Tipado estático para código robusto
- **Tailwind CSS** - Framework de CSS utilitario
- **Radix UI** - Componentes de interfaz accesibles
- **React 19** - Última versión de React

### **Características de la Interfaz:**
- 🎨 **Diseño Moderno** y profesional
- 📱 **Completamente Responsive** para todos los dispositivos
- 🧩 **Arquitectura Modular** y escalable
- 🔒 **TypeScript** para mayor robustez
- ⚡ **Rendimiento optimizado** con Next.js
- 🔌 **Integración ROS2** en tiempo real

## 🔧 Comandos Útiles

### **Scripts de Inicio (Windows):**
```cmd
# Solo interfaz web
start_web.bat

# Sistema ROS 2 completo en WSL
start_system.bat
```

### **Scripts de Inicio (Git Bash):**
```bash
# Solo interfaz web
./start_web_gitbash.sh

# Sistema ROS 2 completo en WSL
./start_system_gitbash.sh
```

### **Scripts de Inicio (WSL/Linux):**
```bash
# Sistema ROS 2 completo
./start_ros_system.sh
```

### **Interfaz Web:**
```bash
# Desarrollo
npm run dev

# Construir para producción
npm run build

# Servidor de producción
npm run start

# Verificar código
npm run lint
```

### **Sistema ROS 2:**
```bash
# Ver nodos activos
ros2 node list

# Ver topics
ros2 topic list

# Ver telemetría del dron
ros2 topic echo /drone/telemetry

# Armar el dron
ros2 topic pub --once /drone/arm std_msgs/Bool "{data: true}"

# Ver bridge ROS2-Web
ros2 node info /ros2_web_bridge
```

## 📊 Monitoreo y Control

### **Telemetría en Tiempo Real:**
- **Posición**: X, Y, Z en metros
- **Velocidad**: VX, VY, VZ en m/s
- **Orientación**: Roll, Pitch, Yaw en grados
- **Estado**: Armado/Desarmado, Modo de vuelo
- **Batería**: Nivel de carga en porcentaje
- **Latencia de red**: Tiempo de respuesta de la red 5G

### **Controles Disponibles:**
- **Joystick virtual**: Control de movimiento horizontal (adelante/atrás, izquierda/derecha)
- **Slider de altura**: Control de movimiento vertical
- **Slider de rotación**: Control de giro (yaw)
- **Botones de control**: Armar/Desarmar, Despegar/Aterrizar
- **Botón de emergencia**: Parada de emergencia

## 🚨 Solución de Problemas

### **Interfaz Web no se abre:**
1. Verifica que Node.js esté instalado
2. Asegúrate de usar `--legacy-peer-deps` al instalar
3. Verifica que el puerto 3000 esté disponible
4. Revisa la consola del navegador para errores

### **Sistema ROS 2 no responde:**
1. Verifica que WSL esté ejecutándose
2. Confirma que ROS 2 esté instalado
3. Verifica que el workspace esté compilado
4. Revisa los logs de error

### **Bridge ROS2-Web no conecta:**
1. Verifica que el puerto 8080 esté disponible
2. Confirma que el nodo `ros2_web_bridge` esté ejecutándose
3. Revisa los logs de ROS2 para errores
4. Verifica que la interfaz web esté en puerto 3000

### **No hay comunicación en tiempo real:**
1. Verifica que ambos componentes estén ejecutándose
2. Confirma que el WebSocket esté conectado (consola del navegador)
3. Verifica que los topics ROS2 estén publicando datos
4. Revisa la configuración de QoS en el bridge

## 📚 Documentación Adicional

- **README_NEXTJS_INTERFACE.md** - Documentación completa de la interfaz web
- **docs/** - Documentación técnica del proyecto
- **launch/** - Archivos de configuración de lanzamiento

## 🤝 Contribuir

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📄 Licencia

Este proyecto está bajo la Licencia MIT. Ver el archivo `LICENSE` para más detalles.

## 📞 Soporte

Si tienes problemas o preguntas:
1. Revisa la documentación en `docs/`
2. Verifica los logs de error
3. Abre un issue en el repositorio
4. Contacta al equipo de desarrollo

---

**¡Disfruta controlando tu dron con la nueva interfaz moderna integrada con ROS2!** 🚁✨ 