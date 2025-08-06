# Teleoperación de Dron con ROS 2 y 5G

Este proyecto implementa un sistema de teleoperación de dron usando ROS 2 con simulación de red 5G para control remoto.

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

- ROS 2 Humble o superior
- Python 3.8+
- Node.js 16+ (para interfaz web)
- Navegador web moderno

## 🚀 Instalación

1. **Clonar el repositorio:**
```bash
git clone <repository-url>
cd drone_teleop_5g
```

2. **Instalar dependencias:**
```bash
./scripts/install_dependencies.sh
```

3. **Compilar el workspace:**
```bash
colcon build
source install/setup.bash
```

## 🎮 Uso

1. **Lanzar el sistema completo:**
```bash
ros2 launch drone_teleop_5g full_system.launch.py
```

2. **Abrir interfaz web:**
   - Navegar a `http://localhost:8080`
   - Usar el joystick virtual para controlar el dron

3. **Monitorear telemetría:**
```bash
ros2 topic echo /drone/telemetry
```

## 📊 QoS y Configuración 5G

El proyecto incluye configuraciones QoS optimizadas para:
- **Baja latencia**: < 10ms para control crítico
- **Alta confiabilidad**: Mensajes garantizados
- **Ancho de banda eficiente**: Compresión de datos

## 🔧 Configuración Avanzada

Ver `docs/advanced_configuration.md` para:
- Configuración de hardware real
- Optimización de red
- Personalización de interfaz

## 📝 Licencia

MIT License - ver LICENSE para detalles.

## 🤝 Contribuciones

¡Las contribuciones son bienvenidas! Por favor lee CONTRIBUTING.md antes de enviar un PR. 