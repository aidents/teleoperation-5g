# Guía Técnica - Teleoperación de Dron con ROS 2 y 5G

## 📋 Índice

1. [Arquitectura del Sistema](#arquitectura-del-sistema)
2. [Componentes Principales](#componentes-principales)
3. [Configuración QoS para 5G](#configuración-qos-para-5g)
4. [Protocolos de Comunicación](#protocolos-de-comunicación)
5. [Interfaz Web](#interfaz-web)
6. [Simulación del Dron](#simulación-del-dron)
7. [Optimización de Rendimiento](#optimización-de-rendimiento)
8. [Troubleshooting](#troubleshooting)

## 🏗️ Arquitectura del Sistema

### Diagrama de Arquitectura

```
┌─────────────────┐    WebSocket    ┌─────────────────┐
│   Interfaz Web  │ ◄─────────────► │  Servidor Web   │
│   (Cliente)     │                 │   (ROS Node)    │
└─────────────────┘                 └─────────────────┘
                                              │
                                              │ ROS Topics
                                              ▼
┌─────────────────┐                 ┌─────────────────┐
│  Simulador de   │ ◄─────────────► │  Controlador    │
│   Red 5G        │                 │    del Dron     │
└─────────────────┘                 └─────────────────┘
                                              │
                                              │ TF
                                              ▼
                                    ┌─────────────────┐
                                    │   Simulador     │
                                    │    del Dron     │
                                    └─────────────────┘
```

### Flujo de Datos

1. **Control Remoto**: El usuario interactúa con la interfaz web
2. **WebSocket**: Los comandos se envían al servidor web via WebSocket
3. **ROS Topics**: El servidor web publica en topics ROS
4. **Simulación de Red**: Los mensajes pasan por el simulador de red 5G
5. **Controlador**: El controlador del dron procesa los comandos
6. **Telemetría**: El estado del dron se publica de vuelta
7. **Interfaz**: La telemetría se muestra en tiempo real

## 🔧 Componentes Principales

### 1. Controlador del Dron (`drone_controller`)

**Responsabilidades:**
- Procesar comandos de control
- Simular física del dron
- Publicar telemetría
- Manejar estados del dron (armado/desarmado)

**Topics Principales:**
- `/drone/control` (Twist) - Comandos de control
- `/drone/telemetry` (Odometry) - Telemetría del dron
- `/drone/imu` (Imu) - Datos del IMU
- `/drone/battery` (BatteryState) - Estado de batería
- `/drone/arm` (Bool) - Comando de armado

**Parámetros Configurables:**
```yaml
max_velocity: 5.0              # Velocidad máxima (m/s)
max_angular_velocity: 2.0      # Velocidad angular máxima (rad/s)
control_rate: 50.0             # Frecuencia de control (Hz)
battery_drain_rate: 0.01       # Tasa de consumo de batería
```

### 2. Interfaz Remota (`remote_interface`)

**Responsabilidades:**
- Servir interfaz web
- Manejar conexiones WebSocket
- Convertir comandos web a ROS
- Mostrar telemetría en tiempo real

**Características:**
- Interfaz responsive
- Joystick virtual
- Controles de teclado
- Monitoreo de latencia
- Visualización de telemetría

### 3. Simulador de Red 5G (`network_simulator`)

**Responsabilidades:**
- Simular latencia de red
- Simular pérdida de paquetes
- Simular jitter
- Simular ancho de banda limitado

**Parámetros de Simulación:**
```yaml
latency_ms: 5.0                # Latencia de red (ms)
jitter_ms: 1.0                 # Variación de latencia (ms)
packet_loss_percent: 0.1       # Pérdida de paquetes (%)
bandwidth_mbps: 1000.0         # Ancho de banda (Mbps)
```

## ⚡ Configuración QoS para 5G

### Perfiles QoS

#### 1. Control Crítico
```yaml
control:
  reliability: BEST_EFFORT
  durability: VOLATILE
  depth: 10
  deadline:
    sec: 0
    nanosec: 10000000  # 10ms
  liveliness:
    policy: AUTOMATIC
    lease_duration:
      sec: 1
      nanosec: 0
```

**Uso:** Comandos de control del dron
**Prioridad:** Máxima
**Latencia Objetivo:** < 10ms

#### 2. Telemetría
```yaml
telemetry:
  reliability: RELIABLE
  durability: VOLATILE
  depth: 100
  deadline:
    sec: 0
    nanosec: 50000000  # 50ms
  liveliness:
    policy: AUTOMATIC
    lease_duration:
      sec: 2
      nanosec: 0
```

**Uso:** Datos de telemetría
**Prioridad:** Media
**Latencia Objetivo:** < 50ms

#### 3. Emergencia
```yaml
emergency:
  reliability: RELIABLE
  durability: TRANSIENT_LOCAL
  depth: 1
  deadline:
    sec: 0
    nanosec: 1000000   # 1ms
  liveliness:
    policy: MANUAL_BY_TOPIC
    lease_duration:
      sec: 0
      nanosec: 100000000  # 100ms
```

**Uso:** Comandos de emergencia
**Prioridad:** Crítica
**Latencia Objetivo:** < 1ms

## 🌐 Protocolos de Comunicación

### WebSocket Messages

#### Comandos de Control
```json
{
  "type": "control",
  "data": {
    "forward": 0.5,    // -1.0 a 1.0
    "right": 0.0,      // -1.0 a 1.0
    "up": 0.3,         // -1.0 a 1.0
    "yaw": 0.0         // -1.0 a 1.0
  }
}
```

#### Comando de Armado
```json
{
  "type": "arm",
  "data": true
}
```

#### Ping/Pong
```json
{
  "type": "ping",
  "timestamp": 1640995200000
}
```

#### Telemetría
```json
{
  "type": "telemetry",
  "data": {
    "position": {
      "x": 1.5,
      "y": 0.0,
      "z": 2.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    },
    "velocity": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "battery": 85.5,
    "armed": true
  }
}
```

## 🎮 Interfaz Web

### Características Técnicas

#### Joystick Virtual
- **Tecnología:** HTML5 Canvas + JavaScript
- **Resolución:** 200x200 píxeles
- **Zona muerta:** 5 píxeles
- **Soporte:** Mouse y Touch
- **Frecuencia de actualización:** 50Hz

#### Controles de Teclado
```
W/↑     - Adelante
S/↓     - Atrás
A/←     - Izquierda
D/→     - Derecha
Q       - Rotación izquierda
E       - Rotación derecha
R       - Subir
F       - Bajar
ESPACIO - Parada de emergencia
```

#### Monitoreo de Latencia
- **Método:** Ping/Pong con timestamp
- **Historial:** Últimos 50 valores
- **Visualización:** Gráfico en tiempo real
- **Métricas:** Promedio, máximo, mínimo

### Responsive Design

#### Breakpoints
```css
/* Desktop */
@media (min-width: 1024px) {
  .main-content {
    grid-template-columns: 1fr 1fr;
  }
}

/* Tablet */
@media (max-width: 1023px) {
  .main-content {
    grid-template-columns: 1fr;
  }
}

/* Mobile */
@media (max-width: 767px) {
  .joystick {
    width: 150px;
    height: 150px;
  }
}
```

## 🚁 Simulación del Dron

### Modelo Físico Simplificado

#### Cinemática
```python
# Actualización de posición
dt = 1.0 / control_rate
position[0] += velocity[0] * dt
position[1] += velocity[1] * dt
position[2] += velocity[2] * dt

# Actualización de orientación
orientation[0] += angular_velocity[0] * dt
orientation[1] += angular_velocity[1] * dt
orientation[2] += angular_velocity[2] * dt
```

#### Consumo de Batería
```python
# Consumo proporcional a la velocidad
velocity_magnitude = sqrt(sum(v**2 for v in velocity))
battery -= velocity_magnitude * drain_rate * dt
```

#### Límites de Seguridad
```python
# Velocidad máxima
velocity = np.clip(velocity, -max_velocity, max_velocity)

# Altura mínima
position[2] = max(position[2], 0.0)

# Batería crítica
if battery < 10.0:
    armed = False
```

### Estados del Dron

1. **STANDBY**: Dron desarmado, esperando comandos
2. **ARMED**: Dron armado, listo para volar
3. **FLYING**: Dron en vuelo activo
4. **EMERGENCY**: Parada de emergencia activada
5. **LANDING**: Secuencia de aterrizaje
6. **ERROR**: Error del sistema

## ⚡ Optimización de Rendimiento

### Optimizaciones de Red

#### Compresión de Mensajes
- **Telemetría:** Reducción de precisión decimal
- **Comandos:** Valores normalizados (-1 a 1)
- **Headers:** Minimización de metadatos

#### Buffering Inteligente
```python
# Buffer circular para telemetría
telemetry_buffer = deque(maxlen=100)

# Envío por lotes
if len(telemetry_buffer) >= batch_size:
    send_batch(telemetry_buffer)
```

#### QoS Adaptativo
```python
# Ajustar QoS según latencia
if current_latency > threshold:
    switch_to_best_effort()
else:
    switch_to_reliable()
```

### Optimizaciones de Interfaz

#### Interpolación Suave
```javascript
// Interpolación lineal para movimiento suave
currentValue += (targetValue - currentValue) * 0.3;
```

#### Throttling de Eventos
```javascript
// Limitar frecuencia de actualización
const throttle = (func, limit) => {
    let inThrottle;
    return function() {
        const args = arguments;
        const context = this;
        if (!inThrottle) {
            func.apply(context, args);
            inThrottle = true;
            setTimeout(() => inThrottle = false, limit);
        }
    }
}
```

#### Lazy Loading
```javascript
// Cargar componentes solo cuando se necesiten
const loadComponent = async (componentName) => {
    if (!loadedComponents.has(componentName)) {
        const module = await import(`./components/${componentName}.js`);
        loadedComponents.set(componentName, module);
    }
    return loadedComponents.get(componentName);
}
```

## 🔧 Troubleshooting

### Problemas Comunes

#### 1. WebSocket No Conecta
**Síntomas:** Interfaz muestra "Desconectado"
**Solución:**
```bash
# Verificar puerto
netstat -tulpn | grep 8081

# Reiniciar servidor web
ros2 node kill /web_server
ros2 run remote_interface web_server
```

#### 2. Alta Latencia
**Síntomas:** Controles lentos, telemetría desactualizada
**Solución:**
```bash
# Verificar configuración QoS
ros2 param get /drone_controller qos_profile

# Ajustar parámetros de red
ros2 param set /network_simulator latency_ms 2.0
```

#### 3. Dron No Responde
**Síntomas:** Comandos no afectan al dron
**Solución:**
```bash
# Verificar estado del dron
ros2 topic echo /drone/telemetry

# Verificar armado
ros2 topic echo /drone/arm

# Reiniciar controlador
ros2 node kill /drone_controller
ros2 run drone_controller drone_controller
```

#### 4. Interfaz Web No Carga
**Síntomas:** Error 404 o página en blanco
**Solución:**
```bash
# Verificar archivos web
ls -la src/remote_interface/web/

# Verificar puerto HTTP
netstat -tulpn | grep 8080

# Reiniciar servidor
pkill -f web_server
ros2 run remote_interface web_server
```

### Logs y Debugging

#### Habilitar Logs Detallados
```bash
# Configurar nivel de log
ros2 run drone_controller drone_controller --ros-args --log-level DEBUG

# Ver logs en tiempo real
ros2 run drone_controller drone_controller --ros-args --log-level DEBUG 2>&1 | tee drone.log
```

#### Monitoreo de Recursos
```bash
# CPU y memoria
htop

# Red
iftop

# Disco
iotop
```

#### Profiling de WebSocket
```javascript
// Habilitar logging detallado
websocketManager.debug = true;

// Monitorear latencia
setInterval(() => {
    console.log('Latencia:', websocketManager.getLatencyStats());
}, 1000);
```

## 📊 Métricas de Rendimiento

### Objetivos de Rendimiento

| Métrica | Objetivo | Actual |
|---------|----------|--------|
| Latencia de Control | < 10ms | 5ms |
| Latencia de Telemetría | < 50ms | 20ms |
| FPS Interfaz | 60 FPS | 60 FPS |
| Uso de CPU | < 20% | 15% |
| Uso de Memoria | < 500MB | 300MB |
| Pérdida de Paquetes | < 0.1% | 0.05% |

### Monitoreo Continuo

#### Script de Monitoreo
```bash
#!/bin/bash
# monitor_performance.sh

while true; do
    echo "=== $(date) ==="
    
    # Latencia WebSocket
    curl -s http://localhost:8080/api/latency
    
    # Uso de CPU
    top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1
    
    # Memoria
    free -m | grep Mem | awk '{print $3"/"$2}'
    
    # Topics ROS
    ros2 topic list | wc -l
    
    sleep 5
done
```

## 🔮 Próximas Mejoras

### Funcionalidades Planificadas

1. **Simulación 3D**: Integración con Gazebo
2. **Machine Learning**: Control adaptativo
3. **Multi-dron**: Control de flota
4. **Realidad Virtual**: Interfaz VR
5. **Autonomía**: Waypoints y navegación automática

### Optimizaciones Técnicas

1. **GPU Acceleration**: Renderizado WebGL
2. **Edge Computing**: Procesamiento distribuido
3. **5G Real**: Integración con hardware 5G
4. **Blockchain**: Logs inmutables
5. **AI/ML**: Predicción de fallos 