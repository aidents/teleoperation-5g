#!/bin/bash

# Script de instalación de dependencias para Teleoperación de Dron con ROS 2 y 5G
# Autor: Tu Nombre
# Fecha: 2024

set -e  # Salir en caso de error

echo "🚁 Instalando dependencias para Teleoperación de Dron con ROS 2 y 5G"
echo "================================================================"

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Función para imprimir mensajes
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Verificar sistema operativo
check_os() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        OS="linux"
        print_status "Sistema operativo detectado: Linux"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS="macos"
        print_status "Sistema operativo detectado: macOS"
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        OS="windows"
        print_status "Sistema operativo detectado: Windows"
    else
        print_error "Sistema operativo no soportado: $OSTYPE"
        exit 1
    fi
}

# Verificar si ROS 2 está instalado
check_ros2() {
    print_status "Verificando instalación de ROS 2..."
    
    if command -v ros2 &> /dev/null; then
        ROS2_VERSION=$(ros2 --version 2>/dev/null | head -n1 | cut -d' ' -f2)
        print_success "ROS 2 detectado: $ROS2_VERSION"
    else
        print_error "ROS 2 no está instalado. Por favor instala ROS 2 Humble o superior."
        print_status "Instrucciones: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
}

# Instalar dependencias de Python
install_python_deps() {
    print_status "Instalando dependencias de Python..."
    
    # Crear entorno virtual si no existe
    if [ ! -d "venv" ]; then
        print_status "Creando entorno virtual Python..."
        python3 -m venv venv
    fi
    
    # Activar entorno virtual
    source venv/bin/activate
    
    # Actualizar pip
    pip install --upgrade pip
    
    # Instalar dependencias
    pip install -r requirements.txt
    
    print_success "Dependencias de Python instaladas"
}

# Instalar dependencias del sistema (Linux)
install_system_deps_linux() {
    if [ "$OS" = "linux" ]; then
        print_status "Instalando dependencias del sistema (Linux)..."
        
        # Detectar distribución
        if command -v apt &> /dev/null; then
            # Ubuntu/Debian
            sudo apt update
            sudo apt install -y \
                python3-pip \
                python3-venv \
                python3-dev \
                build-essential \
                cmake \
                git \
                curl \
                wget \
                nodejs \
                npm \
                libssl-dev \
                libffi-dev
                
        elif command -v dnf &> /dev/null; then
            # Fedora
            sudo dnf install -y \
                python3-pip \
                python3-devel \
                gcc \
                gcc-c++ \
                cmake \
                git \
                curl \
                wget \
                nodejs \
                npm \
                openssl-devel \
                libffi-devel
                
        elif command -v pacman &> /dev/null; then
            # Arch Linux
            sudo pacman -S --noconfirm \
                python-pip \
                python-virtualenv \
                base-devel \
                cmake \
                git \
                curl \
                wget \
                nodejs \
                npm \
                openssl \
                libffi
        else
            print_warning "Distribución no reconocida. Instala manualmente las dependencias."
        fi
        
        print_success "Dependencias del sistema instaladas"
    fi
}

# Instalar dependencias del sistema (macOS)
install_system_deps_macos() {
    if [ "$OS" = "macos" ]; then
        print_status "Instalando dependencias del sistema (macOS)..."
        
        # Verificar si Homebrew está instalado
        if ! command -v brew &> /dev/null; then
            print_status "Instalando Homebrew..."
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        fi
        
        # Instalar dependencias
        brew install \
            python3 \
            cmake \
            git \
            curl \
            wget \
            node \
            openssl \
            libffi
            
        print_success "Dependencias del sistema instaladas"
    fi
}

# Instalar dependencias del sistema (Windows)
install_system_deps_windows() {
    if [ "$OS" = "windows" ]; then
        print_status "Instalando dependencias del sistema (Windows)..."
        
        # Verificar si Chocolatey está instalado
        if ! command -v choco &> /dev/null; then
            print_status "Instalando Chocolatey..."
            print_warning "Por favor ejecuta PowerShell como administrador y ejecuta:"
            echo "Set-ExecutionPolicy Bypass -Scope Process -Force"
            echo "[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072"
            echo "iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"
            print_warning "Después de instalar Chocolatey, ejecuta:"
            echo "choco install -y python3 cmake git curl wget nodejs openssl visualstudio2019buildtools"
        else
            print_status "Chocolatey detectado. Instalando dependencias..."
            choco install -y python3 cmake git curl wget nodejs openssl visualstudio2019buildtools
        fi
        
        print_success "Dependencias del sistema instaladas"
    fi
}

# Instalar dependencias de Node.js
install_node_deps() {
    print_status "Instalando dependencias de Node.js..."
    
    # Verificar si Node.js está instalado
    if ! command -v node &> /dev/null; then
        print_error "Node.js no está instalado. Por favor instálalo primero."
        exit 1
    fi
    
    # Instalar dependencias globales
    npm install -g \
        http-server \
        websocket-server \
        nodemon
        
    print_success "Dependencias de Node.js instaladas"
}

# Configurar QoS para 5G
setup_qos_config() {
    print_status "Configurando QoS para simulación 5G..."
    
    # Crear directorio de configuración si no existe
    mkdir -p config
    
    # Crear archivo de configuración QoS
    cat > config/qos_5g.yaml << 'EOF'
# Configuración QoS para simulación de red 5G
qos_profiles:
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

network_simulation:
  latency_ms: 5.0
  jitter_ms: 1.0
  packet_loss_percent: 0.1
  bandwidth_mbps: 1000.0
  connection_timeout_ms: 1000
EOF

    print_success "Configuración QoS creada"
}

# Crear archivo requirements.txt
create_requirements() {
    print_status "Creando archivo requirements.txt..."
    
    cat > requirements.txt << 'EOF'
# Dependencias para Teleoperación de Dron con ROS 2 y 5G

# ROS 2 Python
rclpy>=0.10.0
geometry_msgs
sensor_msgs
nav_msgs
std_msgs
tf2_ros
tf2_geometry_msgs

# WebSocket y servidor web
websockets>=10.0
aiohttp>=3.8.0
fastapi>=0.68.0
uvicorn>=0.15.0

# Utilidades
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.5.0
pandas>=1.3.0

# Desarrollo y testing
pytest>=6.0.0
pytest-asyncio>=0.15.0
black>=21.0.0
flake8>=3.9.0

# Simulación de red
netem>=0.1.0
tc>=0.1.0
EOF

    print_success "Archivo requirements.txt creado"
}

# Configurar entorno de desarrollo
setup_dev_environment() {
    print_status "Configurando entorno de desarrollo..."
    
    # Crear archivo .env
    cat > .env << 'EOF'
# Configuración del entorno de desarrollo
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Configuración del servidor web
WEB_PORT=8080
WEBSOCKET_PORT=8081
WEB_HOST=0.0.0.0

# Configuración de simulación 5G
NETWORK_LATENCY_MS=5.0
NETWORK_JITTER_MS=1.0
NETWORK_PACKET_LOSS=0.1
NETWORK_BANDWIDTH_MBPS=1000.0

# Configuración del dron
DRONE_MAX_VELOCITY=5.0
DRONE_MAX_ANGULAR_VELOCITY=2.0
DRONE_CONTROL_RATE=50.0
DRONE_BATTERY_DRAIN_RATE=0.01
EOF

    print_success "Archivo .env creado"
}

# Función principal
main() {
    echo "🚁 Iniciando instalación de dependencias..."
    echo ""
    
    # Verificar sistema operativo
    check_os
    
    # Verificar ROS 2
    check_ros2
    
    # Instalar dependencias del sistema
    install_system_deps_linux
    install_system_deps_macos
    install_system_deps_windows
    
    # Instalar dependencias de Node.js
    install_node_deps
    
    # Crear archivos de configuración
    create_requirements
    setup_qos_config
    setup_dev_environment
    
    # Instalar dependencias de Python
    install_python_deps
    
    echo ""
    echo "🎉 ¡Instalación completada exitosamente!"
    echo ""
    echo "📋 Próximos pasos:"
    echo "1. Activa el entorno virtual: source venv/bin/activate"
    echo "2. Compila el workspace: colcon build"
    echo "3. Configura el entorno: source install/setup.bash"
    echo "4. Lanza el sistema: ros2 launch drone_teleop_5g full_system.launch.py"
    echo ""
    echo "🌐 La interfaz web estará disponible en: http://localhost:8080"
    echo ""
}

# Ejecutar función principal
main "$@" 