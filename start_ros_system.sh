#!/bin/bash

echo "==============================================="
echo "🚁 Iniciando Sistema ROS 2 - Teleoperacion Dron"
echo "==============================================="
echo

# Colores para output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Verificar directorio
if [[ ! -f "install/setup.bash" ]]; then
    echo -e "${RED}❌ Error: No se encontró install/setup.bash${NC}"
    echo "Verifica que estás en el directorio correcto del proyecto"
    exit 1
fi

echo -e "${BLUE}📁 Configurando entorno ROS 2...${NC}"
source /opt/ros/rolling/setup.bash
source install/setup.bash

echo -e "${GREEN}✅ Entorno ROS 2 configurado${NC}"
echo

# Verificar nodos existentes
echo -e "${BLUE}🔍 Verificando nodos activos...${NC}"
EXISTING_NODES=$(ros2 node list 2>/dev/null | wc -l)
if [ $EXISTING_NODES -gt 0 ]; then
    echo -e "${YELLOW}⚠️  Nodos ya ejecutándose:${NC}"
    ros2 node list
    echo
    read -p "¿Deseas detener nodos existentes? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${BLUE}🛑 Deteniendo nodos existentes...${NC}"
        pkill -f "ros2\|drone\|web\|network" 2>/dev/null || true
        sleep 2
    fi
fi

echo -e "${BLUE}🚁 Iniciando Sistema Completo...${NC}"
echo -e "${YELLOW}📊 El controlador publicará telemetría en /drone/telemetry${NC}"
echo -e "${YELLOW}🌐 El bridge ROS2-Web estará disponible en ws://localhost:8080${NC}"
echo -e "${YELLOW}🎮 Para armar el dron: ros2 topic pub --once /drone/arm std_msgs/Bool \"{data: true}\"${NC}"
echo

# Función para limpiar procesos al salir
cleanup() {
    echo -e "\n${YELLOW}🛑 Deteniendo procesos...${NC}"
    pkill -f "ros2 run drone_controller" 2>/dev/null || true
    pkill -f "ros2 run remote_interface" 2>/dev/null || true
    echo -e "${GREEN}✅ Procesos detenidos${NC}"
    exit 0
}

# Configurar trap para cierre limpio
trap cleanup SIGINT SIGTERM

echo -e "${BLUE}🎮 Iniciando Controlador del Dron...${NC}"
# Iniciar controlador del dron en background
ros2 run drone_controller drone_controller &
DRONE_PID=$!

echo -e "${BLUE}🌐 Iniciando Bridge ROS2-Web...${NC}"
# Iniciar bridge ROS2-Web en background
ros2 run remote_interface ros2_web_bridge &
BRIDGE_PID=$!

echo -e "${GREEN}✅ Sistema iniciado correctamente!${NC}"
echo -e "${BLUE}📱 Interfaz web: http://localhost:3000${NC}"
echo -e "${BLUE}🔌 Bridge WebSocket: ws://localhost:8080${NC}"
echo -e "${BLUE}🎮 Controlador: Activo${NC}"
echo
echo -e "${YELLOW}💡 Para detener: Ctrl+C${NC}"
echo -e "${YELLOW}📊 Monitoreando nodos activos...${NC}"
echo "==============================================="

# Monitorear nodos activos
while true; do
    sleep 5
    echo -e "${BLUE}🔄 Nodos activos:${NC}"
    ros2 node list 2>/dev/null || echo "No hay nodos activos"
    echo "---"
done
