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

echo -e "${BLUE}🚁 Iniciando Controlador del Dron...${NC}"
echo -e "${YELLOW}📊 El controlador publicará telemetría en /drone/telemetry${NC}"
echo -e "${YELLOW}🎮 Para armar el dron: ros2 topic pub --once /drone/arm std_msgs/Bool \"{data: true}\"${NC}"
echo

# Iniciar controlador del dron
ros2 run drone_controller drone_controller
