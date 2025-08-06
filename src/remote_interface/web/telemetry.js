class TelemetryManager {
    constructor() {
        this.currentTelemetry = {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
            velocity: { x: 0, y: 0, z: 0 },
            battery: 100,
            armed: false
        };
        
        this.updateInterval = null;
        this.smoothUpdate = true;
        
        this.init();
    }
    
    init() {
        // Inicializar elementos de la interfaz
        this.updateAllDisplays();
        
        // Configurar actualización suave
        if (this.smoothUpdate) {
            this.updateInterval = setInterval(() => {
                this.updateAllDisplays();
            }, 50); // 20 FPS para actualización suave
        }
    }
    
    updateTelemetry(newTelemetry) {
        // Actualizar telemetría con interpolación suave
        if (this.smoothUpdate) {
            this.interpolateTelemetry(newTelemetry);
        } else {
            this.currentTelemetry = { ...newTelemetry };
            this.updateAllDisplays();
        }
    }
    
    interpolateTelemetry(newTelemetry) {
        const interpolationFactor = 0.3; // Factor de suavizado
        
        // Interpolar posición
        this.currentTelemetry.position.x += (newTelemetry.position.x - this.currentTelemetry.position.x) * interpolationFactor;
        this.currentTelemetry.position.y += (newTelemetry.position.y - this.currentTelemetry.position.y) * interpolationFactor;
        this.currentTelemetry.position.z += (newTelemetry.position.z - this.currentTelemetry.position.z) * interpolationFactor;
        
        // Interpolar velocidad
        this.currentTelemetry.velocity.x += (newTelemetry.velocity.x - this.currentTelemetry.velocity.x) * interpolationFactor;
        this.currentTelemetry.velocity.y += (newTelemetry.velocity.y - this.currentTelemetry.velocity.y) * interpolationFactor;
        this.currentTelemetry.velocity.z += (newTelemetry.velocity.z - this.currentTelemetry.velocity.z) * interpolationFactor;
        
        // Actualizar valores inmediatos
        this.currentTelemetry.battery = newTelemetry.battery;
        this.currentTelemetry.armed = newTelemetry.armed;
        
        // Interpolar orientación (cuaterniones)
        this.interpolateQuaternion(newTelemetry.orientation);
    }
    
    interpolateQuaternion(newOrientation) {
        const current = this.currentTelemetry.orientation;
        const target = newOrientation;
        const factor = 0.3;
        
        // Interpolación lineal de cuaterniones (SLERP simplificado)
        const dot = current.x * target.x + current.y * target.y + current.z * target.z + current.w * target.w;
        
        if (dot > 0.9995) {
            // Cuaterniones muy similares, usar interpolación lineal
            current.x += (target.x - current.x) * factor;
            current.y += (target.y - current.y) * factor;
            current.z += (target.z - current.z) * factor;
            current.w += (target.w - current.w) * factor;
        } else {
            // Usar interpolación esférica
            const angle = Math.acos(Math.abs(dot));
            const sinAngle = Math.sin(angle);
            
            if (sinAngle > 0.001) {
                const invSinAngle = 1.0 / sinAngle;
                const coeff1 = Math.sin((1.0 - factor) * angle) * invSinAngle;
                const coeff2 = Math.sin(factor * angle) * invSinAngle;
                
                if (dot < 0) {
                    coeff2 = -coeff2;
                }
                
                current.x = coeff1 * current.x + coeff2 * target.x;
                current.y = coeff1 * current.y + coeff2 * target.y;
                current.z = coeff1 * current.z + coeff2 * target.z;
                current.w = coeff1 * current.w + coeff2 * target.w;
            }
        }
        
        // Normalizar cuaternión
        const length = Math.sqrt(current.x * current.x + current.y * current.y + current.z * current.z + current.w * current.w);
        if (length > 0.001) {
            current.x /= length;
            current.y /= length;
            current.z /= length;
            current.w /= length;
        }
    }
    
    updateAllDisplays() {
        this.updatePositionDisplay();
        this.updateVelocityDisplay();
        this.updateBatteryDisplay();
        this.updateStatusDisplay();
    }
    
    updatePositionDisplay() {
        const posX = document.getElementById('posX');
        const posY = document.getElementById('posY');
        const posZ = document.getElementById('posZ');
        
        if (posX) posX.textContent = this.currentTelemetry.position.x.toFixed(2);
        if (posY) posY.textContent = this.currentTelemetry.position.y.toFixed(2);
        if (posZ) posZ.textContent = this.currentTelemetry.position.z.toFixed(2);
    }
    
    updateVelocityDisplay() {
        const velX = document.getElementById('velX');
        const velY = document.getElementById('velY');
        const velZ = document.getElementById('velZ');
        
        if (velX) velX.textContent = this.currentTelemetry.velocity.x.toFixed(2);
        if (velY) velY.textContent = this.currentTelemetry.velocity.y.toFixed(2);
        if (velZ) velZ.textContent = this.currentTelemetry.velocity.z.toFixed(2);
    }
    
    updateBatteryDisplay() {
        const batteryLevel = document.getElementById('batteryLevel');
        const batteryPercentage = document.getElementById('batteryPercentage');
        
        if (batteryLevel) {
            batteryLevel.style.width = `${this.currentTelemetry.battery}%`;
            
            // Cambiar color según nivel de batería
            if (this.currentTelemetry.battery > 50) {
                batteryLevel.style.background = 'linear-gradient(90deg, #27ae60, #f39c12)';
            } else if (this.currentTelemetry.battery > 20) {
                batteryLevel.style.background = 'linear-gradient(90deg, #f39c12, #e74c3c)';
            } else {
                batteryLevel.style.background = '#e74c3c';
            }
        }
        
        if (batteryPercentage) {
            batteryPercentage.textContent = `${Math.round(this.currentTelemetry.battery)}%`;
        }
    }
    
    updateStatusDisplay() {
        const armedStatus = document.getElementById('armedStatus');
        const flightMode = document.getElementById('flightMode');
        
        if (armedStatus) {
            armedStatus.textContent = this.currentTelemetry.armed ? 'Sí' : 'No';
            armedStatus.style.color = this.currentTelemetry.armed ? '#27ae60' : '#e74c3c';
        }
        
        if (flightMode) {
            flightMode.textContent = this.currentTelemetry.armed ? 'MANUAL' : 'STANDBY';
        }
    }
    
    // Convertir cuaternión a ángulos de Euler (para visualización)
    quaternionToEuler(q) {
        const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        const pitch = Math.asin(2 * (q.w * q.y - q.z * q.x));
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        
        return {
            roll: roll * 180 / Math.PI,
            pitch: pitch * 180 / Math.PI,
            yaw: yaw * 180 / Math.PI
        };
    }
    
    // Obtener telemetría actual
    getCurrentTelemetry() {
        return { ...this.currentTelemetry };
    }
    
    // Obtener estadísticas de telemetría
    getTelemetryStats() {
        const velocity = Math.sqrt(
            this.currentTelemetry.velocity.x ** 2 +
            this.currentTelemetry.velocity.y ** 2 +
            this.currentTelemetry.velocity.z ** 2
        );
        
        const altitude = this.currentTelemetry.position.z;
        const distance = Math.sqrt(
            this.currentTelemetry.position.x ** 2 +
            this.currentTelemetry.position.y ** 2
        );
        
        return {
            velocity: velocity,
            altitude: altitude,
            distance: distance,
            battery: this.currentTelemetry.battery,
            armed: this.currentTelemetry.armed
        };
    }
    
    // Configurar modo de actualización
    setSmoothUpdate(enabled) {
        this.smoothUpdate = enabled;
        
        if (enabled && !this.updateInterval) {
            this.updateInterval = setInterval(() => {
                this.updateAllDisplays();
            }, 50);
        } else if (!enabled && this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    }
    
    // Limpiar recursos
    destroy() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    }
}

// Inicializar telemetría cuando el DOM esté listo
document.addEventListener('DOMContentLoaded', () => {
    window.telemetryManager = new TelemetryManager();
}); 