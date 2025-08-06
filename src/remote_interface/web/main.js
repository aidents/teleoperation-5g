class ControlSystem {
    constructor() {
        this.isArmed = false;
        this.controlData = {
            forward: 0,
            right: 0,
            up: 0,
            yaw: 0
        };
        
        this.controlInterval = null;
        this.controlRate = 50; // Hz
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.startControlLoop();
    }
    
    setupEventListeners() {
        // Controles de altura y rotación
        const altitudeControl = document.getElementById('altitudeControl');
        const yawControl = document.getElementById('yawControl');
        const altitudeValue = document.getElementById('altitudeValue');
        const yawValue = document.getElementById('yawValue');
        
        if (altitudeControl) {
            altitudeControl.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                this.controlData.up = value;
                if (altitudeValue) altitudeValue.textContent = value.toFixed(1);
            });
        }
        
        if (yawControl) {
            yawControl.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                this.controlData.yaw = value;
                if (yawValue) yawValue.textContent = value.toFixed(1);
            });
        }
        
        // Botones de control
        const armButton = document.getElementById('armButton');
        const emergencyButton = document.getElementById('emergencyButton');
        const takeoffButton = document.getElementById('takeoffButton');
        const landButton = document.getElementById('landButton');
        
        if (armButton) {
            armButton.addEventListener('click', () => {
                this.toggleArm();
            });
        }
        
        if (emergencyButton) {
            emergencyButton.addEventListener('click', () => {
                this.emergencyStop();
            });
        }
        
        if (takeoffButton) {
            takeoffButton.addEventListener('click', () => {
                this.takeoff();
            });
        }
        
        if (landButton) {
            landButton.addEventListener('click', () => {
                this.land();
            });
        }
        
        // Controles de teclado
        this.setupKeyboardControls();
        
        // Prevenir comportamiento por defecto en controles
        this.preventDefaultControls();
    }
    
    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => {
            if (!this.isArmed) return;
            
            const key = e.key.toLowerCase();
            const step = 0.1;
            
            switch (key) {
                case 'w':
                case 'arrowup':
                    this.controlData.forward = Math.min(1, this.controlData.forward + step);
                    break;
                case 's':
                case 'arrowdown':
                    this.controlData.forward = Math.max(-1, this.controlData.forward - step);
                    break;
                case 'a':
                case 'arrowleft':
                    this.controlData.right = Math.max(-1, this.controlData.right - step);
                    break;
                case 'd':
                case 'arrowright':
                    this.controlData.right = Math.min(1, this.controlData.right + step);
                    break;
                case 'q':
                    this.controlData.yaw = Math.max(-1, this.controlData.yaw - step);
                    break;
                case 'e':
                    this.controlData.yaw = Math.min(1, this.controlData.yaw + step);
                    break;
                case 'r':
                    this.controlData.up = Math.min(1, this.controlData.up + step);
                    break;
                case 'f':
                    this.controlData.up = Math.max(-1, this.controlData.up - step);
                    break;
                case ' ':
                    e.preventDefault();
                    this.emergencyStop();
                    break;
            }
        });
        
        document.addEventListener('keyup', (e) => {
            const key = e.key.toLowerCase();
            
            // Resetear controles cuando se sueltan las teclas
            switch (key) {
                case 'w':
                case 's':
                case 'arrowup':
                case 'arrowdown':
                    this.controlData.forward = 0;
                    break;
                case 'a':
                case 'd':
                case 'arrowleft':
                case 'arrowright':
                    this.controlData.right = 0;
                    break;
                case 'q':
                case 'e':
                    this.controlData.yaw = 0;
                    break;
                case 'r':
                case 'f':
                    this.controlData.up = 0;
                    break;
            }
        });
    }
    
    preventDefaultControls() {
        // Prevenir scroll en controles de rango
        const rangeInputs = document.querySelectorAll('input[type="range"]');
        rangeInputs.forEach(input => {
            input.addEventListener('keydown', (e) => {
                if (e.key === 'ArrowUp' || e.key === 'ArrowDown' || 
                    e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
                    e.preventDefault();
                }
            });
        });
    }
    
    updateJoystick(values) {
        if (!this.isArmed) return;
        
        this.controlData.forward = values.y;
        this.controlData.right = values.x;
    }
    
    toggleArm() {
        this.isArmed = !this.isArmed;
        
        const armButton = document.getElementById('armButton');
        if (armButton) {
            if (this.isArmed) {
                armButton.innerHTML = '<i class="fas fa-power-off"></i> Desarmar Dron';
                armButton.classList.remove('btn-primary');
                armButton.classList.add('btn-secondary');
            } else {
                armButton.innerHTML = '<i class="fas fa-power-off"></i> Armar Dron';
                armButton.classList.remove('btn-secondary');
                armButton.classList.add('btn-primary');
                this.resetControls();
            }
        }
        
        // Enviar comando de armado
        if (window.websocketManager) {
            window.websocketManager.sendArm(this.isArmed);
        }
        
        console.log(`Dron ${this.isArmed ? 'armado' : 'desarmado'}`);
    }
    
    emergencyStop() {
        this.resetControls();
        
        // Enviar comando de emergencia
        if (window.websocketManager) {
            window.websocketManager.sendControl({
                forward: 0,
                right: 0,
                up: 0,
                yaw: 0
            });
        }
        
        console.log('¡PARADA DE EMERGENCIA!');
        
        // Mostrar notificación
        this.showNotification('¡Parada de emergencia activada!', 'error');
    }
    
    takeoff() {
        if (!this.isArmed) {
            this.showNotification('El dron debe estar armado para despegar', 'warning');
            return;
        }
        
        // Secuencia de despegue
        this.controlData.up = 0.5;
        
        setTimeout(() => {
            this.controlData.up = 0;
        }, 2000);
        
        console.log('Secuencia de despegue iniciada');
        this.showNotification('Despegando...', 'info');
    }
    
    land() {
        if (!this.isArmed) {
            this.showNotification('El dron debe estar armado para aterrizar', 'warning');
            return;
        }
        
        // Secuencia de aterrizaje
        this.controlData.up = -0.3;
        
        setTimeout(() => {
            this.controlData.up = 0;
        }, 3000);
        
        console.log('Secuencia de aterrizaje iniciada');
        this.showNotification('Aterrizando...', 'info');
    }
    
    resetControls() {
        this.controlData = {
            forward: 0,
            right: 0,
            up: 0,
            yaw: 0
        };
        
        // Resetear controles visuales
        const altitudeControl = document.getElementById('altitudeControl');
        const yawControl = document.getElementById('yawControl');
        const altitudeValue = document.getElementById('altitudeValue');
        const yawValue = document.getElementById('yawValue');
        
        if (altitudeControl) altitudeControl.value = 0;
        if (yawControl) yawControl.value = 0;
        if (altitudeValue) altitudeValue.textContent = '0.0';
        if (yawValue) yawValue.textContent = '0.0';
    }
    
    startControlLoop() {
        this.controlInterval = setInterval(() => {
            if (this.isArmed && window.websocketManager) {
                window.websocketManager.sendControl(this.controlData);
            }
        }, 1000 / this.controlRate);
    }
    
    showNotification(message, type = 'info') {
        // Crear notificación
        const notification = document.createElement('div');
        notification.className = `notification notification-${type}`;
        notification.innerHTML = `
            <i class="fas fa-${this.getNotificationIcon(type)}"></i>
            <span>${message}</span>
        `;
        
        // Estilos de notificación
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${this.getNotificationColor(type)};
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            z-index: 1000;
            display: flex;
            align-items: center;
            gap: 10px;
            font-weight: 500;
            transform: translateX(400px);
            transition: transform 0.3s ease;
        `;
        
        document.body.appendChild(notification);
        
        // Animar entrada
        setTimeout(() => {
            notification.style.transform = 'translateX(0)';
        }, 100);
        
        // Remover después de 3 segundos
        setTimeout(() => {
            notification.style.transform = 'translateX(400px)';
            setTimeout(() => {
                if (notification.parentNode) {
                    notification.parentNode.removeChild(notification);
                }
            }, 300);
        }, 3000);
    }
    
    getNotificationIcon(type) {
        switch (type) {
            case 'error': return 'exclamation-triangle';
            case 'warning': return 'exclamation-circle';
            case 'success': return 'check-circle';
            default: return 'info-circle';
        }
    }
    
    getNotificationColor(type) {
        switch (type) {
            case 'error': return '#e74c3c';
            case 'warning': return '#f39c12';
            case 'success': return '#27ae60';
            default: return '#3498db';
        }
    }
    
    // Obtener estado del sistema
    getSystemStatus() {
        return {
            armed: this.isArmed,
            controls: { ...this.controlData },
            connected: window.websocketManager ? window.websocketManager.isConnected : false
        };
    }
    
    // Limpiar recursos
    destroy() {
        if (this.controlInterval) {
            clearInterval(this.controlInterval);
            this.controlInterval = null;
        }
    }
}

// Inicializar sistema de control cuando el DOM esté listo
document.addEventListener('DOMContentLoaded', () => {
    window.controlSystem = new ControlSystem();
    
    // Mostrar mensaje de bienvenida
    setTimeout(() => {
        if (window.controlSystem) {
            window.controlSystem.showNotification(
                'Sistema de control remoto iniciado. Conecta el dron para comenzar.',
                'info'
            );
        }
    }, 1000);
});

// Manejar cierre de página
window.addEventListener('beforeunload', () => {
    if (window.controlSystem) {
        window.controlSystem.destroy();
    }
    if (window.telemetryManager) {
        window.telemetryManager.destroy();
    }
    if (window.websocketManager) {
        window.websocketManager.disconnect();
    }
}); 