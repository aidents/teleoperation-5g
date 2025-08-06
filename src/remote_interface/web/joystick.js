class VirtualJoystick {
    constructor(container, options = {}) {
        this.container = container;
        this.knob = container.querySelector('.joystick-knob');
        
        // Configuración por defecto
        this.options = {
            maxDistance: 70,
            deadZone: 5,
            onMove: null,
            onEnd: null,
            ...options
        };
        
        // Estado del joystick
        this.isActive = false;
        this.centerX = 0;
        this.centerY = 0;
        this.currentX = 0;
        this.currentY = 0;
        
        // Inicializar
        this.init();
    }
    
    init() {
        // Calcular centro del joystick
        this.updateCenter();
        
        // Event listeners para mouse
        this.container.addEventListener('mousedown', this.onStart.bind(this));
        document.addEventListener('mousemove', this.onMove.bind(this));
        document.addEventListener('mouseup', this.onEnd.bind(this));
        
        // Event listeners para touch
        this.container.addEventListener('touchstart', this.onStart.bind(this));
        document.addEventListener('touchmove', this.onMove.bind(this));
        document.addEventListener('touchend', this.onEnd.bind(this));
        
        // Prevenir contexto del menú
        this.container.addEventListener('contextmenu', e => e.preventDefault());
        
        // Centrar el knob inicialmente
        this.updateKnobPosition(0, 0);
    }
    
    updateCenter() {
        const rect = this.container.getBoundingClientRect();
        this.centerX = rect.width / 2;
        this.centerY = rect.height / 2;
    }
    
    onStart(event) {
        event.preventDefault();
        this.isActive = true;
        this.container.style.cursor = 'grabbing';
        this.updateCenter();
        this.updatePosition(event);
    }
    
    onMove(event) {
        if (!this.isActive) return;
        event.preventDefault();
        this.updatePosition(event);
    }
    
    onEnd(event) {
        if (!this.isActive) return;
        event.preventDefault();
        this.isActive = false;
        this.container.style.cursor = 'grab';
        
        // Resetear posición
        this.updateKnobPosition(0, 0);
        
        // Llamar callback de fin
        if (this.options.onEnd) {
            this.options.onEnd({ x: 0, y: 0 });
        }
    }
    
    updatePosition(event) {
        const rect = this.container.getBoundingClientRect();
        let clientX, clientY;
        
        if (event.type.includes('touch')) {
            clientX = event.touches[0].clientX;
            clientY = event.touches[0].clientY;
        } else {
            clientX = event.clientX;
            clientY = event.clientY;
        }
        
        // Calcular posición relativa al centro
        const deltaX = clientX - rect.left - this.centerX;
        const deltaY = clientY - rect.top - this.centerY;
        
        // Calcular distancia
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Aplicar zona muerta
        if (distance < this.options.deadZone) {
            this.updateKnobPosition(0, 0);
            this.currentX = 0;
            this.currentY = 0;
        } else {
            // Limitar distancia máxima
            let limitedDistance = distance;
            if (distance > this.options.maxDistance) {
                limitedDistance = this.options.maxDistance;
            }
            
            // Calcular posición normalizada
            const angle = Math.atan2(deltaY, deltaX);
            const limitedX = Math.cos(angle) * limitedDistance;
            const limitedY = Math.sin(angle) * limitedDistance;
            
            this.updateKnobPosition(limitedX, limitedY);
            
            // Normalizar valores entre -1 y 1
            this.currentX = limitedX / this.options.maxDistance;
            this.currentY = -limitedY / this.options.maxDistance; // Invertir Y para intuición
        }
        
        // Llamar callback de movimiento
        if (this.options.onMove) {
            this.options.onMove({
                x: this.currentX,
                y: this.currentY
            });
        }
    }
    
    updateKnobPosition(x, y) {
        this.knob.style.transform = `translate(${x}px, ${y}px)`;
    }
    
    getValues() {
        return {
            x: this.currentX,
            y: this.currentY
        };
    }
    
    destroy() {
        // Remover event listeners
        this.container.removeEventListener('mousedown', this.onStart);
        this.container.removeEventListener('touchstart', this.onStart);
        document.removeEventListener('mousemove', this.onMove);
        document.removeEventListener('touchmove', this.onMove);
        document.removeEventListener('mouseup', this.onEnd);
        document.removeEventListener('touchend', this.onEnd);
    }
}

// Inicializar joystick cuando el DOM esté listo
document.addEventListener('DOMContentLoaded', () => {
    const joystickContainer = document.getElementById('joystick');
    if (joystickContainer) {
        window.virtualJoystick = new VirtualJoystick(joystickContainer, {
            onMove: (values) => {
                // Enviar valores del joystick al sistema de control
                if (window.controlSystem) {
                    window.controlSystem.updateJoystick(values);
                }
            },
            onEnd: () => {
                // Detener movimiento cuando se suelta el joystick
                if (window.controlSystem) {
                    window.controlSystem.updateJoystick({ x: 0, y: 0 });
                }
            }
        });
    }
}); 