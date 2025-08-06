class WebSocketManager {
    constructor(url = 'ws://localhost:8081') {
        this.url = url;
        this.socket = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 2000;
        this.pingInterval = null;
        this.latencyHistory = [];
        this.maxLatencyHistory = 50;
        
        // Callbacks
        this.onConnect = null;
        this.onDisconnect = null;
        this.onTelemetry = null;
        this.onError = null;
        
        // Inicializar
        this.connect();
    }
    
    connect() {
        try {
            this.socket = new WebSocket(this.url);
            
            this.socket.onopen = () => {
                console.log('WebSocket conectado');
                this.isConnected = true;
                this.reconnectAttempts = 0;
                this.updateConnectionStatus(true);
                this.startPingInterval();
                
                if (this.onConnect) {
                    this.onConnect();
                }
            };
            
            this.socket.onmessage = (event) => {
                this.handleMessage(event.data);
            };
            
            this.socket.onclose = (event) => {
                console.log('WebSocket desconectado:', event.code, event.reason);
                this.isConnected = false;
                this.updateConnectionStatus(false);
                this.stopPingInterval();
                
                if (this.onDisconnect) {
                    this.onDisconnect();
                }
                
                // Intentar reconectar
                this.attemptReconnect();
            };
            
            this.socket.onerror = (error) => {
                console.error('Error de WebSocket:', error);
                if (this.onError) {
                    this.onError(error);
                }
            };
            
        } catch (error) {
            console.error('Error al conectar WebSocket:', error);
            this.attemptReconnect();
        }
    }
    
    handleMessage(data) {
        try {
            const message = JSON.parse(data);
            
            switch (message.type) {
                case 'telemetry':
                    if (this.onTelemetry) {
                        this.onTelemetry(message.data);
                    }
                    break;
                    
                case 'pong':
                    this.calculateLatency(message.timestamp);
                    break;
                    
                default:
                    console.log('Mensaje desconocido:', message);
            }
        } catch (error) {
            console.error('Error al procesar mensaje:', error);
        }
    }
    
    send(message) {
        if (this.isConnected && this.socket) {
            try {
                this.socket.send(JSON.stringify(message));
            } catch (error) {
                console.error('Error al enviar mensaje:', error);
            }
        } else {
            console.warn('WebSocket no conectado');
        }
    }
    
    sendControl(controlData) {
        this.send({
            type: 'control',
            data: controlData
        });
    }
    
    sendArm(armed) {
        this.send({
            type: 'arm',
            data: armed
        });
    }
    
    startPingInterval() {
        this.pingInterval = setInterval(() => {
            if (this.isConnected) {
                this.send({
                    type: 'ping',
                    timestamp: Date.now()
                });
            }
        }, 1000);
    }
    
    stopPingInterval() {
        if (this.pingInterval) {
            clearInterval(this.pingInterval);
            this.pingInterval = null;
        }
    }
    
    calculateLatency(sendTime) {
        const receiveTime = Date.now();
        const latency = receiveTime - sendTime;
        
        this.latencyHistory.push(latency);
        
        // Mantener solo los últimos valores
        if (this.latencyHistory.length > this.maxLatencyHistory) {
            this.latencyHistory.shift();
        }
        
        // Actualizar gráfico de latencia
        this.updateLatencyChart();
    }
    
    updateLatencyChart() {
        const canvas = document.getElementById('latencyCanvas');
        if (!canvas) return;
        
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;
        
        // Limpiar canvas
        ctx.clearRect(0, 0, width, height);
        
        if (this.latencyHistory.length < 2) return;
        
        // Calcular estadísticas
        const maxLatency = Math.max(...this.latencyHistory);
        const avgLatency = this.latencyHistory.reduce((a, b) => a + b, 0) / this.latencyHistory.length;
        
        // Actualizar valores mostrados
        document.getElementById('avgLatency').textContent = Math.round(avgLatency);
        document.getElementById('maxLatency').textContent = Math.round(maxLatency);
        
        // Dibujar gráfico
        ctx.strokeStyle = '#3498db';
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        const stepX = width / (this.latencyHistory.length - 1);
        const maxValue = Math.max(maxLatency, 100); // Mínimo 100ms para escala
        
        this.latencyHistory.forEach((latency, index) => {
            const x = index * stepX;
            const y = height - (latency / maxValue) * height;
            
            if (index === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        });
        
        ctx.stroke();
        
        // Dibujar línea de promedio
        ctx.strokeStyle = '#e74c3c';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(0, height - (avgLatency / maxValue) * height);
        ctx.lineTo(width, height - (avgLatency / maxValue) * height);
        ctx.stroke();
        ctx.setLineDash([]);
    }
    
    updateConnectionStatus(connected) {
        const indicator = document.getElementById('statusIndicator');
        const statusText = document.getElementById('statusText');
        
        if (indicator && statusText) {
            if (connected) {
                indicator.classList.add('connected');
                statusText.textContent = 'Conectado';
                statusText.style.color = '#27ae60';
            } else {
                indicator.classList.remove('connected');
                statusText.textContent = 'Desconectado';
                statusText.style.color = '#e74c3c';
            }
        }
    }
    
    attemptReconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`Intentando reconectar... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
            
            setTimeout(() => {
                this.connect();
            }, this.reconnectDelay * this.reconnectAttempts);
        } else {
            console.error('Máximo número de intentos de reconexión alcanzado');
            this.updateConnectionStatus(false);
        }
    }
    
    disconnect() {
        if (this.socket) {
            this.socket.close();
        }
        this.stopPingInterval();
    }
    
    getLatencyStats() {
        if (this.latencyHistory.length === 0) {
            return { average: 0, max: 0, min: 0 };
        }
        
        return {
            average: this.latencyHistory.reduce((a, b) => a + b, 0) / this.latencyHistory.length,
            max: Math.max(...this.latencyHistory),
            min: Math.min(...this.latencyHistory)
        };
    }
}

// Inicializar WebSocket cuando el DOM esté listo
document.addEventListener('DOMContentLoaded', () => {
    window.websocketManager = new WebSocketManager();
    
    // Configurar callbacks
    window.websocketManager.onConnect = () => {
        console.log('Sistema de control conectado');
    };
    
    window.websocketManager.onDisconnect = () => {
        console.log('Sistema de control desconectado');
    };
    
    window.websocketManager.onTelemetry = (telemetry) => {
        // Actualizar telemetría en la interfaz
        if (window.telemetryManager) {
            window.telemetryManager.updateTelemetry(telemetry);
        }
    };
    
    window.websocketManager.onError = (error) => {
        console.error('Error en WebSocket:', error);
    };
}); 