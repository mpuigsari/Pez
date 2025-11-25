#!/usr/bin/env python3
# scripts/serial_emulator.py
import os
import pty
import select
import time
import random
import argparse
import sys
import signal
import termios

class SerialEmulator:
    def __init__(self, port1_link, port2_link, delay_ms=0, jitter_ms=0, loss_pct=0):
        self.port1_link = port1_link
        self.port2_link = port2_link
        self.delay_ms = delay_ms
        self.jitter_ms = jitter_ms
        self.loss_pct = loss_pct
        self.running = True
        
        print(f"[SerialEmulator] Creating virtual serial ports:")
        print(f"[SerialEmulator]   {port1_link} ↔ {port2_link}")
        print(f"[SerialEmulator] Parameters: delay={delay_ms}ms, jitter={jitter_ms}ms, loss={loss_pct}%")
        
        # Crear los PTYs
        self.master1, self.slave1 = pty.openpty()
        self.master2, self.slave2 = pty.openpty()
        
        # Configurar como raw (sin procesamiento)
        self._set_raw_mode(self.master1)
        self._set_raw_mode(self.master2)
        self._set_raw_mode(self.slave1)
        self._set_raw_mode(self.slave2)
        
        # Crear symlinks
        self._create_symlink(self.slave1, port1_link)
        self._create_symlink(self.slave2, port2_link)
        
        print(f"[SerialEmulator] Virtual ports created successfully")
        
        # Buffer para simular delay
        self.buffer_1to2 = []
        self.buffer_2to1 = []
    
    def _set_raw_mode(self, fd):
        """Configura el PTY en modo raw"""
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | 
                         termios.ISTRIP | termios.INLCR | termios.IGNCR | 
                         termios.ICRNL | termios.IXON)
            attrs[1] &= ~termios.OPOST
            attrs[2] &= ~(termios.CSIZE | termios.PARENB)
            attrs[2] |= termios.CS8
            attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | 
                         termios.ISIG | termios.IEXTEN)
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except:
            pass
    
    def _create_symlink(self, fd, link_path):
        """Crea un symlink al PTY esclavo"""
        # Eliminar symlink existente
        if os.path.exists(link_path):
            os.remove(link_path)
        
        # Obtener el nombre del dispositivo
        slave_name = os.ttyname(fd)
        
        # Crear symlink
        os.symlink(slave_name, link_path)
        os.chmod(link_path, 0o666)
    
    def _should_drop_packet(self):
        """Simula pérdida de paquetes"""
        return random.random() * 100 < self.loss_pct
    
    def _get_delay(self):
        """Calcula delay con jitter"""
        if self.jitter_ms > 0:
            jitter = random.uniform(-self.jitter_ms, self.jitter_ms)
        else:
            jitter = 0
        return max(0, self.delay_ms + jitter) / 1000.0
    
    def _process_buffers(self):
        """Procesa los buffers con delay"""
        current_time = time.time()
        
        # Procesar buffer 1->2
        while self.buffer_1to2 and self.buffer_1to2[0][0] <= current_time:
            _, data = self.buffer_1to2.pop(0)
            try:
                os.write(self.master2, data)
            except OSError:
                pass
        
        # Procesar buffer 2->1
        while self.buffer_2to1 and self.buffer_2to1[0][0] <= current_time:
            _, data = self.buffer_2to1.pop(0)
            try:
                os.write(self.master1, data)
            except OSError:
                pass
    
    def start(self):
        """Inicia el forwarding bidireccional"""
        print("[SerialEmulator] Emulation running... Press Ctrl+C to stop")
        
        try:
            while self.running:
                # Usar select para esperar datos
                readable, _, _ = select.select(
                    [self.master1, self.master2], [], [], 0.01
                )
                
                # Leer de master1 y enviar a master2
                if self.master1 in readable:
                    try:
                        data = os.read(self.master1, 4096)
                        if data:
                            if not self._should_drop_packet():
                                delay_time = time.time() + self._get_delay()
                                self.buffer_1to2.append((delay_time, data))
                            else:
                                print(f"[SerialEmulator] [1→2] Dropped {len(data)} bytes")
                    except OSError:
                        pass
                
                # Leer de master2 y enviar a master1
                if self.master2 in readable:
                    try:
                        data = os.read(self.master2, 4096)
                        if data:
                            if not self._should_drop_packet():
                                delay_time = time.time() + self._get_delay()
                                self.buffer_2to1.append((delay_time, data))
                            else:
                                print(f"[SerialEmulator] [2→1] Dropped {len(data)} bytes")
                    except OSError:
                        pass
                
                # Procesar buffers
                self._process_buffers()
                
        except KeyboardInterrupt:
            print("\n[SerialEmulator] Shutting down...")
        finally:
            self.stop()
    
    def stop(self):
        """Detiene el emulador y limpia recursos"""
        self.running = False
        
        # Cerrar file descriptors
        try:
            os.close(self.master1)
            os.close(self.master2)
            os.close(self.slave1)
            os.close(self.slave2)
        except:
            pass
        
        # Eliminar symlinks
        try:
            if os.path.exists(self.port1_link):
                os.remove(self.port1_link)
            if os.path.exists(self.port2_link):
                os.remove(self.port2_link)
        except:
            pass
        
        print("[SerialEmulator] Stopped")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Serial Port Network Emulator')
    parser.add_argument('port1', help='First serial port symlink path')
    parser.add_argument('port2', help='Second serial port symlink path')
    parser.add_argument('--delay', type=float, default=0, help='Delay in ms')
    parser.add_argument('--jitter', type=float, default=0, help='Jitter in ms')
    parser.add_argument('--loss', type=float, default=0, help='Packet loss percentage')
    
    args = parser.parse_args()
    
    emulator = SerialEmulator(
        args.port1,
        args.port2,
        delay_ms=args.delay,
        jitter_ms=args.jitter,
        loss_pct=args.loss
    )
    
    # Manejar señales
    def signal_handler(sig, frame):
        emulator.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    emulator.start()