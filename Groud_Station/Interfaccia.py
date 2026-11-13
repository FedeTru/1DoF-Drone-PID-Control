import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, TextBox
from collections import deque
import threading
import queue
import time
import csv
import os
import datetime

# ==========================================
# CONFIGURAZIONE
# ==========================================
PORT = 'COM6'         
BAUD_RATE = 115200
WINDOW_SIZE = 1000   
DIR_ESPERIMENTI = "esperimenti_csv" # Cartella di salvataggio

# Buffer circolari per i grafici (Rolling Window)
time_history = deque(maxlen=WINDOW_SIZE)
target_history = deque(maxlen=WINDOW_SIZE)
angle_history = deque(maxlen=WINDOW_SIZE)
pwm1_history = deque(maxlen=WINDOW_SIZE)
pwm2_history = deque(maxlen=WINDOW_SIZE)

data_lock = threading.Lock()
running = True

# Variabili di stato globale
is_armed = False
recording = False
experiment_start_time = 0.0
experiment_data = [] # Buffer RAM per i dati CSV dell'esperimento
current_target = 0.0 # Memoria dell'ultimo target per l'Heartbeat

# CODA DI TRASMISSIONE
tx_queue = queue.Queue()

# Creazione cartella esperimenti
if not os.path.exists(DIR_ESPERIMENTI):
    os.makedirs(DIR_ESPERIMENTI)
    print(f"[SISTEMA] Creata directory: {DIR_ESPERIMENTI}")

# ==========================================
# THREAD 1: LETTURA E SCRITTURA SERIALE
# ==========================================
def serial_thread():
    global running, recording, experiment_data
    print(f"[SERIAL] Tentativo di connessione su {PORT}...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        print("[SERIAL] Connesso con successo!")
        
        start_time = time.time()
        last_heartbeat = time.time()
        
        while running:
            # --- FAILSAFE HEARTBEAT (2 Hz) ---
            if time.time() - last_heartbeat > 0.5:
                tx_queue.put(f"T:{current_target}")
                last_heartbeat = time.time()

            # 1. INVIO COMANDI
            while not tx_queue.empty():
                comando = tx_queue.get()
                ser.write((comando + '\n').encode('utf-8'))
                ser.flush()
                time.sleep(0.005)

                if not comando.startswith("T:"): # Evita di spammare il terminale con l'heartbeat
                    print(f"[TX] Inviato: {comando}")

            # 2. RICEZIONE DATI
            if ser.in_waiting > 0:
                try:
                    raw_line = ser.readline()
                    if not raw_line.endswith(b'\n'):
                        continue 
                        
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    parts = line.split(',')
                    
                    if len(parts) == 4:
                        target = float(parts[0])
                        angle = float(parts[1])
                        pwm1 = int(parts[2])
                        pwm2 = int(parts[3])
                        current_time = time.time() - start_time
                        
                        with data_lock:
                            # Aggiornamento grafici in tempo reale
                            time_history.append(current_time)
                            target_history.append(target)
                            angle_history.append(angle)
                            pwm1_history.append(pwm1)
                            pwm2_history.append(pwm2)
                            
                            # Registrazione in RAM solo se l'esperimento è in corso
                            if recording:
                                t_exp = time.time() - experiment_start_time
                                experiment_data.append([f"{t_exp:.3f}", target, angle, pwm1, pwm2])
                                
                except (ValueError, IndexError):
                    continue
                    
    except Exception as e:
        print(f"\n[ERRORE SERIALE]: {e}")
    finally:
        running = False
        print("[SERIAL] Thread terminato.")

# ==========================================
# THREAD 2: INTERFACCIA GRAFICA & WIDGETS
# ==========================================
def setup_gui():
    fig, (ax_angle, ax_pwm) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.subplots_adjust(bottom=0.25)
    
    fig.suptitle("Ground Control Station 1DOF - SINGLE LOOP PID CONTROL", fontsize=14, fontweight='bold')
    
    # Setup assi grafici
    line_target, = ax_angle.plot([], [], 'r--', label="Target Angle")
    line_angle, = ax_angle.plot([], [], 'b-', label="Angolo Stimato", linewidth=2.0)
    ax_angle.set_ylabel("Angolo [°]")
    ax_angle.grid(True)
    ax_angle.legend(loc="upper right")
    ax_angle.set_ylim(-25, 25)
    
    # --- HUD DISPLAY ---
    testo_angolo = ax_angle.text(0.02, 0.85, '', transform=ax_angle.transAxes, 
                                 fontsize=13, fontweight='bold', color='navy',
                                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))
    
    testo_errore = ax_angle.text(0.02, 0.70, '', transform=ax_angle.transAxes, 
                                 fontsize=12, fontweight='bold', color='darkred',
                                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))
                                 
    testo_sforzo = ax_pwm.text(0.02, 0.85, '', transform=ax_pwm.transAxes, 
                                 fontsize=12, fontweight='bold', color='purple',
                                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))

    line_pwm1, = ax_pwm.step([], [], 'g-', label="PWM Motore 1", where='post')
    line_pwm2, = ax_pwm.step([], [], 'm-', label="PWM Motore 2", where='post')
    ax_pwm.set_xlabel("Tempo [s]")
    ax_pwm.set_ylabel("PWM [us]")
    ax_pwm.grid(True)
    ax_pwm.legend(loc="upper right")
    ax_pwm.set_ylim(900, 2100)

    # --- CONTROLLI UI PID SINGOLO ANELLO ---
    ax_box_t = plt.axes([0.08, 0.1, 0.10, 0.05])
    text_box_t = TextBox(ax_box_t, 'Target (°): ', initial='0.0')

    ax_box_p = plt.axes([0.28, 0.1, 0.10, 0.05])
    text_box_p = TextBox(ax_box_p, 'Kp: ', initial='0.0')
    
    ax_box_i = plt.axes([0.48, 0.1, 0.10, 0.05])
    text_box_i = TextBox(ax_box_i, 'Ki: ', initial='0.0')
    
    ax_box_d = plt.axes([0.68, 0.1, 0.10, 0.05])
    text_box_d = TextBox(ax_box_d, 'Kd: ', initial='0.0')

    ax_btn_update = plt.axes([0.82, 0.1, 0.07, 0.05])
    btn_update = Button(ax_btn_update, 'INVIA', color='lightblue')

    ax_btn_arm = plt.axes([0.90, 0.1, 0.07, 0.05])
    btn_arm = Button(ax_btn_arm, 'START', color='lightgreen')

    # --- CALLBACKS ---
    def on_update_clicked(event):
        global current_target
        try:
            target_val = float(text_box_t.text)
            if target_val > 18.0: target_val = 18.0
            elif target_val < -18.0: target_val = -18.0
            current_target = target_val # Aggiorna per l'Heartbeat
            tx_queue.put(f"T:{target_val}")
        except ValueError:
            print("\n[AVVISO GUI] Target non valido.")

        try:
            p_val = float(text_box_p.text)
            i_val = float(text_box_i.text)
            d_val = float(text_box_d.text)
            
            tx_queue.put(f"P:{p_val}")
            tx_queue.put(f"I:{i_val}")
            tx_queue.put(f"D:{d_val}")
        except ValueError:
            print("\n[AVVISO GUI] Parametri PID non validi.")
        
    def on_arm_clicked(event):
        global is_armed, recording, experiment_start_time, experiment_data
        
        is_armed = not is_armed 
        if is_armed:
            btn_arm.label.set_text('STOP')
            btn_arm.color = 'salmon'
            tx_queue.put("S:1")
            
            # Setup avvio esperimento
            with data_lock:
                experiment_data.clear() # Svuota i vecchi dati
                experiment_start_time = time.time()
                recording = True
            print("[ESPERIMENTO] Registrazione avviata...")
            
        else:
            btn_arm.label.set_text('START')
            btn_arm.color = 'lightgreen'
            tx_queue.put("S:0")  
            
            # Chiusura e valutazione esperimento
            with data_lock:
                recording = False
                durata = time.time() - experiment_start_time
                dati_da_salvare = list(experiment_data)
                
            if durata > 10.0 and len(dati_da_salvare) > 0:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                nome_file = os.path.join(DIR_ESPERIMENTI, f"exp_{timestamp}.csv")
                
                try:
                    with open(nome_file, mode='w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(["Tempo Exp (s)", "Target (deg)", "Angolo (deg)", "PWM1", "PWM2"])
                        writer.writerows(dati_da_salvare)
                    print(f"[ESPERIMENTO] Salvato con successo: {nome_file} (Durata: {durata:.1f}s, Campioni: {len(dati_da_salvare)})")
                except Exception as e:
                    print(f"[ESPERIMENTO] ERRORE durante il salvataggio: {e}")
            else:
                print(f"[ESPERIMENTO] Scartato. Durata insufficiente: {durata:.1f}s (Richiesti >10s)")
                
        plt.draw() 

    btn_update.on_clicked(on_update_clicked)
    btn_arm.on_clicked(on_arm_clicked)

    # --- ANIMAZIONE ---
    def init():
        line_target.set_data([], [])
        line_angle.set_data([], [])
        line_pwm1.set_data([], [])
        line_pwm2.set_data([], [])
        
        testo_angolo.set_text('')
        testo_errore.set_text('')
        testo_sforzo.set_text('')

        return line_target, line_angle, line_pwm1, line_pwm2, testo_angolo, testo_errore, testo_sforzo

    def update(frame):
        if not running:
            plt.close(fig)
            return line_target, line_angle, line_pwm1, line_pwm2, testo_angolo, testo_errore, testo_sforzo
            
        with data_lock:
            if len(time_history) == 0:
                return line_target, line_angle, line_pwm1, line_pwm2, testo_angolo, testo_errore, testo_sforzo
                
            x = list(time_history)
            y_target = list(target_history)
            y_angle = list(angle_history)
            y_pwm1 = list(pwm1_history)
            y_pwm2 = list(pwm2_history)
            
        line_target.set_data(x, y_target)
        line_angle.set_data(x, y_angle)
        line_pwm1.set_data(x, y_pwm1)
        line_pwm2.set_data(x, y_pwm2)
        ax_angle.set_xlim(x[0], x[-1])
        
        ultimo_angolo = y_angle[-1]
        ultimo_target = y_target[-1]
        errore_attuale = ultimo_target - ultimo_angolo
        sforzo_pid = (y_pwm1[-1] - y_pwm2[-1]) / 2.0
        
        testo_angolo.set_text(f"Angolo: {ultimo_angolo:+.2f}°")
        testo_errore.set_text(f"Errore: {errore_attuale:+.2f}°")
        testo_sforzo.set_text(f"Sforzo PID: {sforzo_pid:+.0f} µs")

        return line_target, line_angle, line_pwm1, line_pwm2, testo_angolo, testo_errore, testo_sforzo

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=33, blit=False, cache_frame_data=False)
    plt.show()

# ==========================================
# AVVIO
# ==========================================
if __name__ == "__main__":
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    
    time.sleep(0.5)
    
    if running:
        try:
            setup_gui()
        except KeyboardInterrupt:
            pass
        finally:
            running = False
            print("\n[INFO] Chiusura applicazione.")
    else:
        print("\n[ERRORE FATALE] Seriale non trovata. Ground Station abortita.")