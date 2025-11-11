import serial
import time
import csv
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# CONFIGURAÇÕES
PORTA_SERIAL = 'COM3'         # 🔁 Altere para a porta correta (ex: '/dev/ttyUSB0' no Linux)
BAUD_RATE = 921600            # Igual ao do código no ESP32
ARQUIVO_CSV = 'bno055_log.csv'
BUFFER_MAX = 500              # N° de amostras no gráfico (em tempo real)

# BUFFER CIRCULAR PARA GRÁFICO
timestamps = deque(maxlen=BUFFER_MAX)
ax_vals = deque(maxlen=BUFFER_MAX)
ay_vals = deque(maxlen=BUFFER_MAX)
az_vals = deque(maxlen=BUFFER_MAX)

# INICIALIZA SERIAL E CSV
ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=1)
time.sleep(2)  # Aguarda ESP32 reiniciar

csv_file = open(ARQUIVO_CSV, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp_us', 'ax (m/s^2)', 'ay (m/s^2)', 'az (m/s^2)'])

print(f"[INFO] Gravando dados em: {ARQUIVO_CSV}")
print("[INFO] Pressione Ctrl+C para parar.")

# PLOT
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='ax')
line2, = ax.plot([], [], label='ay')
line3, = ax.plot([], [], label='az')
ax.set_ylim(-20, 20)  # Ajuste conforme necessário
ax.set_xlim(0, BUFFER_MAX)
ax.legend()
ax.set_title('Aceleração BNO055 (m/s²)')
ax.set_xlabel('Amostras recentes')
ax.set_ylabel('m/s²')

def update_plot(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        if '\t' not in line:
            return line1, line2, line3  # Ignora mensagens como "Samples/sec"

        parts = line.split('\t')
        if len(parts) != 4:
            return line1, line2, line3

        t_us, ax_val, ay_val, az_val = parts
        t_us = int(t_us)
        ax_val = float(ax_val)
        ay_val = float(ay_val)
        az_val = float(az_val)

        timestamps.append(t_us)
        ax_vals.append(ax_val)
        ay_vals.append(ay_val)
        az_vals.append(az_val)

        # Grava no CSV
        csv_writer.writerow([t_us, ax_val, ay_val, az_val])

        # Atualiza gráfico
        line1.set_data(range(len(ax_vals)), ax_vals)
        line2.set_data(range(len(ay_vals)), ay_vals)
        line3.set_data(range(len(az_vals)), az_vals)
        ax.relim()
        ax.autoscale_view()

    except Exception as e:
        print(f"[ERRO] {e}")
    return line1, line2, line3

ani = animation.FuncAnimation(fig, update_plot, interval=1, blit=True)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    csv_file.close()
    ser.close()
    print("\n[OK] Finalizado. Dados salvos em", ARQUIVO_CSV)
