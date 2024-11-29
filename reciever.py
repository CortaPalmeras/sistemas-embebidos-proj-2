from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QMessageBox, QProgressDialog
from PyQt5.QtCore import Qt
import sys
import serial
from struct import pack, unpack
import matplotlib.pyplot as plt
import array
import time

# Se configura el puerto y el BAUD_Rate
PORT = '/dev/ttyUSB0'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuración de la ESP32

# Se abre la conexión serial
ser = serial.Serial(PORT, BAUD_RATE, timeout=5)

while (True):
    # reinicia el programa en caso de que estaba en curso
    _ = ser.write('q'.encode())

    # imprimir a la terminal logs de inicio
    print("\n-- output inicial --")
    print(ser.readall().decode())

    ser.timeout = 1

    # handshake para sincronizar el computador con el ESP
    print("\n-- handshake --")
    _ = ser.write('++++++++++*'.encode())
    resultado = ser.read(4).decode()
    print(resultado)

    if resultado == 'GOOD':
        break
    else:
        print("error al realizar handshake, intentando de nuevo")
        ser.timeout = 5

# por si algo se imprime despues del handshake
print("\n-- output post handshake --")
print(ser.readall().decode())


# se lee el tamaño de la ventana de datos
def solicitar_tamano_ventana():
    print("\n-- leer tamaño del sample --")
    _ = ser.write('s'.encode())
    raw_sample_size = ser.read(4)

    if len(raw_sample_size) == 4:
        new_sample_size = unpack('I', raw_sample_size)[0]
        print(f"el tamaño de la muestra es {new_sample_size}")
        return new_sample_size
    else:
        print(f"se leyeron {len(raw_sample_size)} bytes")
        exit(1)

samples = solicitar_tamano_ventana()


def generar_graficos(tiempo, 
                     lista_acc_x, esp_rms_acc_x, 
                     lista_acc_y, esp_rms_acc_y,
                     lista_acc_z, esp_rms_acc_z,
                     lista_gyr_x, esp_rms_gyr_x,
                     lista_gyr_y, esp_rms_gyr_y,
                     lista_gyr_z, esp_rms_gyr_z):
    
    # Crear una figura con una cuadrícula de subgráficos de 3 filas x 2 columnas
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    axes = axes.flatten()  # Aplanar para acceso fácil a cada subgráfico

    # Gráficos de aceleración
    axes[0].plot(tiempo, lista_acc_x, marker='o', linestyle='-', color='b', label=f'RMS={esp_rms_acc_x:.2f} g')
    axes[0].set_title('Variación de la aceleración en el eje X')
    axes[0].set_xlabel('Tiempo (segundos)')
    axes[0].set_ylabel('Aceleración (g)')
    axes[0].legend()

    axes[1].plot(tiempo, lista_acc_y, marker='o', linestyle='-', color='b', label=f'RMS={esp_rms_acc_y:.2f} g')
    axes[1].set_title('Variación de la aceleración en el eje Y')
    axes[1].set_xlabel('Tiempo (segundos)')
    axes[1].set_ylabel('Aceleración (g)')
    axes[1].legend()

    axes[2].plot(tiempo, lista_acc_z, marker='o', linestyle='-', color='b', label=f'RMS={esp_rms_acc_z:.2f} g')
    axes[2].set_title('Variación de la aceleración en el eje Z')
    axes[2].set_xlabel('Tiempo (segundos)')
    axes[2].set_ylabel('Aceleración (g)')
    axes[2].legend()

    # Gráficos de giroscopio
    axes[3].plot(tiempo, lista_gyr_x, marker='o', linestyle='-', color='r', label=f'RMS={esp_rms_gyr_x:.2f} °/s')
    axes[3].set_title('Variación del giro en el eje X')
    axes[3].set_xlabel('Tiempo (segundos)')
    axes[3].set_ylabel('Velocidad angular (°/s)')
    axes[3].legend()

    axes[4].plot(tiempo, lista_gyr_y, marker='o', linestyle='-', color='r', label=f'RMS={esp_rms_gyr_y:.2f} °/s')
    axes[4].set_title('Variación del giro en el eje Y')
    axes[4].set_xlabel('Tiempo (segundos)')
    axes[4].set_ylabel('Velocidad angular (°/s)')
    axes[4].legend()

    axes[5].plot(tiempo, lista_gyr_z, marker='o', linestyle='-', color='r', label=f'RMS={esp_rms_gyr_z:.2f} °/s')
    axes[5].set_title('Variación del giro en el eje Z')
    axes[5].set_xlabel('Tiempo (segundos)')
    axes[5].set_ylabel('Velocidad angular (°/s)')
    axes[5].legend()

    # Ajustar espaciado entre subgráficos
    plt.tight_layout()
    plt.show()


def solicitar_ventana_datos(progress_dialog):
    """Función para solicitar la ventana de datos"""
    global samples

    print("\n-- extraccion de datos --")

    _ = ser.write('g'.encode())
    lista_acc_x = []
    lista_acc_y = []
    lista_acc_z=[]
    lista_gyr_x=[]
    lista_gyr_y=[]
    lista_gyr_z=[]
    tiempo = list(range(samples))

    time.sleep(1.5)

    # Actualizar la barra de progreso
    for i in range(samples):
        data = ser.read(24)
        if len(data) != 24:
            print("\nOcurrió un error en la lectura de datos")
            print(ser.readall().decode())
            progress_dialog.close()
            return

        acc_x, acc_y, acc_z, gyr_x,gyr_y,gyr_z = unpack("ffffff", data)
        lista_acc_x.append(acc_x)
        lista_acc_y.append(acc_y)
        lista_acc_z.append(acc_z)
        lista_gyr_x.append(gyr_x)
        lista_gyr_y.append(gyr_y)
        lista_gyr_z.append(gyr_z)
        print(f"acc_x: {acc_x:.5f}\tacc_y: {acc_y:.5f}" +
              f"\tacc_z:{acc_z:.5f}\tgyr_x:{gyr_x:.5f}" +
              f"\tgyr_y:{gyr_y:.5f}\tgyr_z:{gyr_z:.5f}")
        

        progress_dialog.setValue(i+1)
        if progress_dialog.wasCanceled():
            print("esperando a que termine la muestra...", end='')
            _ = ser.readall()
            print(" listo")
            return

    data = ser.read(24)
    esp_rms_acc_x, esp_rms_acc_y,esp_rms_acc_z,esp_rms_gyr_x,esp_rms_gyr_y,esp_rms_gyr_z = unpack("ffffff", data)

    print(f"\n-- resultados RMS esp --")
    print(f"acc_x: {esp_rms_acc_x}")
    print(f"acc_y: {esp_rms_acc_y}")
    print(f"acc_z: {esp_rms_acc_z} ")
    print(f"gyr_x: {esp_rms_gyr_x} ")
    print(f"gyr_y: {esp_rms_gyr_y} ")
    print(f"gyr_z: {esp_rms_gyr_z} ")

    data5peaks_acc_x = ser.read(20)
    data5peaks_acc_y = ser.read(20)
    data5peaks_acc_z = ser.read(20)
    data5peaks_gyr_x = ser.read(20)
    data5peaks_gyr_y = ser.read(20)
    data5peaks_gyr_z = ser.read(20)
    
    peaks_acc_x = array.array('f', data5peaks_acc_x).tolist()
    peaks_acc_y = array.array('f', data5peaks_acc_y).tolist()
    peaks_acc_z = array.array('f', data5peaks_acc_z).tolist()
    peaks_gyr_x = array.array('f', data5peaks_gyr_x).tolist()
    peaks_gyr_y = array.array('f', data5peaks_gyr_y).tolist()
    peaks_gyr_z = array.array('f', data5peaks_gyr_z).tolist()
    
    print(f"\n-- resultados 5 peaks esp --")
    print(f"acc_x: {peaks_acc_x}")
    print(f"acc_y: {peaks_acc_y}")
    print(f"acc_z: {peaks_acc_z}")
    print(f"gyr_x: {peaks_gyr_x}")
    print(f"gyr_y: {peaks_gyr_y}")
    print(f"gyr_z: {peaks_gyr_z}")
    
    data_fft_real_acc_x = ser.read(4 * samples)
    data_fft_imag_acc_x = ser.read(4 * samples)
    fft_real_acc_x = array.array('f', data_fft_real_acc_x).tolist()
    fft_imag_acc_x = array.array('f', data_fft_imag_acc_x).tolist()

    data_fft_real_acc_y = ser.read(4 * samples)
    data_fft_imag_acc_y = ser.read(4 * samples)
    fft_real_acc_y = array.array('f', data_fft_real_acc_y).tolist()
    fft_imag_acc_y = array.array('f', data_fft_imag_acc_y).tolist()

    data_fft_real_acc_z = ser.read(4 * samples)
    data_fft_imag_acc_z = ser.read(4 * samples)
    fft_real_acc_z = array.array('f', data_fft_real_acc_z).tolist()
    fft_imag_acc_z = array.array('f', data_fft_imag_acc_z).tolist()

    data_fft_real_gyr_x = ser.read(4 * samples)
    data_fft_imag_gyr_x = ser.read(4 * samples)
    fft_real_gyr_x = array.array('f', data_fft_real_gyr_x).tolist()
    fft_imag_gyr_x = array.array('f', data_fft_imag_gyr_x).tolist()

    data_fft_real_gyr_y = ser.read(4 * samples)
    data_fft_imag_gyr_y = ser.read(4 * samples)
    fft_real_gyr_y = array.array('f', data_fft_real_gyr_y).tolist()
    fft_imag_gyr_y = array.array('f', data_fft_imag_gyr_y).tolist()

    data_fft_real_gyr_z = ser.read(4 * samples)
    data_fft_imag_gyr_z = ser.read(4 * samples)
    fft_real_gyr_z = array.array('f', data_fft_real_gyr_z).tolist()
    fft_imag_gyr_z = array.array('f', data_fft_imag_gyr_z).tolist()

    print(f"\n-- resultados fft esp --")
    print(f"acc_x: {fft_real_acc_x}")
    print(f"acc_y: {fft_real_acc_y}")
    print(f"acc_z: {fft_real_acc_z}")
    print(f"gyr_x: {fft_real_gyr_x}")
    print(f"gyr_y: {fft_real_gyr_y}")
    print(f"gyr_z: {fft_real_gyr_z}")
    
    print(f"\n-- resultados fft imag esp --")
    print(f"acc_x: {fft_imag_acc_x}")
    print(f"acc_y: {fft_imag_acc_y}")
    print(f"acc_z: {fft_imag_acc_z}")
    print(f"gyr_x: {fft_imag_gyr_x}")
    print(f"gyr_y: {fft_imag_gyr_y}")
    print(f"gyr_z: {fft_imag_gyr_z}")

    progress_dialog.close()

    print(ser.readall(), end="")

    generar_graficos(tiempo, 
                     lista_acc_x, esp_rms_acc_x, 
                     lista_acc_y, esp_rms_acc_y,
                     lista_acc_z, esp_rms_acc_z,
                     lista_gyr_x, esp_rms_gyr_x,
                     lista_gyr_y, esp_rms_gyr_y,
                     lista_gyr_z, esp_rms_gyr_z)

def cambiar_tamanho_ventana(tamanho: int) -> bool:
    """Función para cambiar el tamanho de la ventana de datos"""
    global samples
    
    print("\n-- cambiar tamaño del sample --")
    _ = ser.write('c'.encode())
    _ = ser.write(pack('I', tamanho))

    # en caso de el esp devuelva errores estos se imprimen en la terminal
    print(ser.readall(), end="")

    samples = solicitar_tamano_ventana()
    return samples == tamanho



# Clase principal para la interfaz PyQt5
class ESP32App(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        """Configuración de la interfaz gráfica"""

        # Layout principal
        layout = QVBoxLayout()

        # Botones
        self.label = QLabel("Seleccione una opción:")
        self.boton_ventana = QPushButton("Solicitar ventana de datos", self)
        self.cambiar_tamanho_ventana_button = QPushButton("Cambiar tamaño de la ventana", self)
        self.reiniciar_conexion_button = QPushButton("Cerrar conexión", self)

        # Input para cambiar el tamanho de la ventana
        self.window_size_input = QLineEdit(self)
        self.window_size_input.setPlaceholderText("Nuevo tamaño de ventana")

        # Eventos de los botones
        self.boton_ventana.clicked.connect(self.solicitar_ventana_datos)
        self.cambiar_tamanho_ventana_button.clicked.connect(self.cambiar_tamanho_ventana)
        self.reiniciar_conexion_button.clicked.connect(self.reiniciar_conexion)

        # Anhadir los widgets al layout
        layout.addWidget(self.label)
        layout.addWidget(self.boton_ventana)
        layout.addWidget(self.window_size_input)
        layout.addWidget(self.cambiar_tamanho_ventana_button)
        layout.addWidget(self.reiniciar_conexion_button)

        # Configuración final de la ventana
        self.setLayout(layout)
        self.setWindowTitle('ESP32 Interface')
        self.setGeometry(300, 300, 400, 200)
        self.show()

    def solicitar_ventana_datos(self):
        """Solicitar la ventana de datos a la ESP32"""
        # Crear el cuadro de progreso
        progress_dialog = QProgressDialog("Recopilando datos...", "Cancelar", 0, samples, self)
        progress_dialog.setWindowModality(Qt.WindowModal) 
        progress_dialog.setMinimumDuration(0)
        progress_dialog.setValue(0)
         # Asegurar que el diálogo esté al frente y tenga el foco
        progress_dialog.raise_()
        progress_dialog.activateWindow()

        solicitar_ventana_datos(progress_dialog)


    def cambiar_tamanho_ventana(self):
        """Cambiar el tamanho de la ventana de datos"""
        tamanho = self.window_size_input.text()
        if tamanho.isdigit():
            if cambiar_tamanho_ventana(int(tamanho)):
                QMessageBox.information(self, 'Éxito', "Se guardó el tamaño de la ventana exitosamente.")
            else:
                QMessageBox.information(self, 'Error', "Ocurrió un error al cambiar el tamaño de la ventana de datos.")
        else:
            QMessageBox.information(self, 'Numero no valido', "Numero no valido")


    def reiniciar_conexion(self):
        _ = ser.write('q'.encode())
        QMessageBox.information(self, 'Reinicio', 'La conexión se ha reiniciado.')
        ser.close()
        print("Saliendo del programa")
        sys.exit(0)



# Función principal para ejecutar la aplicación
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ESP32App()
    sys.exit(app.exec_())
