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
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# reinicia el programa en caso de que estaba en curso
_ = ser.write('q'.encode())
_ = ser.readall()

# handshake para sincronizar el computador con el ESP
print("\n-- handshake --")
_ = ser.write('++++++++++*'.encode())
resultado = ser.read(4).decode()
print(resultado)

# imprimir a la terminal logs de inicio
print("\n-- output inicial --")
print(ser.readall().decode())

# se lee el tamanho de la ventana de datos
def solicitar_tamano_ventana():
    print("\n-- leer tamaño del sample --")
    _ = ser.write('s'.encode())
    raw_sample_size = ser.read(4)

    if len(raw_sample_size) == 4:
        new_sample_size = unpack('I', raw_sample_size)[0]
        print(f"el tamaño previo de la muestra es {new_sample_size}")
        return new_sample_size
    else:
        print(f"se leyeron {len(raw_sample_size)} bytes")
        exit(1)

samples = solicitar_tamano_ventana()


def generar_graficos(tiempo, 
                     lista_temp, esp_rms_temp, 
                     lista_pres, esp_rms_pres,
                     lista_hum, esp_rms_hum,
                     lista_gas, esp_rms_gas):
    
    fig, axes = plt.subplots(2, 2, figsize=(10, 5))
    (ax1, ax2), (ax3, ax4) = axes

    ax1.plot(tiempo, lista_temp, marker='o', linestyle='-', color='b',  label=f'Temperatura (°C) RMS={esp_rms_temp}')
    ax1.set_title('Variación de la Temperatura en el Tiempo')
    ax1.set_xlabel('Tiempo (segundos)')
    ax1.set_ylabel('Temperatura (°C)')

    ax2.plot(tiempo, lista_pres, marker='o', linestyle='-', color='b',  label=f'Pascal (kPa) RMS={esp_rms_pres}')
    ax2.set_title('Variación de la Presión en el Tiempo')
    ax2.set_xlabel('Tiempo (segundos)')
    ax2.set_ylabel('Presión (kPa)')

    ax3.plot(tiempo, lista_hum, marker='o', linestyle='-', color='b',  label=f'Porcentaje (%) RMS={esp_rms_hum}%')
    ax3.set_title('Variación de la Humedad atmosferica en el Tiempo')
    ax3.set_xlabel('Tiempo (segundos)')
    ax3.set_ylabel('Humedad (%)')
    
    ax4.plot(tiempo, lista_gas, marker='o', linestyle='-', color='b',  label=f'Ohms (Ω) RMS={esp_rms_gas}')
    ax4.set_title('Variación de la Resistencia en el Tiempo')
    ax4.set_xlabel('Tiempo (segundos)')
    ax4.set_ylabel('Resistencia (Ω)')

    plt.tight_layout()
    plt.show()


def solicitar_ventana_datos(progress_dialog):
    """Función para solicitar la ventana de datos"""
    global samples

    print("\n-- extraccion de datos --")

    _ = ser.write('s'.encode())
    data = ser.read(4)

    _ = ser.write('g'.encode())
    lista_temp = []
    lista_pres = []
    lista_hum=[]
    lista_gas=[]
    tiempo = list(range(samples))

    time.sleep(1.5)

    # Actualizar la barra de progreso
    for i in range(samples):
        data = ser.read(16)
        temp, pres, hum, gas = unpack("ffff", data)
        lista_temp.append(temp)
        lista_pres.append(pres)
        lista_hum.append(hum)
        lista_gas.append(gas)   
        print(f"temp: {temp:.5f}\tpres: {pres:.5f}" +
              f"\thum:{hum:.5f}%\tgas:{gas:.5f}")
        

        progress_dialog.setValue(i+1)
        if progress_dialog.wasCanceled():
            print("esperando a que termine la muestra")
            _ = ser.readall()
            return

    data = ser.read(16)
    esp_rms_temp, esp_rms_pres,esp_rms_hum,esp_rms_gas = unpack("ffff", data)

    print(f"\n-- resultados RMS esp --")
    print(f"temp: {esp_rms_temp}")
    print(f"pres: {esp_rms_pres}")
    print(f"hum: {esp_rms_hum} %")
    print(f"gas: {esp_rms_gas} ohm")

    data5peaks_temp = ser.read(20)
    data5peaks_pres = ser.read(20)
    data5peaks_hum = ser.read(20)
    data5peaks_gas = ser.read(20)
    
    peaks_temp = array.array('f', data5peaks_temp).tolist()
    peaks_pres = array.array('f', data5peaks_pres).tolist()
    peaks_hum = array.array('f', data5peaks_hum).tolist()
    peaks_gas = array.array('f', data5peaks_gas).tolist()
    
    print(f"\n-- resultados 5 peaks esp --")
    print(f"temp: {peaks_temp}")
    print(f"pres: {peaks_pres}")
    print(f"hum: {peaks_hum}")
    print(f"gas: {peaks_gas}")

    data_fft_real_temp = ser.read(4 * samples)
    data_fft_imag_temp = ser.read(4 * samples)
    fft_real_temp = array.array('f', data_fft_real_temp).tolist()
    fft_imag_temp = array.array('f', data_fft_imag_temp).tolist()

    data_fft_real_pres = ser.read(4 * samples)
    data_fft_imag_pres = ser.read(4 * samples)
    fft_real_pres = array.array('f', data_fft_real_pres).tolist()
    fft_imag_pres = array.array('f', data_fft_imag_pres).tolist()

    data_fft_real_hum = ser.read(4 * samples)
    data_fft_imag_hum = ser.read(4 * samples)
    fft_real_hum = array.array('f', data_fft_real_hum).tolist()
    fft_imag_hum = array.array('f', data_fft_imag_hum).tolist()

    data_fft_real_gas = ser.read(4 * samples)
    data_fft_imag_gas = ser.read(4 * samples)
    fft_real_gas = array.array('f', data_fft_real_gas).tolist()
    fft_imag_gas = array.array('f', data_fft_imag_gas).tolist()

    print(f"\n-- resultados fft esp --")
    print(f"temp: {fft_real_temp}")
    print(f"pres: {fft_real_pres}")
    print(f"hum: {fft_real_hum}")
    print(f"gas: {fft_real_gas}")
    
    print(f"\n-- resultados fft imag esp --")
    print(f"temp: {fft_imag_temp}")
    print(f"pres: {fft_imag_pres}")
    print(f"hum: {fft_imag_hum}")
    print(f"gas: {fft_imag_gas}")

    progress_dialog.close()

    print(ser.readall(), end="")

    generar_graficos(tiempo, 
                     lista_temp, esp_rms_temp, 
                     lista_pres, esp_rms_pres,
                     lista_hum, esp_rms_hum,
                     lista_gas, esp_rms_gas)

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
