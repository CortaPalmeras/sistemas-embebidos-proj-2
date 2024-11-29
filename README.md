# Proyecto 2
## CC5328 - Sistemas Embebidos y Sensores

Esta tarea fue probada en Debain 12, por lo que el puerto usado fue `/dev/ttyUSB0`, para cambiarlo, se debe modificar la variable `PORT` en el archivo `reciever.py`.

Para ejecutar la tarea es necesario cargar el programa en el ESP, ejecutando el comando:

```bash
idf.py flash
```

Luego se deben instalar las dependencias del programa de python:

```bash
pip install -r requirements.txt
```

Finalmente, se puede ejecutar la aplicación usando python:

```bash
python computador.py
```
