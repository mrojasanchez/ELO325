
<!-- Referencia python -->
<!-- https://blog.sravjti.in/2021/06/26/using-python-api-in-coppeliasim.html -->

## Links utiles
[CoppeliaSim API](https://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm)

[Intro a Coppelia (en Lua)](https://www.youtube.com/watch?v=l32IiRkCwxg&list=PLjzuoBhdtaXOYfcZOPS98uDTf4aAoDSRR&index=1) -> Aunque esté en Lua, las funciones son similares tanto para Python y Matlab.

# Opción A: Windows + Matlab
### Prerequisitos
1. Descargar e instalar Matlab
2. Descargar e instalar CoppeliaSim ([link](https://www.coppeliarobotics.com/downloads))

## Configuración y ejecución de ejemplo
1. Para comunicar CoppeliaSim con Matlab, debemos tener siempre los siguientes archivos:
 ![alt text](img/copiar.png)

2. Estos archivos deben pegarse en la carpeta donde trabajemos:
 ![alt text](img/pegar.png)

3. Ejecutamos CoppeliaSim y creamos un script en el objeto definido como "Floor":
 ![alt text](img/lua_script_win.png)

4. Luego editamos dicho script, agregando la siguiente línea de código:
 ![alt text](img/setRemoteApi.png)

5. Si todo está correcto, al ejecutar el simulador y correr el script de Matlab debiese imprimirse la posicion del puntero en el simulador:

https://github.com/mrojasanchez/ELO325/assets/49776685/3df38a0d-cdf1-401b-b02e-d091bdd573f6

6. Si ahora queremos comunicarnos con el robot Pionner, podemos utilizar la libreria disponible en CoppeliaSim. Cabe señalar que se debe desactivar el script que viene por defecto en el robot dado que el [script de Matlab](https://github.com/mrojasanchez/ELO325/blob/master/ayudantia_01/matlab/pioneerExample.m) es el que tomará control. NOTA: recordar que el script de Matlab debe residir en la misma carpeta donde se encuentren los archivos `remApi.m` , `remoteApi.dll` , `remoteApiProto.m` y `simpleSynchronousTest.m`.

https://github.com/mrojasanchez/ELO325/assets/49776685/e0c80cbf-e727-4256-81da-ceb66723cf38

# Opción B: Ubuntu 20.04 + Python
### Prerequisitos
1. Miniconda ([link](https://docs.conda.io/en/latest/miniconda.html))
2. Descargar CoppeliaSim ([link](https://www.coppeliarobotics.com/downloads))
3. Visual Studio Code (Opcional) ([link](https://code.visualstudio.com/download))

## Crear ambiente de python
```
conda create -n ELO325
conda activate ELO325
conda install -c conda-forge opencv
```
## Configuración de Coppeliasim + Python 3
1. Para mantener organizado el espacio de trabajo, crearemos una carpeta donde ubicaremos el programa CoppeliaSim y los ejemplos.
 	```
    mkdir ~/coppeliasim_ws
    ``` 
2. Una vez ubicado CoppeliaSim en nuestro espacio de trabajo, debemos indicarle a python donde debe buscar las librerias de CoppeliaSim. En este caso, las librerias de python se encuentran ubicadas en: 

    `${HOME}/copeliasim_ws/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04/programming/legacyRemoteApi/remoteApiBindings/python/python`

3. Habiendo confirmado la ubicación, agregamos una nueva ruta a PYTHONPATH, para esto agregamos al final del archivo `~/.bashrc`
    ```
    export PYTHONPATH="${HOME}/copeliasim_ws/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04/programming/legacyRemoteApi/remoteApiBindings/python/python"
    ```
4. Para que los cambios surjan efecto, cerramos y abrimos terminal o ejecutamos en el mismo terminal 

    ```
    source ~/.bashrc
    ```

5. Luego, abrimos el archivo `sim.py` ubicado en la misma direccion y modificamos la siguiente linea con la dirección del archivo `remoteApi.so` ubicado en:

    `/home/<USER>/copeliasim_ws/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04/programming/legacyRemoteApi/remoteApiBindings/lib/lib/Ubuntu20_04`

    ![alt text](img/drawing0.svg)

6. Ahora, python debería ser capaz de importar la libreria de CoppeliaSim. Para comprobar, abrimos un terminal y ejecutamos:
    
    ![alt text](img/testing.png)

## Comunicación CoppeliaSim y Python

1. Abrimos un terminal y ejecutamos el siguiente archivo shell:

    ![alt text](img/bash_coppelia.png)

2. Dentro de la interfaz gráfica, nos dirigimos a [Model browser -> robots -> mobile] y en la ventana inferior arrastramos el modelo "pionner p3dx.ttm"

    ![alt text](img/pioneer.png)

3. Deshabilitamos el script que trae el modelo tal como muestra la imagen siguiente:

    ![alt text](img/disable_script.png)

4. Y haciendo click derecho en cualquier otro objeto (en este caso hacemos click derecho en el objeto Floor), generamos un script de Lua tal como muestra la imagen:

    ![alt text](img/lua_script_0.png)

5. Abrimos el script y colocamos las dos lineas marcadas como muestra la imagen siguiente:

    ![alt text](img/lua_script_1.png)

6. Cerramos la ventana del script, y procedemos a crear un archivo de python

    ![alt text](img/ejemplo_python.png)

7. Ejecutamos el simulador y luego corremos el codigo de python





    
