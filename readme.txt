Primero, se creó el paquete segundo_parcial con sus debidas dependencias. Dentro de la carpeta con el mismo nombre 
se crearon los nodos de python, sub_and_process_img.py para el primer punto y pub_segmentation.py para el segundo.
Aparte se creó una carpeta launch, que contiene los archivos primer_punto.launch.py y segundo_punto.launch.py, para 
ejecutar todos los nodos necesarios para cada punto del parcial (usb_cam_exe, static_transform_publisher, rviz2 y 
el ejecutable del nodo correspondiente). Para esto se tuvo que agregar dependencias adicionales de launch y una modificación en el setup.py, para leer estos archivos launch. Además de asignarle un nombre ejecutable a ambos nodos creados en el mismo archivo setup.py.

Para ambos nodos se utilizó la misma estructura de suscripción y publicación, en la que se obtiene el mensaje tipo 
Image, se convierte a cv2, se le realiza el procesamiento y se vuelve a convertir en mensaje tipo Image. Al cual se
le define los parámetros header.stamp con el valor del tiempo de adquisición de la imagen y el header.frame_id como
el marco óptico de la cámara. 

Por último, con la estructura del mensaje Image completo, se publica a un topic anteriormente definido por la clase
create_publisher.

