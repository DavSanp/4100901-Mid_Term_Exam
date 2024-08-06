# 4100901-Mid_Term_Exam
Please fork this repo, then clone it into your computer, and place your work for the Mid term exam here.

## Criterio de Evaluación 

* Funcionalidad: 60% (valor equivalente para cada requerimiento)
* Repositorio: 20% (commits entendibles que describan los cambios realizados)
* Documentación: 20% (en código y readme del repositorio)


## Instrucciones

Implemente un sistema de señales direccionales como se especifica.

### Requerimientos del sistema:

#### No funcionales:
1. Tener 2 botones: Giro Izquierda, Giro Derecha.
2. Tener 2 luces(LEDs): Luz Izquierda, Luz Derecha.
3. Tener un puerto de depuración con el PC: USART2

#### Funcionales:
4. Debe enviar mensajes de información por la consola cuando hayan eventos en el sistema.
5. Si un botón de giro se presiona 1 vez: la luz del lado correspondiente parpadea 3 veces.
6. Si un botón de giro se presiona 2 o mas veces durante 500ms: la luz del lado correspondiente parpadea indefinidamente.
7. Si un botón de giro se presiona y la luz del otro lado esta activa: se desactiva la luz del otro lado.
8. La frecuencia de parpadeo de las luces debe ser de 2Hz.

Basándonos en el ejercicio realizado en el simulacro, se realizaron algunas modificaciones y se añadieron nuevas características para un funcionamiento adicional. Este se centra en un sistema de direccionales similar al ejercicio anterior. Sin embargo, en esta ocasión se agregó una función donde, si se presiona dos veces o durante un tiempo determinado, las luces quedan parpadeando indefinidamente.