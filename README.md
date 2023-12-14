ARQUITECTURA DE PROGRAMACIÓN PARA EL CONTROL DE HARDWARE
La aplicación tiene los siguientes elementos:
• Sensores S_IN entrada y S_OUT de salida para el conteo de las personas que entran y salen, y así determinar el número que permanece adentro (dentro del límite establecido).
• Sensor de temperatura corporal TEMCOR para el acceso
• Actuador o cerradura eléctrica para permitir acceso DOOR
• Sensor de temperatura ambiental TEMPAMB
• Abanicos FAN

1. Primero el sistema apagado debe indicar Sistema: OFF, si el sistema se enciende indica en terminal
Sistema:ON (al principio) y prende un LED indicador.
La puerta marcará DOOR: closed.

2. Si una persona es detectada por S_IN, y si hay espacio y su temperatura es normal, abrirá la puerta para que entre, marcará DOOR: open por 5 segundos y la contará.
      a. Si una persona es detectada por S_IN, su temperatura es normal y no hay espacio, mostrará: We are Full, wait.
      b. Si una persona es detectada por S_IN, su temperatura es fuera de rango, por 5 seg emitirá la alarma (secuencia luces rojo-azul) y mostrará: Temp_out_of_range
         Si una persona es detectada por S_OUT, se descontará de la cuenta.
4. También tiene un control de temperatura del establecimiento: tiene un botón MODO para seleccionar AUTO/ ON para el encendido de FAN.
tiene un botón COOL para seleccionar cool/ heat.
      a. En Modo AUTO y cool, si TEMPAMB está por encima del set point entonces FAN:
On, de lo contrario Off
      b. En Modo AUTO y heat, si TEMPAMB está por abajo del set point entonces FAN: On
de lo contrario Off
      c. Modo On y cool, entonces FAN: On, de lo contrario Off
