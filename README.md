# XC16 - Soldador de Estano
Proyecto soldador estano con punta JBC realizado con un pic24f64ga02 en Mplab XC16

EL proyecto busca implementar una soldador de estano usando como base una punta JBC (en algun momento la conseguire) y a partir de alli desarrollar todo el proyecto.
  El control de temperatura sera una PID implementado en un pic 24F64GA002.
  La etapa de potencia estara comformado por un driver para un mosfet P usado en el lado alto de la carga (hight side).
  La realimentacion se realiza a traves de dos entradas analogicas del PIC, una para la termocupla tipo k de la punta JBC y otra para medir la corriente.
  Para mostrar informacion se usara un display LCD16x4 en un principio con intencion de actualizarlo en algun momento a un Display OLed.
  El menu estara basado en una estacion de soldado Jabe o JBC.

El proyecto esta motivado porque siempre he usado soldadores tipo lapiz de bajo costo y deseo mejorar mis herramientas.
