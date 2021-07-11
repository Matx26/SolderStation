/* 
 * File: lcd_16x4.h
 * Author: Luna Matias
 * Comments: Archivo de cabecera en donde se definen los prototipos de las fun
 *          ciones que se usan por el modulo LCD 16x4, sus declaraciones estan
 *          en el archivo lcd_16x4.c
 * Revision history: 
 */

// LCD 16x4 es compatible con los LCD 16x2


/*Funciones:
*Lcd_Cmd(); envia el comando de 4 bits a la ves ya que esta configurado
                 para un buss de datos de 4 bits (Ver detalles en la funcion).

 *Lcd_Clear(); Limpia el display, retorna el cursor al origen y si el display
                    esta desplazado lo retorna tabien al origen
                     (Ver detalles en la funcion).
                    
 *Lcd_Set_Cursor(x,y); Desplaza el cursor el cursor a la posicion de memoria
                            (x=columna,y=fila) (Ver detalles en la funcion).
 
 *Lcd_Init(); Iniciliza el display (Ver detalles en la funcion).
 
 *Lcd_Write_Char(); escribe un caracter en el display en la posicion en la que
                    este el cursor (Ver detalle en la funcion).
 
 *Lcd_Write_String(); escribe un string en el display en la posicion en la que
                    este el cursor (Ver detalle en la funcion).
  
 *Lcd_Shift_Right(); desplaza el display virtual a la derecha.
 *Lcd_Shift_Left(); 
*/


void Lcd_Port(char a);
void Lcd_Cmd(char a);
void Lcd_Clear();
void Lcd_Set_Cursor(char b, char a);
void Lcd_Init();
void Lcd_Write_Char(char a);
void Lcd_Write_String(char *a);
void Lcd_Shift_Right();
void Lcd_Shift_Left();

