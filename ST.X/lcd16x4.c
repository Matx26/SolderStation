/*
 * File:   lcd_16x4.c
 * Author: Electronica
 * Comments: en este archivo se declaraciones de las funciones que se usan para el LCD
 * Created on 13 de julio de 2018, 11:13
 */

// LCD 16x4 es compatible con los LCD 16x2

//Con estas definiciones se determina que puesrtos se usaran para
//para interconectar el display lcd y los pic16F
// y si o si se deben definir en el lcd_16x4.c antes de incluir a la libreria lcd_16x4.h en el codigo.c
/*
#define EN RB2
#define RS RB3
#define D4 RB4
#define D5 RB5
#define D6 RB6
#define D7 RB7
 

y para interconectar el display lcd y los pic18F y 24f
#define EN LATDbits.LATD2
#define RS LATDbits.LATD3
#define D4 LATDbits.LATD4
#define D5 LATDbits.LATD5
#define D6 LATDbits.LATD6
#define D7 LATDbits.LATD7 
*/

/* Para Pic16 y Pic18
 * Se debe incluir esta libreria lcd_16x4.h y lcd_16x4.c despues definir _XTAL_FREQ 16000000 ya que la 
 * libreria lcd_16x4.h hace uso de las funciones __delay_ms y __delay_us las 
 * cuales se vasa en esa definicion
 */

/* Para Pic24
 * se debe incluir esta libreria lcd_16x4.h y lcd_16x4.c despues de definir FCY e incluir <libpic30.h> 
 * ya que la libreria lcd_16x4.h hace uso de las funciones __delay_ms y 
 * __delay_us las cuales se vasan en esas definiciones 
 */



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

/*
//descomentar la opcion deseada
//#define _PIC16/18_Seting
#define _PIC24f_Seting

#ifdef _PIC24f_Seting
#define FCY 16000000UL      //definimos la velocidad a 16MIPS, UL=40bits (unsigned long)
                            //si el clock trabaja a 32Mhz y como cada siclo de instrucion
                            //se realiza cada 2 clocks la frecuencia de las instrucciones es de 16Mips 
#include <libpic30.h>
#endif 

#ifdef _PIC16/18_Seting
*/

#ifndef _PORT_SETING_

#define _PORT_SETING_

//configuracion de los puertos que se conectan al LCD, se deben definir antes de
//incluir a la libreria lcd_16x4.h y este es el lugar adecuado
#define EN LATBbits.LATB0
#define RS LATBbits.LATB1
#define D4 LATBbits.LATB2
#define D5 LATBbits.LATB3
#define D6 LATBbits.LATB4
#define D7 LATBbits.LATB5

//#endif /* fin de configuracion de los puertos*/


#include "xc.h"

#define FCY 16000000UL      //definimos la velocidad a 16MIPS, UL=40bits (unsigned long)
                            //si el clock trabaja a 32Mhz y como cada siclo de instrucion
                            //se realiza cada 2 clocks la frecuencia de las instrucciones es de 16Mips 
#include <libpic30.h>
#include "lcd16x4.h"


#endif /* fin de configuracion de los puertos*/

void Lcd_Port(char a)
{
	if(a & 1)
		D4 = 1;
	else
		D4 = 0;

	if(a & 2)
		D5 = 1;
	else
		D5 = 0;

	if(a & 4)
		D6 = 1;
	else
		D6 = 0;

	if(a & 8)
		D7 = 1;
	else
		D7 = 0;
}
void Lcd_Cmd(char a)
{
	RS = 0;             // => RS = 0 - indicamos que se ingresara a los registros de datos
	Lcd_Port(a);        // envia el comando de 4 bits al puerto
	EN  = 1;             // => E = 1, genera un pulso de 4ms para el comando sea aceptado
        __delay_ms(4);   // 
        EN  = 0;             // => E = 0
}

void Lcd_Clear() // comando clear: RS RW E D7 D6 D5 D4 D3 D2 D1 D0
            //                 0  0 0  0  0  0  0  0  0  0  1
{                       // como esta configurado con un bus de 4 bits
	Lcd_Cmd(0);         // primero envia los 4 bits mas significaticvos"0000")
	Lcd_Cmd(1);         // en segundo lugar envia los 4 bits menos signiticaftivos "0001")
}

void Lcd_Set_Cursor(char b, char a) // comando Set DDRAN: RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                                    //                     0  0 0  1  0 [Dirreccion DDran]
/*mapa de la memoria ddram
01..................27
40..................4F
10..................1F
50..................5F
*/
{
	char temp,z,y;
	if(a == 1)    // si a(eje y) es igual a 1         
	{
	  temp = 0x80 + b - 1; //Se le suma 10000000 el cual es del comando Set Ddram
                           //mas 00000000 ya que la fila uno comienza en la posicion
                            // 0x00 . (0x80+x00 = 0x80).     
		z = temp>>4;   // se desplaza 4 veces a la derecha para deajr los 4 bits
                       // mas significativos en los 4 menos significativos.     
		y = temp & 0x0F; // se borron los 4 bits mas significaticvos
		Lcd_Cmd(z);     //envia los 4 bits mas significativos
		Lcd_Cmd(y);     // envia los 4 bits menos significativos
	}
	else if(a == 2)     // si a(eje y) es igual a 2
	{
		temp = 0xC0 + b - 1;//Se le suma 10000000 el cual es del comando Set Ddram
                            // mas 01000000 ya que la fila dos comienza en la posicion
                            // 0x40 . (0x80+x40 = 0xC0)     
		z = temp>>4;
		y = temp & 0x0F;
		Lcd_Cmd(z);
		Lcd_Cmd(y);
	}
    else if(a == 3)       // si a(eje y) es igual a 3
	{
		temp = 0x90 + b - 1;//Se le suma 10000000 el cual es del comando Set Ddram
                            // mas 00010000 ya que la fila tres comienza en la posicion
                            // 0x10 . (0x80+x10 = 0x90)
		z = temp>>4;
		y = temp & 0x0F;
		Lcd_Cmd(z);
		Lcd_Cmd(y);
	}
    else if(a == 4)         // si a(eje y) es igual a 4
	{
		temp = 0xD0 + b - 1;//Se le suma 10000000 el cual es del comando Set Ddram
                            // mas 01010000 ya que la fila cuatro comienza en la posicion
                            // 0x40 . (0x80+x50 = 0xD0)
		z = temp>>4;
		y = temp & 0x0F;
		Lcd_Cmd(z);
		Lcd_Cmd(y);
	}
}

void Lcd_Init() // inicializa el display     
{
  Lcd_Port(0x00);   // una ves energizado el display se debe se ponen todos los bits                     
   __delay_ms(20);  // a cero y se espera mas de 15ms
  Lcd_Cmd(0x03);    // comando Funtion Set RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                      0  0 0  0  0  1  1 nc nc nc nc (nc = no se conecta)
                    //Bit4(D/L)=1 indica bus de 8 bits
__delay_ms(5);      // esperamos mas de 4,1ms
  Lcd_Cmd(0x03);    // comando Funtion Set RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                      0  0 0  0  0  1  1 nc nc nc nc (nc = no se conecta)
                    //Bit4(D/L)=1 indica bus de 8 bits
	__delay_ms(1); // esperamso mas de 100us
  Lcd_Cmd(0x03);    // comando Funtion Set RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                      0  0 0  0  0  1  1 nc nc nc nc (nc = no se conecta)
                    //Bit4(D/L)=1 indica bus de 8 bits
  /////////////////////////////////////////////////////////////////////////////
  Lcd_Cmd(0x02);    //recian ahora podemos configurar el bus de 4 bits
                    // comando Funtion Set RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                      0  0 0  0  0  1  0 nc nc nc nc (nc = no se conecta)
                    //Bit4(D/L)=0 indica bus de 4 bits
  
  Lcd_Cmd(0x02);    // 1ero 4 bits mas sifnificativos (0010)
  Lcd_Cmd(0x08);    // 2do 4 bits menos significativos (1000)
                    // comando Funtion Set RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                      0  0 0  0  0  1  0  1  0  0  0
                    //Bit4(D/L)=0 indica bus de 4 bits
                    //Bit3(N)=1 indica dos linea (16x4) o 4 lineas (16x4)
                    //Bit2(F)=0 caracter 5x7                    
  
  Lcd_Cmd(0x00);    // 1ero 4 bits mas sifnificativos (0000)
  Lcd_Cmd(0x0C);    // 2do 4 bits menos significativos (1100)
                    // comando Display ON/OFF RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                         0  0 0  0  0  0  0  1  1  0  0
                    //Bit2(D)=1 display ON
                    //Bit1(C)=0 cursor OFF
                    //Bit0(B)=0 parpadeo del cursor OFF  
  
  Lcd_Cmd(0x00);    // 1ero 4 bits mas sifnificativos (0000)
  Lcd_Cmd(0x04);    // 2do 4 bits menos significativos (0110)
                    // comando Entry Mode Set RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                         0  0 0  0  0  0  0  0  1  0  0
                    //Bit1(I/D)=0 decremneto del curso automatico
                    //Bit0(S)=0 modo normal (sin desplazamiento del display virtual)  
  
}

void Lcd_Write_Char(char a)
                    // comando Data Write to DDram o CGram
                    //                     RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                    //                      1  0 0 (codigo ascii         )
{
   char temp,y;     
   temp = a&0x0F;      //borra lso 4 bit mas sign para resaltar los 4 menos sign  
   y = a&0xF0;         //borra lso 4 bit menos sign para resaltar los 4 mas sign    
   RS = 1;             // => RS = 1 - indicamos que se ingresara a los registro de instruccion
   Lcd_Port(y>>4);     //enviamos los 4 bits mas significativos
   EN = 1;             // generamso unpulso de 40ms en el eneble (E)
   __delay_us(40);      // para confirmar el comando
   EN = 0;
   Lcd_Port(temp);      //enviamos los 4 bits menos significativos
   EN = 1;              // generamso unpulso de 40ms en el eneble (E)
   __delay_us(40);      // para confirmar el comando
   EN = 0;
}

void Lcd_Write_String(char *a)
{
	int i;
	for(i=0;a[i]!='\0';i++)
	   Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right() // comando Cursor/Display shift 
                       //                     RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                       //                      0  0 0  0  0  0  1  1  1  x  x
                       //Bit3(S/C)=1 desplaza el display
                       //Bit2(R/L)=1 desplaza a la derecha
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left() // comando Cursor/Display shift 
                       //                     RS RW E D7 D6 D5 D4 D3 D2 D1 D0
                       //                      0  0 0  0  0  0  1  1  0  x  x
                       //Bit3(S/C)=1 desplaza el display
                       //Bit2(R/L)=0 desplaza a la izquierda
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x08);
}

