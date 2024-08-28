#include "LCD.h"

void initLCD8bit(void){
	DDRD |= (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7);
	PORTD &= ~(1<<PORTD2)|~(1<<PORTD3)|~(1<<PORTD4)|~(1<<PORTD5)|~(1<<PORTD6)|~(1<<PORTD7); 
	
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3);
	PORTB &= ~(1<<PORTB0)|~(1<<PORTB1)|~(1<<PORTB2)|~(1<<PORTB3);
	
	//LCD_CMD(0x00);
	_delay_ms(20);
	
	//Function set 8 bit 
	LCD_CMD(0x38);
	_delay_ms(5);
	/*LCD_CMD(0x38);
	_delay_ms(1);
	LCD_CMD(0x38);
	_delay_ms(1);
	
	LCD_CMD(0x38);*/
	
	//Display on/off
	//LCD_CMD(0x0C); => Poner E en ves de C habilita el cursor 
	LCD_CMD(0x0E);
	_delay_ms(2);
	
	//clear display
	LCD_CMD(0x01);
	
	_delay_ms(2);
	
	//Entry mode
	LCD_CMD(0x06);
	
}

void LCD_Port(char a){
	if(a & 1){
		//D0 = 1
		PORTD |= (1<<PORTD4);
	}
	else {
		//D0 = 0
		PORTD &= ~(1<<PORTD4);
	}
	if(a & 2){
		//D1 = 1
		PORTD |= (1<<PORTD5);
	}
	else {
		//D1 = 0
		PORTD &= ~(1<<PORTD5);
	}
	if(a & 4){
		//D2 = 1
		PORTD |= (1<<PORTD6);
	}
	else {
		//D2 = 0
		PORTD &= ~(1<<PORTD6);
	}
	if(a & 8){
		//D3 = 1
		PORTD |= (1<<PORTD7);
	}
	else {
		//D3 = 0
		PORTD &= ~(1<<PORTD7);
	}
	if(a & 16){
		//D4 = 1
		PORTB |= (1<<PORTB0);
	}
	else {
		//D4 = 0
		PORTB &= ~(1<<PORTB0);
	}
	if(a & 32){
		//D5 = 1
		PORTB |= (1<<PORTB1);
	}
	else {
		//D5 = 0
		PORTB &= ~(1<<PORTB1);
	}
	if(a & 64){
		//D6 = 1
		PORTB |= (1<<PORTB2);
	}
	else {
		//D6 = 0
		PORTB &= ~(1<<PORTB2);
	}
	if(a & 128){
		//D7 = 1
		PORTB |= (1<<PORTB3);
	}
	else {
		//D7 = 0
		PORTB &= ~(1<<PORTB3);
	}
}

void LCD_CMD(char a){
	
	PORTD &= ~(1<<PORTD2); //=> RS = 0
	LCD_Port(a); 
	
	PORTD |= (1<<PORTD3); //=> EN = 1
	_delay_us(1);
	PORTD &= ~(1<<PORTD3); //=> En = 0
	_delay_ms(4);
	
}

//Funci?n para enviar un caracter
void LCD_Write_CHAR(char c){
	
	PORTD |= (1<<PORTD2); //=> RS = 1 Dato en el puerto se interpreta como caracter (DATO)
	LCD_Port(c);
	
	PORTD |= (1<<PORTD3); //=> EN = 1
	_delay_us(1);
	PORTD &= ~(1<<PORTD3); //=> En = 0
	_delay_ms(2);
	
}

//Funci?n para enviar una cadena
void LCD_Write_String(char *a){
	int i;
	for (i = 0; a[i] != '\0'; i++)
	LCD_Write_CHAR(a[i]);
}

//Desplazamiento hacia la derecha
void Lcd_Shift_Right(void){
	LCD_CMD(0x1C);
}

//Desplazamiento hacia la izquierda
void Lcd_Shift_Left(void){
	LCD_CMD(0x18);
}

//Establecer el cursor 
void LCD_Set_Cursor(char c, char f){
	char temp;
	if (f == 1){
		temp = 0x80 + c -1; 
		/*z = temp >> 4; 
		y = temp & 0x0F;*/
		LCD_CMD(temp);
		 
	}else if( f == 2){
		temp = 0xC0 + c -1;
		LCD_CMD(temp);
	}
}

void LCD_Clear(void){
	LCD_CMD(0x01); // Enviar el comando para limpiar la pantalla
	_delay_ms(2); // Esperar el tiempo necesario para que el comando se complete
}