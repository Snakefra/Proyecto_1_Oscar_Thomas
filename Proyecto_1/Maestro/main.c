/*Universidad del Valle de Guatemala 
 * main.c
 * IE3054 Electronica Digital 2
 *	Proyecto Red de sensores
 * Created: 20/08/2024 13:01:49
 *  Author: Thomas 21626 - Oscar Melchor 
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "LCD/LCD.h"

// Definir las direcciones de los esclavos
#define SLAVE_ADDRESS_HUMIDITY 0x09
#define SLAVE_ADDRESS_RAIN 0x0B
#define AHT10_ADDRESS 0x38

// Estado para controlar qué se muestra en la pantalla
volatile uint8_t display_option = 0;  // 0: Humedad, 1: Temperatura, 2: Lluvia

// Función para iniciar la comunicación I2C como Maestro
void i2c_init_master() {
	TWSR = 0x00; // Prescaler = 1
	TWBR = 0x48; // SCL frequency is 100K for 16MHz
}

// Función para enviar un START condition
void i2c_start() {
	TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT); // Enviar START
	while (!(TWCR & (1<<TWINT))); // Esperar a que se complete
}

// Función para enviar un STOP condition
void i2c_stop() {
	TWCR = (1<<TWSTO) | (1<<TWEN) | (1<<TWINT); // Enviar STOP
	while (TWCR & (1<<TWSTO)); // Esperar a que se complete
}

// Función para escribir un byte en I2C
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1<<TWEN) | (1<<TWINT); // Iniciar transmisión
	while (!(TWCR & (1<<TWINT))); // Esperar a que se complete
}

// Función para leer un byte con ACK
uint8_t i2c_read_ack() {
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA); // Habilitar ACK
	while (!(TWCR & (1<<TWINT))); // Esperar a que se complete
	return TWDR;
}

// Función para leer un byte con NACK
uint8_t i2c_read_nack() {
	TWCR = (1<<TWEN) | (1<<TWINT); // No ACK
	while (!(TWCR & (1<<TWINT))); // Esperar a que se complete
	return TWDR;
}

// Configuración UART
void uart_init(unsigned int baudrate) {
	unsigned int ubrr = F_CPU/16/baudrate-1;
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0) | (1<<TXEN0); // Habilitar transmisión y recepción
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // Configurar frame de 8 bits
}

void uart_transmit(unsigned char data) {
	while (!(UCSR0A & (1<<UDRE0))); // Esperar a que el buffer esté vacío
	UDR0 = data; // Enviar el dato
}

void uart_send_string(const char* str) {
	while(*str) {
		uart_transmit(*str++);
	}
}

void uart_send_number(uint16_t number) {
	char buffer[10];
	itoa(number, buffer, 10); // Convertir número a cadena
	uart_send_string(buffer);
}

void uart_send_float(float number) {
	char buffer[10];
	dtostrf(number, 5, 2, buffer); // Convertir número flotante a cadena con 2 decimales
	uart_send_string(buffer);
}

// Función para leer la temperatura desde el sensor AHT10
float readTemperature() {
	uint8_t data[6];

	i2c_start();
	i2c_write(AHT10_ADDRESS << 1); // Write address
	i2c_write(0xAC); // Command for measure
	i2c_write(0x33);
	i2c_write(0x00);
	i2c_stop();

	_delay_ms(80); // Wait for the measurement to complete

	i2c_start();
	i2c_write((AHT10_ADDRESS << 1) | 1); // Read address

	for (int i = 0; i < 5; i++) {
		data[i] = i2c_read_ack();
	}
	data[5] = i2c_read_nack();
	i2c_stop();

	// Convertir los datos a temperatura
	uint32_t tempRaw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
	float temperature = ((float)tempRaw * 200 / 1048576) - 50;

	return temperature;
}

// Función para solicitar el valor del sensor de humedad del esclavo
uint16_t request_humidity() {
	uint16_t humidity = 0;
	uint8_t msb, lsb;

	i2c_start();
	i2c_write((SLAVE_ADDRESS_HUMIDITY << 1) | 1); // Dirección del esclavo con bit de lectura

	msb = i2c_read_ack(); // Leer MSB con ACK
	lsb = i2c_read_nack(); // Leer LSB con NACK

	i2c_stop();

	humidity = ((uint16_t)msb << 8) | lsb;
	return humidity;
}

// Función para solicitar el valor del sensor de lluvia del esclavo
uint16_t request_rain() {
	uint16_t rain_value = 0;
	uint8_t msb, lsb;

	i2c_start();
	i2c_write((SLAVE_ADDRESS_RAIN << 1) | 1); // Dirección del esclavo con bit de lectura

	msb = i2c_read_ack(); // Leer MSB con ACK
	lsb = i2c_read_nack(); // Leer LSB con NACK

	i2c_stop();

	rain_value = ((uint16_t)msb << 8) | lsb;
	return rain_value;
}

// Función para mostrar el estado de lluvia en la LCD
void displayRain(uint16_t rain) {
	char buffer[16];

	if (rain < 700) {
		sprintf(buffer, "Mojado");
		} else if (rain > 1000) {
		sprintf(buffer, "Seco");
		} else {
		sprintf(buffer, "Intermedio");
	}

	LCD_Clear();
	LCD_Set_Cursor(1, 1);
	LCD_Write_String("Lluvia:");
	LCD_Set_Cursor(2, 1);
	LCD_Write_String(buffer);
}

// Función para mostrar el estado de humedad en la LCD
void displayHumidity(uint16_t humidity) {
	char buffer[16];

	if (humidity < 700) {
		sprintf(buffer, "Humedad");
		} else if (humidity > 1000) {
		sprintf(buffer, "SECO");
		} else {
		sprintf(buffer, "Intermedio");
	}

	LCD_Set_Cursor(2, 1); // Asumiendo que deseas mostrarlo en la segunda línea
	LCD_Write_String("Humedad: ");
	LCD_Write_String(buffer);
}

// Función para mostrar la temperatura en la LCD
void displayTemperature(float temperature) {
	char buffer[16];

	sprintf(buffer, "Temp: ");
	LCD_Clear();
	LCD_Set_Cursor(1, 1);
	LCD_Write_String(buffer);

	dtostrf(temperature, 5, 2, buffer);
	LCD_Write_String(buffer);
	LCD_Write_String(" C");
}

// Función para leer el estado del botón
uint8_t readButton() {
	if (!(PINC & (1 << PINC1))) { // Si el botón en A1 es presionado (LOW)
		_delay_ms(50); // Desacoplar rebotes
		if (!(PINC & (1 << PINC1))) {
			return 1; // Botón presionado
		}
	}
	return 0; // Botón no presionado
}

// Función para alternar entre mostrar humedad, temperatura o lluvia
void toggleDisplay() {
	if (readButton()) {
		display_option = (display_option + 1) % 3;  // Alternar entre 0 (humedad), 1 (temperatura), 2 (lluvia)
		while (readButton()); // Esperar a que el botón sea liberado
	}
}

int main(void) {
	// Inicializar I2C como Maestro
	i2c_init_master();

	// Inicializar UART
	uart_init(9600); // Inicializar UART a 9600 baudios

	// Inicializar LCD y mostrar mensaje de inicio
	initLCD8bit();
	LCD_Clear();
	LCD_Write_String("Iniciando...");
	_delay_ms(2000);

	// Configurar el pin A0 como salida para el relé
	DDRC |= (1 << DDC0); // Configura A0 como salida
	PORTC &= ~(1 << PORTC0); // Inicialmente el pin en LOW

	// Configurar el pin A1 como entrada para el botón
	DDRC &= ~(1 << DDC1); // Configura A1 como entrada
	PORTC |= (1 << PORTC1); // Habilitar pull-up interno

	while(1) {
		toggleDisplay(); // Verificar si el botón fue presionado

		switch (display_option) {
			case 0: {
				uint16_t humidity = request_humidity();
				displayHumidity(humidity);

				// Enviar la humedad al ESP32 por UART
				uart_send_string("H:");
				uart_send_number(humidity);  // Enviar el valor de la humedad
				uart_send_string("\r\n");

				break;
			}
			case 1: {
				float temperature = readTemperature(); // Obtener temperatura del esclavo
				displayTemperature(temperature);

				// Enviar la temperatura al ESP32 por UART
				uart_send_string("T:");
				uart_send_float(temperature);  // Enviar el valor de la temperatura
				uart_send_string("\r\n");

				// Lógica para controlar el relé basado en la temperatura
				if (temperature >= 27.0) {
					PORTC |= (1 << PORTC0); // Establece A0 en HIGH
					} else if (temperature <= 25.0) {
					PORTC &= ~(1 << PORTC0); // Establece A0 en LOW
				}
				break;
			}
			case 2: {
				uint16_t rain = request_rain();
				displayRain(rain);

				// Enviar la lluvia al ESP32 por UART
				uart_send_string("L:");
				uart_send_number(rain);  // Enviar el valor de la lluvia
				uart_send_string("\r\n");

				break;
			}
		}

		_delay_ms(500); // Espera antes de la siguiente lectura
	}
}