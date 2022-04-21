/***************************************************************************************************/
; FIUBA - (86.07) Laboratorio de Microprocesadores - 2do Cuatrimestre 2021
; Trabajo Práctico Integrador
; Versión interna: 06 - Versión entregada: 02
;
; Autores: 	Nimo, Luciana	
;			      Spaltro, Francisco
;
; Fecha de entrega: 27/12/21
;
; Programa que calcula el valor de un capacitor desconocido y muestra en una PC el valor
; obtenido y su recta de carga.
; /***************************************************************************************************/

.include "m328pdef.inc"

/***************************************************************************************/
; DEFINICIÓN DE CONSTANTES
/***************************************************************************************/

.equ	VIN_PORT_DIR				= DDRB
.equ	VIN_PORT					= PORTB
.equ	VIN_PIN						= PINB3

.equ	VOUT_PORT_DIR				= DDRC
.equ	VOUT_PORT					= PORTC
.equ	VOUT_PIN					= PINC0

.equ	LED_PORT_DIR				= DDRB
.equ	LED_PORT					= PORTB
.equ	LED_PIN						= PINB0

.equ	F_CPU						= 16000000				; Frecuencia
.equ	BAUD						= 9600					; baudrate
.equ	BPS							= ((F_CPU/16/baud) - 1)	; baud prescale
.equ	SEP_C_VALUE					= '-'					; 0x2D
.equ	END_FLAG					= '*'
.equ	END_CHAR					= '&'					; 0x26
.equ	SEP_CHAR					= ','					; 0x2C
.equ	TRUE						= 1
.equ	FALSE						= 0
.equ	ADC_LIMIT					= 649	;63.3% 5 V = 3.165 V -> 3.165 V * 1024/5V = 648.192 -> 649
.equ	R_VALUE						= 160
.equ	VOLTAGE_RAM_END				= 0x06DC

; Errores (OOR == Out of range)
.equ	STATUS_OOR_TABLE			= 0xAA					; este es un error que no impide calcular C
.equ	STATUS_OOR_TAU				= 0xFF					; este es un error terminal
.equ	MAX_STATUS					= 4
/***************************************************************************************/
; DEFINICIÓN DE ALIAS
/***************************************************************************************/

.def	zero						= r0
.def	one							= r1

.def	div_counter_high			= r2
.def	div_counter_mid				= r3
.def	div_counter_low				= r4

.def	r_high						= r5
.def	r_mid						= r6
.def	r_low						= r7

.def	aux							= r16
.def	aux_counter					= r17

.def	timer0_counter				= r18
.def	timer1_counter				= r19

.def	data_low					= r20
.def	data_mid					= r21
.def	data_high					= r22

.def	bool_conversion_end			= r23
.def	bool_terminate				= r24
.def	bool_out_of_range			= r25

/***************************************************************************************/
; INICIO DEL SEGMENTO DE DATOS
/***************************************************************************************/

.dseg

.org	SRAM_START

voltage_RAM:	.byte	1500

/***************************************************************************************/
; INICIO DEL SEGMENTO DE CÓDIGO
/***************************************************************************************/

.cseg

.org 0x0000
	rjmp init_micro

/***************************************************************************************/
; INTERRUPCIONES
/***************************************************************************************/
.org	ADCCaddr								; Para guardar el dato cuando termine la conversión
	rjmp	Handler_ADC_Conversion	
								
.org	OVF0addr								; Para solucionar problemas con el auto trigger
	rjmp	Handler_Timer0_Overflow

.org	OVF1addr								; Para calcular tiempo de 63%
	rjmp	Handler_Timer1_Overflow

.org	UDREaddr								; Para la transmisión de los valores al final
	rjmp	Handler_TX

.org INT_VECTORS_SIZE

init_micro:
	/*Inicializamos el Stack Pointer*/
	ldi		aux, high(RAMEND)
	out		sph, aux
	ldi		aux, low(RAMEND)
	out		spl, aux
	
	clr		zero
	ldi		aux, 1
	mov		one, aux

	ldi		bool_conversion_end, FALSE		; Indica si termino la conversión de todos los valores
	ldi		bool_out_of_range, FALSE		; Indica si la tabla se fue de rango
	ldi		bool_terminate, FALSE			; Indica si el capacitor es demasiado grande

	rcall	config_ports
	rcall	config_serial_TX
	rcall	config_timer0					; Arranca apagado
	rcall	config_timer1					; Arranca apagado
	rcall	config_ADC

	sei
	
	;le damos 4 segundos para descargarse en caso de que esté cargado,
	;finalice la primera conversión "ficticia" del ADC y el usuario abra la terminal serie
	sbi		LED_PORT, LED_PIN	; Agregamos un LED en el pin 0 del puerto B 
								; que indica la descarga del capacitor
	rcall	delay_4s
	cbi		LED_PORT, LED_PIN

	rcall	init_RAM

	sbi		VIN_PORT, VIN_PIN	; Iniciamos la carga del capacitor
	rcall	start_timer1
	rcall	start_timer0
	
main:
	; Validamos el rango
	cpi		bool_terminate, TRUE
	breq	terminate
	; Esperamos a que finalice la conversion total
	cpi		bool_conversion_end, TRUE
	brne	main

	; Al llegar a este punto: el ADC terminó y se detuvieron los timers 0 y 1.
	; Calculamos y agregamos datos en RAM
	rcall	cap_discharge
	rcall	add_sep_char
	rcall	calculate_tau

	rcall	calculate_c_value

	rcall	add_c_value
	rcall	add_final_char

	; Transmito todo. Formato:	[tensión 1], [tensión 2], ..., [tensión n]-[valor de C]&
	rcall	begin_TX

end:
	rjmp	end

terminate:
	rcall	cap_discharge
	rcall	oor_terminate_msg
	rcall	begin_TX

end2:
	rjmp	end2

/***************************************************************************************/
;									SUBRUTINAS
/***************************************************************************************/

/***************************************************************************************/
; CONFIG_PORTS
; Configuración de los puertos de entrada/salida necesarios
; PB3 es el pin de carga del circuito RC (5V) -> SALIDA (1)
; PC0 es el pin que mide la tensión del capacitor (Vc) -> ENTRADA (0)
; PBO es el pin que enciende el LED -> SALIDA (1)
/***************************************************************************************/
config_ports:
	sbi		VIN_PORT_DIR, VIN_PIN				; Salida para el pin de carga
	cbi		VOUT_PORT_DIR, VOUT_PIN				; Entrada para el pin de lectura

	sbi		LED_PORT_DIR, LED_PIN				; Salida para el LED

	ret

;***********************************************************************************
; config_UART
; Habilitar la transmisión UART 8N17 con el baud rate prescale.
; UMSEL00: modo USART, UPM00: bit 0 de paridad, USBS0: bit de stop, UCSZ00: tamaño
;***********************************************************************************
config_serial_TX:
	ldi		aux, low(BPS)
	sts		UBRR0L, aux
	ldi		aux, high(BPS)
	sts		UBRR0H, aux
	
	lds		aux, UCSR0C
	ori		aux, (1 << UCSZ01) | (1 << UCSZ00)
	andi	aux, ~((1 << UMSEL01) | (1 << UMSEL00) | (1 << UPM01) | (1 << UPM00) | (1 << USBS0))
	sts		UCSR0C, aux
	
	; Para congifurar más bits por caracter:
	; usar UCSZ02 --> en este caso no necesitamos pero lo ponemos en 0 explícitamente
	lds		aux, UCSR0B
	ori		aux, 1 << TXEN0
	andi	aux, ~(1 << UCSZ02)
	sts		UCSR0B, aux

	ret

;***********************************************************************************
; delay_4s
;***********************************************************************************
delay_4s:
	
	clr		aux
	out		TCNT0, aux

	in		aux, TCCR0B
	ori		aux, (1 << CS02) | (1 << CS00)	; prescaler 1024
	andi	aux, ~((1 << FOC0A) | (1 << FOC0B) | (1 << WGM02) | (1 << CS01))
	out		TCCR0B, aux

	clr		timer0_counter
	; timer0_counter cuenta la cantidad de veces que ocurre un OVF del timer
	; cada vez que ocurre un OVF pasaron 255*64us = 16.32ms
esperar:
	cpi		timer0_counter, 0xFF	; si esperamos 255 OVF habrán pasado 255*16.32ms = 4.162segundos
	brne	esperar	

	; Apagamos el timer 0
	in		aux, TCCR0B
	andi	aux, ~((1 << FOC0A) | (1 << FOC0B) | (1 << WGM02) | (1 << CS02) | (1 << CS01) | (1 << CS00))
	out		TCCR0B, aux

	clr		aux
	out		TCNT0, aux

	ret

;***********************************************************************************
; cap_discharge
; Pone en 0 el pin conectado al capacitor, haciendo un corto
;***********************************************************************************

cap_discharge:
	cbi		VIN_PORT, VIN_PIN
	ret

;***********************************************************************************
; config_timer0
;***********************************************************************************
config_timer0:
	; Modo Normal
	in		aux, TCCR0A
	andi	aux, ~((1<<COM0A1)|(1<<COM0A0)|(1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1 << WGM00))
	out		TCCR0A, aux

	; Habilit interrupción por overflow
	lds		aux, TIMSK0
	ori		aux, (1 << TOIE0)
	andi	aux, ~((1 << OCIE0B) | (1 << OCIE0A))
	sts		TIMSK0,aux

	; Arranca desactivado
	in		aux, TCCR0B
	andi	aux, ~((1 << FOC0A) | (1 << FOC0B) | (1 << WGM02) | (1 << CS02) | (1 << CS01) | (1 << CS00))
	out		TCCR0B, aux

	; Empieza en 0
	clr		aux
	out		TCNT0, aux

	ret

;***********************************************************************************
; config_timer1
;***********************************************************************************
config_timer1:
	; Modo de operación normal
	lds		aux, TCCR1A
	andi	aux, ~((1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM11)|(1<<WGM10))
	sts		TCCR1A, aux
	
	; Empieza apagado
	lds		aux, TCCR1B
	andi	aux, ~((1<<ICNC1)|(1<<ICES1)|(1<<WGM13)|(1<<WGM12)|(1<<CS12)|(1<<CS11)|(1<<CS10))
	sts		TCCR1B, aux

	; Activo la interrupción por overflow (tengo que llevar el recuento)
	lds		aux, TIMSK1
	ori		aux, (1 << TOIE1)
	andi	aux, ~((1 << OCIE1A) | (1 << OCIE1B) | (1 << ICIE1))
	sts		TIMSK1, aux

	; Empieza en 0
	clr		aux
	sts		TCNT1H, aux
	sts		TCNT1L, aux

	ret
	
start_timer0:
	; Una conversión del ADC (no la primera) tarda 108us --> con el timer0 en OV y
	; prescaler en 8 le damos 255/(16MHz/8) = 127.5 us
	in		aux, TCCR0B
	ori		aux, (1 << CS01)	; prescaler 8 --> un CM del timer 0 son 8/16MHz = 500ns
	andi	aux, ~((1 << CS02) | (1 << CS00))
	out		TCCR0B, aux
	ret

start_timer1:
	lds		aux, TCCR1B
	ori		aux, (1 << CS10)	; sin prescaler --> un CM del timer 1 son 1/16MHz = 62.5ns
	andi	aux, ~((1 << CS12) | (1 << CS11))
	sts		TCCR1B, aux
	ret

stop_timer1:
	; data_low y data_mid contienen el valor en el cual se frenó el timer 1
	; tras ser reiniciado (debido a un overflow) por última vez
	lds		data_low, TCNT1L
	lds		data_mid, TCNT1H

	lds		aux, TCCR1B
	andi	aux, ~((1 << CS12) | (1 << CS11) | (1 << CS10))
	sts		TCCR1B, aux
	ret

stop_timer0:
	in		aux, TCCR0B
	andi	aux, ~((1 << CS02) | (1 << CS01) | (1 << CS00))
	out		TCCR0B, aux
	ret
	

/***************************************************************************************/
; CONFIG_ADC
; Configuración del conversor digital-analógico (ADC)
; Utilizamos la referencia de tensión por default de 5V (REFS[1:0]=01).
; Tomamos la entrada analógica por ADC0 (en ADMUX --> MUX[3:0]=0000).
; Seteamos la frecuencia de muestreo en 125kHz (128/16MHz) considerando el límite de los 200kHz,
; para lo cual utilizamos el prescaler de 128.
; Habilitamos el ADC (ADEN=1). Todavía no iniciamos la conversión.
; Habilitamos interrupciones (ADIE=1).
; ADC Right adjust result (ADLAR=0 en ADMUX) --> 0000 00HH LLLL LLLL (para más precisión),
; pero usaremos LEFT ADJUST (ADLAR=1).
/***************************************************************************************/
config_ADC:
	ldi		xh, high(voltage_RAM)
	ldi		xl, low(voltage_RAM)

	;Habilitamos el ADC - por interrupción - prescaler en 128 - primer conversión - sin autotrigger
	lds		aux, ADCSRA
	ori		aux, (1 << ADEN) | (1 << ADIE) | (0 << ADATE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
	andi	aux, ~(1 << ADATE)
	sts		ADCSRA, aux

	;Canal de entrada ADC0 (PC0) - Referencia Avcc - left adjusted
	lds		aux, ADMUX
	ori		aux, (1 << REFS0)
	andi	aux, ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << REFS1) | (1 << ADLAR))
	sts		ADMUX, aux

	; Para habilitar el ADC hay que poner en 0 el bit de Power Reduction ADC del registro PRR
	lds		aux, PRR
	andi	aux, ~( 1<<PRADC )
	sts		PRR, aux

	; Iniciamos la conversión
	lds   aux, ADCSRA
	ori   aux, (1 << ADSC)
	sts   ADCSRA, aux

	ret

;***********************************************************************************
; init_RAM
; Carga de valores en RAM
;***********************************************************************************
init_RAM:
	; Apuntamos a la RAM
	ldi		xh, high(voltage_RAM)
	ldi		xl, low(voltage_RAM)

	ldi		aux, 0
	st		x+, aux
	st		x+, aux

	ret

;***********************************************************************************
; add_sep_char ('-')
;***********************************************************************************
add_sep_char:
	ldi		aux, SEP_C_VALUE
	st		x+, aux
	ret

;***********************************************************************************
; add_final_char ('&')
;***********************************************************************************
add_final_char:
	ldi		aux, END_CHAR	
	st		x+, aux
	ldi		aux, END_FLAG
	st		x, aux
	ret

;***********************************************************************************
; calculate_tau
;***********************************************************************************
calculate_tau:
	; Tau = (data_high data_mid data_low)
	clr		data_high

	; data_low y data_mid contienen el valor en el cual se frenó el timer 1
	; tras ser reiniciado (debido a un overflow) por última vez
	lds		data_low, TCNT1L
	lds		data_mid, TCNT1H
	
	; RECORDAR: cada CM del timer 1 corresponde a 62.5ns
	; entonces si por ejemplo TCNT1L=0x0A significa que habrán pasado 10*62.5ns=625ns
	; --> entonces tau va a quedar en múltiplos de ns

	; timer1_counter: cantidad de veces que se produjo overflow en el timer 1
	; El overflow sucede cuando el timer pasa de FFFF a 0000, entonces
	; como el timer 1 cuenta de 0 a 0xFFFF hay que hacer:
	; Tiempo total (63%) = (0x10000) * timer1_counter + (data_mid,data_low)
	; La cuenta se pasa de los dos bytes, por eso necesitamos un registro
	; de 8 bits extra: data_high

	mov		data_high,timer1_counter	; equivale a hacer primero un clr de data_high
							; y luego add data_high,timer1_counter

	ret

;***********************************************************************************
; calculate_c_value
;***********************************************************************************
calculate_c_value:	
	; Calculamos el valor de C por restas sucesivas.
	; Por ejemplo: para calcular 14/5 hay que hacer:
	; 14-5 = 9 ---- VEZ 1
	; 9-5  = 4 ---- VEZ 2
	; 4-5 da menor a 0 entonces el resultado de 14/5 es igual a 2 con resto igual a 4
	; Cargamos el valor de la resistencia (ver informe)
	ldi		aux, 0
	mov		r_high, aux
	ldi		aux, high(R_VALUE)
	mov		r_mid, aux
	ldi		aux, low(R_VALUE)
	mov		r_low, aux

	clr		div_counter_high
	clr		div_counter_mid
	clr		div_counter_low

	; En este caso queremos resolver TAU/R para hallar C
loop:
	; Tau = (data_high data_mid data_low)
	mov		aux, r_high
	inc		aux
	cp		data_high, aux
	brsh	again					; mayor estricto a r_high

	cp		data_high, r_high
	brlo	exit_compare			; menor estricto

	; Caso HIGH igual
	mov		aux, r_mid				
	inc		aux						
	cp		data_mid, aux			; si es mayor estricto -> aun puede restar
	brsh	again

	cp		data_mid, r_mid			; si es menor estricto -> ya no puede restar
	brlo	exit_compare	

	; Caso HIGH - MID igual
	cp		data_low, r_low			; si llego acá es porque es igual -> decide la parte baja
	brlo	exit_compare

again:
	sub		data_low, r_low
	sbc		data_mid, r_mid
	sbc		data_high, r_high
	
	add		div_counter_low, one
	adc		div_counter_mid, zero
	adc		div_counter_high, zero

	rjmp	loop

exit_compare:
	ret

;***********************************************************************************
; add_c_value
; Agrega el valor de C a RAM
;***********************************************************************************

add_c_value:
	st		x+, div_counter_high
	st		x+, div_counter_mid
	st		x+, div_counter_low
	ret

;***********************************************************************************
; begin_TX 
; Comienza la TX de lo que haya en voltage_RAM
;***********************************************************************************
begin_TX:
	; Habilitamos la interrupción para la transmisión poniendo un 1 en el bit 5 (UDRIE0) de UCSR0B
	; (Datasheet: writing this bit to one enables interrupt on the UDRE0 flag)
	ldi		xl, low(voltage_RAM)
	ldi		xh, high(voltage_RAM)

	ld		aux, x+
	; El micro empieza a transmitir cuando escribimos el UDR0
	sts		UDR0,AUX

	lds		AUX,UCSR0B
	ori		AUX,(1 << UDRIE0)
	sts		UCSR0B,AUX

	ret

;***********************************************************************************
; memory_validation
; Valida que x se encuentre dentro de los 1500 bytes asignados en RAM
;***********************************************************************************

memory_validation:
	mov		aux, xh
	cpi		aux, high(VOLTAGE_RAM_END)+1
	brsh	memory_error

	cpi		aux, high(VOLTAGE_RAM_END)
	brlo	exit_memory_validation

	mov		aux, xl
	cpi		aux, low(VOLTAGE_RAM_END)+1
	brlo	exit_memory_validation

memory_error:
	rcall	out_of_range_msg
	ldi		bool_out_of_range, TRUE

exit_memory_validation:
	ret

;***********************************************************************************
; out_of_range_msg
; Carga mensaje de error para el OOR de tabla
;***********************************************************************************

out_of_range_msg:
	; Cargamos el codigo de error
	ldi     aux, STATUS_OOR_TABLE
	ldi		aux_counter, MAX_STATUS

	; Apuntamos a la RAM
	ldi		xh, high(voltage_RAM)
	ldi		xl, low(voltage_RAM)

msg_loop:	
	st		x+, aux
	dec		aux_counter
	brne	msg_loop

	ret

;***********************************************************************************
; oor_terminate_msg
; Carga mensaje de error para el OOR del tau
;***********************************************************************************
oor_terminate_msg:
	; Cargamos el código de error
	ldi     aux, STATUS_OOR_TAU
	ldi		aux_counter, MAX_STATUS

	; Apuntamos a la RAM
	ldi		xh, high(voltage_RAM)
	ldi		xl, low(voltage_RAM)

msg_loop2:	
	st		x+, aux
	dec		aux_counter
	brne	msg_loop2

	ldi		aux, END_CHAR
	st		x+, aux

	ldi		aux, END_FLAG
	st		x+, aux

	ret
/***********************************************************************************/
;						INTERRUPCIONES (HANDLERS)
/***********************************************************************************/

Handler_Timer1_Overflow:
	push	aux
	in		aux, SREG
	push	aux

	add		timer1_counter, one
	brcc	exit_Handler_Timer1_Overflow

	ldi		bool_terminate, TRUE

exit_Handler_Timer1_Overflow:
	pop		aux
	out		SREG, aux
	pop		aux
	reti

;***********************************************************************************
	
Handler_Timer0_Overflow:
	push	aux
	in		aux, SREG
	push	aux

	inc		timer0_counter			; Por el momento solo es útil para el delay de 4 segundos
									; Cuenta la cantidad de overflows que ocurrieron en el timer 0
									; Cada vez que ocurre un OVF del timer 0 pasaron 127.5us

	; Iniciamos la conversión
	lds		aux, ADCSRA
	ori		aux, (1 << ADSC)
	sts		ADCSRA,aux

	pop		aux
	out		SREG, aux
	pop		aux

	reti

;***********************************************************************************

Handler_TX:
	push	aux
	in		aux, SREG
	push	aux

	ld		aux, x+
	cpi		aux, END_FLAG
	breq	end_TX

	; UDR0 es el registro de datos --> guardamos allí lo que queremos transmitir
	sts		UDR0, aux			; transmisión del caracter
	rjmp	break_tx

end_TX:
	; Detenemos la transmisión una vez que se llegó al final
	lds		aux, UCSR0B
	andi	aux, ~(1<<UDRIE0)
	sts		UCSR0B, aux

break_tx:
	pop		aux
	out		SREG, aux
	pop		aux
	reti

;***********************************************************************************

Handler_ADC_Conversion:
	; Leemos el valor convertido en digital
	push	aux
	in		aux, SREG
	push	aux

	lds		data_low, ADCL
	lds		data_mid, ADCH		

	cpi		bool_out_of_range, TRUE		; lo ponemos antes y después porque una vez que se pasó
										; no vuelve a entrar a memory_validation (es preferible
										; que pregunte siempre dos veces a que entre a esa
										; subrutina dos veces)
	breq	no_copiar

	rcall	memory_validation

	cpi		bool_out_of_range, TRUE
	breq	no_copiar

	st		x+, data_mid
	st		x+, data_low
	
	ldi		aux, SEP_CHAR				; Separador entre tensiones
	st		x+, aux	
	
no_copiar:
	cpi		data_mid, high(ADC_LIMIT)+1
	brsh	stop_ADC					; Si la parte alta es más grande ya está
	
	cpi		data_mid, high(ADC_LIMIT)
	brlo	break_int_ADC				; Si la parte alta es menor entonces todavía le falta
	
	cpi		data_low, low(ADC_LIMIT)
	brsh	stop_ADC			; Si la parte alta es igual pero la parte baja es mayor, terminó
	rjmp	break_int_ADC		; Si la parte alta es igual pero la parte baja es menor, todavía le falta

stop_ADC:
	; Apagamos el ADC
	lds		aux, ADCSRA
	andi	aux, ~((1<<ADEN) | (1<<ADIE))
	sts		ADCSRA, aux
	
	rcall	stop_timer1
	rcall	stop_timer0
	ldi		bool_conversion_end, TRUE

break_int_ADC:
	pop		aux
	out		SREG, aux
	pop		aux
	reti

; Una conversión del ADC tarda 108us (128/16MHz * 13,5 = 8us * 13,5 = 108us)--> entonces el tiempo del compare match
; debe ser mayor a 108us (1 overflow del timer 0 con el prescaler en 8 es 127.5 us lo cual
; da tiempo además para cargar a RAM los datos y hacer las comparaciones necesarias)
