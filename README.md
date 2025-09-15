# stm32f1-app-jumper
This repo aims to create an app jumper using UART. The app1 asks the user the period of the LED blink and writes it to an adress at the end of flash memory. The "JUMP" command will trigger the jump_to_app() function and jumps to app2. The app2 reads the LED blink period from flash and blinks LED accordingly. The addresses have been changed in the flash.ld files; for app1 the flash memory address has been kept the same but the length is shortened to 16KB, for app2 the flash memory start address has been set to 0x08004400 and the lenght 48KB.
## app1 configuration
For the example app1 configuration USART1 protocol has been used. Baud rate has been set to 115200 bits/s, word length is 8 bits, no parity bit and 1 stop bit.
<img width="1694" height="763" alt="Screenshot (522)" src="https://github.com/user-attachments/assets/df3fc8b6-891f-48e5-bf9b-66fd7d6d978d" />
## app2 configuration
app2 configuration uses almost the same settings as app1 but an external LED has been connected to PA5 pin and the pin has been configured as GPIO output.
<img width="1679" height="755" alt="Screenshot (523)" src="https://github.com/user-attachments/assets/8017698c-1076-442f-ad73-ce8d12fe7826" />
