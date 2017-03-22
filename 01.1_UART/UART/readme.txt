Polling UART Example - with printf and scanf functions for the nucleo board

printf --> fputc --> uartSendChar --> HAL_UART_Transmit (Blocking mode)

scanf --> fgetc --> uartReceiveChar --> HAL_UART_Receive (Blocking mode)

printf("\f\n\rInsert a number: ");
scanf("%d", &num1);
printf("\n\rInsert a number: ");
scanf("%d", &num2);
res = num1 + num2;
printf("\n\rResult: %d", res);
printf("\n\rInsert a text: ");
scanf("%s", text);
printf("\n\rYou type %s", text);

How to use this example.

Open the proyect and compile it.
Connect the nucleo board 



