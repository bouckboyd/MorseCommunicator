# MorseCommunicator - Nucleo-F446RE

The MorseCommunicator is a text-to-morse translator that runs on a Nucleo-F446RE development board. The program accepts a text message from a keyboard, and uses the board's built-in LED to flash the morse code representation of the message. 

## Usage

At the program start, the prompt "Enter a message:" is given. The user then types the message to be translated.
- The MorseCommunicator supports the following characters: A-Z, a-z, 0-9, and the space character.

To complete the message, the return key is pressed. Alternatively, if the user reaches the max message length of 100 characters, they will be so informed and prompted to enter any key to begin message translation. 

The message is translated character by character and its morse code representation is flashed out on the Nucleo-F446RE board's built-in LED (LD2). 
- Transmission rate is 500ms per "dit" (short signal). A "dah" (long signal) is three times the duration of a "dit".
- If the user enters an unsupported character, it is simply skipped in the LED transmission. 

When message translation is completed, the prompt "Enter a message:" is given again and a new message can be entered and translated. 
