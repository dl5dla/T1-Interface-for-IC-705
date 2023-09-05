# T1 interface for ICOM IC-705

<b>Update on 5.09.2023:</b> The ESP32 behaves now as a bluetooth master, which makes the bluetooth binding between the IC-705 and the ESP32 more robust.<br>

<b>Hardware:</b><br>
As I am operating my IC-705 remotely by means of WFView (www.wfview.org), I was searching for an easy way to switch my Elecraft T1 tuner to the right settings when changing the frequency. The T1 stores its settings for each band which can be recalled via the control lines available on the jack socket of the T1.<br>

The control lines are connected to an ESP32 D1 mini which fetches the current frequency via bluetooth from the IC-705. When the frequency of the TRX changes, the ESP32 will send the new band number to the T1 forcing it to change its settings. For a few seconds the T1 will wait for a rf carrier. So, if one would like to retune, it would be enough to send a carrier directly after the frequency was changed.
<br>
![Screenshot](pic4.png)<br>
The transistor (npn, e.g. BC547 or similar) and the resistor is soldered directly on the pin bar, the NC pin is just used to connect the collector to the wire.<br>
![Screenshot](pic1.png)
![Screenshot](pic2.png)
![Screenshot](pic3.png)
<br><br>
<b>Software:</b><br>
The code is based on the code from<br>
 Ondrej Kolonicny (OK1CDJ) https://github.com/ok1cdj/IC705-BT-CIV 
 for controlling the ic-705 via bluetooth<br>
 and<br>
 Matthew Robinson (VK6MR) https://zensunni.org/blog/2011/01/19/arduino-elecraft-t1-interface/ 
 for controlling the T1 by means of an ardunio.<br><br>
I did the compilation of the code with PlatformIO, but it should be possible also with the Arduino IDE
 <br><b>
You have to replace the dummy bluetooth address in main.cpp with that of your IC-705.</b>
<br><br>
Depending on your country it might be required to adapt the frequency bands in main.c (starting from line 110).<br><br>

<b>Pairing</b>:<br>
After flashing the code to the ESP32, the IC-705 and the ESP32 must be paired. In the bluetooth settings of the IC-705 select the menu item  "Pairing reception" and then immediately press the reset button at the ESP32 board. On the display of the IC-705 you should see now a message, which expects confirmation of a pass key. Press OK and the bluetooth connection shoud be established.
<br><br>
<s>After initial pairing of the ESP32 and ic-705, it is required to power-on the ESP32, before the ic-705 is switched on.</s>
The new version sets the ESP32 as bluetooth master. Now in case the ESP32 or the TRX is temporarly switched off and on again, the TRX will be reconnected automatically.<br><br>

Optional:<br>
The code for controlling the T1 includes some delay() statements, which block the bluetooth communication for a while. This leads to error messages "esp_spp_cb(): RX Full! Discarding 22 bytes" within the PlatformIO console. To avoid those errors, one can change the rx buffer size in the BluetoothSerial library:

In BluetoothSerial.cpp change

#define RX_QUEUE_SIZE 512<br>
to<br>
#define RX_QUEUE_SIZE 2046
