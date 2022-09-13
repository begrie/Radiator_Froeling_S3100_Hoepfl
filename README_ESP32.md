# Überwachungstool für Fröling P2 / Lambdatronic S 3100

## Anpassung für ESP32 (BeGrie, Sept. 2022)

Im Zuge der Energiekrise 2022 und einem Pelletspreisanstieg von 200 auf 800 Euro je Tonne habe ich mein seit langem geplantes und immer wieder beiseite geschobenes Projekt (Heizung seit 2006 im Betrieb - hüstel) der Heizungsüberwachung wieder aktiviert um Optimierungspotentiale zu erschließen. 

Bei der Recherche nach Lösungen bin ich unter anderem auf dieses Projekt von Daniel Höpfl gestoßen und war von Analyse des P2/S3100-Kommunikationsprotkolls und der professionellen Umsetzung in Sourcecode begeistert. Hiermit huldige ich dem grossen Meister, von dem ich aus seinem Projekt einiges lernen durfte! Vielen Dank dafür! 

Als ESP32 Nutzer habe ich mangels RasPi Erfahrungen und fehlender Erläuterungen im Projekt (sorry) doch etwas gebraucht, um den gesamten Ablauf auf dem RasPi mittels C++, Autoconf/Automake, Python und Shellskripten in Kombination mit mysql, rrd, Vu, mailserver etc. zu verstehen. Nun ist alles klar, aber ich brauche so viele Features davon nicht und bleibe doch lieber in den mir bekannten ESP32-Gefilden - die auch einen geringeren Energiebedarf mit sich bringen (<1 Watt) ...

Die ESP32 Lösung besteht nur noch aus dem C++ Sourcecode, bei dem ich im wesentlichen folgendes geändert bzw. ergänzt habe:

* main.cpp neu für ESP32 mit Platformio (bei mir in VSCode) -> Platform Espressif32 4.4.0 und Arduino framework 2.0.3 (neuere Versionen sollten "eigentlich" kein Problem machen ...)

* Serieller Input (in serial.cpp) über Serial2 realisiert (hardwareseitig wie beim RasPi über RS232-Schnittstellenumsetzer ->bei mir SparkFun Transceiver Breakout - MAX3232 https://www.sparkfun.com/products/11189)

* Output fast komplett neu (output.h, output.cpp) -> nun "autark" (ohne externe Skripte o.ä) auf  
    * Console/Serial, 
    * Flash/LittleFS (wg. geringem Speicherplatz nur zum Testen sinnvoll). 
    * SD Card (hardwareseitig jeder 3,3V SD Reader an den ESP32 anschliessbar). Aktuell muss die SD Card entnommen und am PC eingelesen werden. 
    * Buzzer (aktiv) und Quit-Button zur akustischen Fehlermeldung im Keller 

* Geplant sind noch WiFi zum Versand von MQTT-Nachrichten und evtl. ein kleiner Webserver zum Datenabgriff von der SD-Card - schau'mer mal

* Die Konfiguration bzgl. Filesystem, Output und weiterem Verhalten erfolgt ausschließlich über die #defines in main.cpp, output.h und serial.cpp, was bei Änderungen natürlich eine Neucompilierung erfordert (ich weiss: nicht schön - aber der kürzeste Weg). Dies sollte aber durch die Nutzung von Platformio über die hier enthaltene platformio.ini problemlos möglich sein ...

* Schaltplan zur Konfiguration im Sourcecode:
    ![Schaltplan](Schaltplan.jpg)

* Fotos zur Umsetzung
    ![Foto Platine oben](Platine_Oben.jpg)
    ![Foto Platine unten](Platine_Unten.jpg)
    ![Foto Heizung](Heizung1.jpg)


Falls jemand die ESP32 Anpassung nutzt, bin ich für Rückmeldungen, Verbesserungen, Pull requests etc. offen ...
