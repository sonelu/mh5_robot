Setting Up Raspberry PI
=======================

This section of the documentation lists the activities needed to setup the Raspberry Pi for MH5.

Installing the Raspbian OS
--------------------------

We use the mainline Raspbian as this has the highest support for hardware devices that we need to connect to the device as well as software that we need to run the robot. While at this moment (May 2021) the mainline is v7l (32 bit) and it would be beneficial to use a 64-bit OS, because of the significant dependencies with the hardware and software elements, it is better to stick with the standard 32-bit Buster version available.

To download the Raspbian get to the `main download page <https://www.raspberrypi.org/software/operating-systems/>`_ and get the version **Lite** of the OS.

Once downloaded on your machine use `Balena Etcher <https://www.balena.io/etcher/>`_ to write the image to an SD card. Use a card of at least 16Gb (the OS will expand automatically when installing) and with the fastest speed you can get (ex. `UHS-1 or better <https://en.wikipedia.org/wiki/SD_card#Bus>`_). The program can read directly from an ZIP image, so there is no need to unzip the downloaded image first.

Once the Sd card is initialized, you need to activate ``ssh`` access. For this, *before* you insert the card in Raspberry Pi, insert it in a card reader in the computer. The ``boot`` partition of the card is formatted FAT-32 and will be visible on any computer. Create an empty file called ``ssh`` on that partition in the top directory. If you are using a MacOS or a Linux computer you can do this from the command prompt like this:

.. code-block:: bash

   cd /Volumes/boot
   touch ssh

Now you can unmount the disk from your system and place the card in the Raspberry Pi. Attach the MH5 HAT, connect an Ethernet cable (this is the easiest way to setup the Raspberry Pi and we will setup the WiFi later), insert the WiFi dongle and turn the power on for the Raspberry Pi. Wait for it to boot for the first time and you will see the activity LED on the side of the Raspberry Pi blinking intensely (will stay lit a continuous period of time) after which the system will reboot and the activity LED will go back to short bursts of light. We are now ready to setup the hardware.

Connecting over ``ssh``
-----------------------

Use an IP snooper to identify the IP address of the new Raspberry Pi. For MacOS you can use `Angry IP Scanner <https://angryip.org>`_. The device should be listed with a MAC vendor as "Raspberry Pi Trading" and the device hostname should be "raspberrypi.local". Things change over time and descriptions might be different, but in general you should be able to identify from the list of devices the new Raspberry Pi.

Connect to the Pi using a ssh client. For MacOS and Linux simply run from a new terminal:

.. code-block:: bash

   ssh 192.192.168.45 -l pi

You will be asked for the password which, for the newly installed Raspberry Pis is *raspberry* (you will be heckled by the system that it is not safe to keep the default password and you should change it). Depending on your ssh client you might be asked to save the sha256 key generated for the device into your list of known connection to avoid security risks deriving from a spoofed remote machine.

Now you should be connected to the system and should see the prompt.

Running ``raspi-config``
------------------------

Before running ``raspi-config`` we need to get some details about out WiFi devices so that we can properly use them.

There should be 2 wifi interfaces listed when running ``ifconfig``:

.. code-block:: bash

    pi@raspberrypi:~ $ ifconfig
    eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
            inet 192.168.0.45  netmask 255.255.255.0  broadcast 192.168.0.255
            inet6 fe80::8cf9:2353:d7fd:4918  prefixlen 64  scopeid 0x20<link>
            ether dc:a6:32:49:3b:74  txqueuelen 1000  (Ethernet)
            RX packets 15654  bytes 2577471 (2.4 MiB)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 1520  bytes 240475 (234.8 KiB)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

    lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
            inet 127.0.0.1  netmask 255.0.0.0
            inet6 ::1  prefixlen 128  scopeid 0x10<host>
            loop  txqueuelen 1000  (Local Loopback)
            RX packets 0  bytes 0 (0.0 B)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 0  bytes 0 (0.0 B)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

    wlan0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
            ether dc:a6:32:49:3b:76  txqueuelen 1000  (Ethernet)
            RX packets 0  bytes 0 (0.0 B)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 0  bytes 0 (0.0 B)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

    wlan1: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
            ether e8:4e:06:61:6f:af  txqueuelen 1000  (Ethernet)
            RX packets 0  bytes 0 (0.0 B)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 0  bytes 0 (0.0 B)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

The first is the Ethernet interface that is connected to the LAN. It is followed by the loopback interface. Our two interfaces are then listed as ``wlan0`` and ``wlan1``. In case names like wl1234567890 then predictable interface names are activated and we need to deactivate them. This is because we would loose the configuration of AP and bridge in case the dongle is replaced with another one or (depending on the settings) if the dongle is moved to another USB port. To disable the predictable names run ``raspi-config`` and select ``6 Advanced Options`` > ``A4 Network Interface Names``. Select ``<No>`` at the question " Would you like to enable predictable network interface names?". You will get a confirmation that "Predictable network interface names are disabled". You will need to reboot your PI and re-logon with ``ssh`` if this is the case. But in most of situations the predictable names are deactivated and you would not need to perform this activity.

What we want now is to make some configurations that will make this particular robot different from other robots that might be connected in the same network or present in the same room. For this we will use the last 4 hex codes of the MAC address of the ``wlan0`` (the inbuilt WiFi) to identify the robot and later for the setup of the Access Point. In the example above the ``wlan0`` has ``3b:76`` as the last codes of the MAC address so, we will call this robot **MH5-3B76**.

In the ssh console run:

.. code-block:: bash

   sudo raspi-config

Select ``1 System Options`` > ``S1 Wireless LAN``. You will now be asked for the country where the system is to be used. Each country has it's own frequencies allocated to WiFi and by default Raspbian deactivates WiFi until the correct country is setup so that no laws are broken. You will get a confirmation about the country being setup, and then you will be asked for the SSID of the network to connect to. **Do not use this**. We will setup the WiFi manually by building an Access Point (AP) using the 5Ghz WiFi interface that is built in the Pi and we will use to connect to an external WiFi (if there is one) using the dongle that is inserted in the USB port. So click ``<Cancel>`` in this screen. You will be sent back to the main menu.

Select ``1 System Options`` > ``S4 Hostname``. Press ``<OK>`` at the next warning and in the next dialog delete the ``raspberrypi`` name and replace it with ``MH5-XXXX`` where XXXX are the last two digits in the MAC address of ``wlan0`` as specified above. For our example here the name would be ``MH5-3B76``. Press ``<OK>`` and you will land again on the main menu.

Select ``1 System Options`` > ``S5 Boot / Auto Login``. Select ``B2. Console Autologin``. This means the system will automatically login as 'pi' and we would be able later to automatically start up the ROS packages that are controlling the robot.

Select ``3 Interface Options`` > ``P4 SPI``. Select ``<Yes>`` when asked "Would you like the SPI interface to be enabled?". The TFT screen and the SC16IS762 chip on the MH5 HAT that provides the buses for Dynamixel servos use the SPI interface and needs to be activated. There will be a message confirming the activation.

Select ``3 Interface Options`` > ``P5 I2C``. Select ``<Yes>`` when asked "Would you like the ARM I2C interface to be enabled?". Several devices on the MH5 HAT use the I2C interface and needs to be activated. There will be a message confirming the activation.

Select ``3 Interface Options`` > ``P6 Serial Port``. Select ``<Yes>`` when asked "Would you like a login shell to be accessible over serial?". The CP2102 device on the MH5 HAT allows for a convenient access to the serial console and provides emergency access in case the network is not accessible. There will be a message confirming the activation.

Select ``4 Performance Options`` > ``P4 Fan``. Select ``<Yes>`` when asked "Would you like to enable fan temperature control?". The MH5 HAT includes fan control on GPIO12 so in the next screen enter "12" when asked "To which GPIO is the fan connected?". At the next question "At what temperature in degrees should the fan turn on?" enter "60" (that is 60 degrees Celsius). We would have later the ability to update these settings in ``/boot/config.txt`` and we can change the temperature or GPIO if there are hardware changes. There will be a message confirming the activation.

We can now click on ``<Finish>``. The program will ask to reboot now, answer ``<Yes>`` and wait for the device to reboot and re-logon using ``ssh``.

When loging in the system you should see the prompt reflecting the new hostname of the device:

.. code-block:: bash 

    Linux MH5-3B76 5.10.17-v7l+ #1403 SMP Mon Feb 22 11:33:35 GMT 2021 armv7l

    The programs included with the Debian GNU/Linux system are free software;
    the exact distribution terms for each program are described in the
    individual files in /usr/share/doc/*/copyright.

    Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
    permitted by applicable law.
    Last login: Sat May  8 23:34:21 2021

    SSH is enabled and the default password for the 'pi' user has not been changed.
    This is a security risk - please login as the 'pi' user and type 'passwd' to set a new password.

    pi@MH5-3B76:~ $

Installing the drivers for hardware
-----------------------------------

The devices on the HAT most likely will not work as they have to be setup and activated.

Update / upgrade
~~~~~~~~~~~~~~~~

Before doing the other installations we need to make sure that all packages and sources are updated.

.. code-block:: bash 

    sudo apt update
    sudo apt-get update
    sudo apt-get -y upgrade

    sudo apt-get install -y python3-pip
    sudo pip3 install --upgrade setuptools

Install the TFT display driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We use Adafruit 2.0" display (no touch) and the installation of the hardware drivers is simplified in a script provided by Adafruit. Simply follow the following commands in the terminal:

.. code-block:: bash 

    cd ~
    sudo apt-get install -y git
    sudo pip3 install --upgrade adafruit-python-shell click==7.0
    git clone https://github.com/adafruit/Raspberry-Pi-Installer-Scripts.git
    cd Raspberry-Pi-Installer-Scripts

Before we run the installation we will need to change on pin setting in the device tree overlay: in the Adafruit implementation the back-light of the display is connected to GPIO12 (it's PWM and the only one remaining if GPIO18 is used by I2S - which will happen because we activate it later to provide support for WM8960 chip on the HAT). So we will need to change the pin to GPIO13 (actually we are not using it, but the driver will fail to initialize if the GPIO12 is used after the fan control that uses this pin for real is initialized).

So run 

.. code-block:: bash 

    nano overlays/st7789v_240x320-overlay.dts

In the file change the sequence:

.. code-block::

    pitft: pitft@0{
                    compatible = "sitronix,st7789v";
                                    reg = <0>;
                                    pinctrl-names = "default";
                                    pinctrl-0 = <&pitft_pins>;
                                    spi-max-frequency = <32000000>;
                                    rotate = <0>;
                                    width = <240>;
                                    height = <320>;
                                    buswidth = <8>;
                                    dc-gpios = <&gpio 25 0>;
                                    led-gpios = <&gpio 12 0>;
                                    debug = <0>;

To read:

.. code-block::

    led-gpios = <&gpio 13 0>;

The you can run:

.. code-block:: bash

    sudo python3 adafruit-pitft.py

You should see a list as follows:

.. code-block::

    This script downloads and installs
    PiTFT Support using userspace touch
    controls and a DTO for display drawing.
    one of several configuration files.
    Run time of up to 5 minutes. Reboot required!

    Select configuration:
    [1] PiTFT 2.4", 2.8" or 3.2" resistive (240x320) (320x240)
    [2] PiTFT 2.2" no touch (320x240)
    [3] PiTFT 2.8" capacitive touch (320x240)
    [4] PiTFT 3.5" resistive touch (480x320)
    [5] PiTFT Mini 1.3" or 1.54" display (240x240) - WARNING! WILL UPGRADE YOUR KERNEL TO LATEST
    [6] ST7789V 2.0" no touch (320x240) - WARNING! WILL UPGRADE YOUR KERNEL TO LATEST
    [7] MiniPiTFT 1.14" display (240x135) - WARNING! WILL UPGRADE YOUR KERNEL TO LATEST
    [8] TFT 1.3" Bonnet + Joystick (240x240) - WARNING! WILL UPGRADE YOUR KERNEL TO LATEST
    [9] Uninstall PiTFT
    [10] Quit without installing

Select [6].

.. code-block::

    Select rotation:
    [1] 90 degrees (landscape)
    [2] 180 degrees (portrait)
    [3] 270 degrees (landscape)
    [4] 0 degrees (portrait)

Select [2].

Wait for the installation to complete. This will update the kernel too. When asked:

.. code-block::

    Would you like the console to appear on the PiTFT display? [y/n]

Answer [y]. And at the end when asked to REBOOT NOW? answer Y and press Enter. After a short pause, you should be able to see the console messages being listed on the display as the system boots and then you should see a prompt with the user pi.
