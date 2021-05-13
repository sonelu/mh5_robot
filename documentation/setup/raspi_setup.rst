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

   $ cd /Volumes/boot
   $ touch ssh

Now you can un-mount the disk from your system and place the card in the Raspberry Pi. Attach the MH5 HAT, connect an Ethernet cable (this is the easiest way to setup the Raspberry Pi and we will setup the WiFi later), insert the WiFi dongle and turn the power on for the Raspberry Pi. Wait for it to boot for the first time and you will see the activity LED on the side of the Raspberry Pi blinking intensely (will stay lit a continuous period of time) after which the system will reboot and the activity LED will go back to short bursts of light. We are now ready to setup the hardware.

Connecting over ``ssh``
-----------------------

Use an IP snooper to identify the IP address of the new Raspberry Pi. For MacOS you can use `Angry IP Scanner <https://angryip.org>`_. The device should be listed with a MAC vendor as "Raspberry Pi Trading" and the device hostname should be "raspberrypi.local". Things change over time and descriptions might be different, but in general you should be able to identify from the list of devices the new Raspberry Pi.

Connect to the Pi using a ssh client. For MacOS and Linux simply run from a new terminal:

.. code-block:: bash

   $ ssh 192.192.168.45 -l pi

You will be asked for the password which, for the newly installed Raspberry Pis is *raspberry* (you will be heckled by the system that it is not safe to keep the default password and you should change it). Depending on your ssh client you might be asked to save the sha256 key generated for the device into your list of known connection to avoid security risks deriving from a spoofed remote machine.

Now you should be connected to the system and should see the prompt.

.. _running_raspi_config:

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

In the ``ssh`` console run:

.. code-block:: bash

   $ sudo raspi-config

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

Setting the ``performance`` governor
------------------------------------

The governor is the way the processor cores are behaving when different type of load is presented. By default the Raspberry Pi uses "on demand" which means that the frequency of the cores is automatically reduced when the load is small and it will pick up when the load is increased:

.. code-block:: bash 

    $ cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
    ondemand

We can change the governor by replacing with one of the available options that we can obtain by running: 

.. code-block:: bash 

    $ cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors
    conservative ondemand userspace powersave performance schedutil

What we are after is the ``performance`` one which we can set by executing:

.. code-block:: bash 

    sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"

We need the ``sudo sh ...`` because it requires ``sudo`` elevation and ``echo`` does not support sudo direct (you cannot write ``sudo echo...``. This statement should also be included in the 

Installing the drivers for hardware
-----------------------------------

The devices on the HAT most likely will not work as they have to be setup and activated.

Update / upgrade
~~~~~~~~~~~~~~~~

Before doing the other installations we need to make sure that all packages and sources are updated.

.. code-block:: bash 

    $ sudo apt update
    $ sudo apt-get update
    $ sudo apt-get -y upgrade

    $ sudo apt-get install -y python3-pip
    $ sudo pip3 install --upgrade setuptools

Install the TFT display driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We use Adafruit 2.0" display (no touch) and the installation of the hardware drivers is simplified in a script provided by Adafruit. Simply follow the following commands in the terminal:

.. code-block:: bash 

    $ cd ~
    $ sudo apt-get install -y git
    $ sudo pip3 install --upgrade adafruit-python-shell click==7.0
    $ git clone https://github.com/adafruit/Raspberry-Pi-Installer-Scripts.git
    $ cd Raspberry-Pi-Installer-Scripts

Before we run the installation we will need to change on pin setting in the device tree overlay: in the Adafruit implementation the back-light of the display is connected to GPIO12 (it's PWM and the only one remaining if GPIO18 is used by I2S - which will happen because we activate it later to provide support for WM8960 chip on the HAT). So we will need to change the pin to GPIO13 (actually we are not using it, but the driver will fail to initialize if the GPIO12 is used after the fan control that uses this pin for real is initialized).

So run 

.. code-block:: bash 

    $ nano overlays/st7789v_240x320-overlay.dts

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

And change these parameters:

.. code-block::

    led-gpios = <&gpio 13 0>;
    spi-max-frequency = <40000000>;

The you can run:

.. code-block:: bash

    $ sudo python3 adafruit-pitft.py

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

Configure the SC16IS762 drivers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The drivers in the Raspberry Pi kernel are ok and they are now working fine. What we need is the overlay that will activate the drivers and contains the settings specific to out board. The stock overlays provided in the Raspberry Pi kernel for SPI interface do not support using the CE1 select, they all assume the connection is using CE0. Also the overlays are for SC16IS752 which supports lower SPI speeds and we take advantage of the increased speed of SC16IS762 to support higher baud-rates for our UART ports.

For this reason we have a changed overlay definition that is provided in the SC16IS762 directory. Bellow it is listed for further reference:

.. code-block::

    /dts-v1/;
    /plugin/;

    / {
        compatible = "brcm,bcm2835";

        fragment@0 {
            target = <&spi0>;
            __overlay__ {
                status = "okay";
                spidev@1{
                    status = "disabled";
                };
            };
        };

        fragment@1 {
            target = <&spi0>;
            __overlay__ {
                #address-cells = <1>;
                #size-cells = <0>;
                status = "okay";

                sc16is762: sc16is762@0 {
                    compatible = "nxp,sc16is762";
                    reg = <1>; /* CE1 */
                    clocks = <&sc16is762_clk>;
                    interrupt-parent = <&gpio>;
                    interrupts = <23 2>; /* IRQ_TYPE_EDGE_FALLING */
                    gpio-controller;
                    #gpio-cells = <2>;
                    spi-max-frequency = <15000000>;

                };
            };
        };
        fragment@2 {
            target-path = "/";
            __overlay__ {
                sc16is762_clk: sc16is762_clk {
                    compatible = "fixed-clock";
                    #clock-cells = <0>;
                    clock-frequency = <32000000>;
                };
            };
        };

        __overrides__ {
            int_pin = <&sc16is762>,"interrupts:0";
            xtal = <&sc16is762_clk>,"clock-frequency:0";
            ce = <&sc16is762>,"reg:0";
        };
    };

You have to compile the ``dts`` file using the ``dtc`` (Device Tree Complier) tool that was already installed by the TFT installer. For this run in the directory where the overlay is located:

.. code-block:: bash

    $ dtc --warning no-unit_address_vs_reg -I dts -O dtb -o sc16is762-spi0-ce1.dtbo sc16is762-spi0-overlay.dts

A file ``sc16is762-spi0-ce1.dtbo`` should have been created in the same directory. Now place the file in the ``/boot/overlays/`` to be used by the kernel:

.. code-block:: bash

    $ sudo cp sc16is762-spi0-ce1.dtbo /boot/overlays/

Now the only thing left is to activate the device in the ``/boot/config.txt`` so that the kernel driver is loaded at boot time. Run:

.. code-block:: bash

    $ sudo nano /boot/config.txt

And add the following line after the line with the ``dtoverlay=gpio-fan,gpiopin=12,temp=60000`` (this was added by the ``rasppi-config``):

.. code-block::

    $ dtoverlay=sc16is762-spi0-ce1

Save the file and reboot your system. When you log back in you should be able to see two additional ``tty`` ports in the ``/dev`` directory:

.. code-block:: bash

    pi@MH5-3B76:~ $ ls /dev | grep SC
    ttySC0
    ttySC1

Installing the audio drivers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The MH5 HAT includes a high-performance audio chip WM8960 that provides support for stereo speakers (2 x 1W) and 2 microphones that are already included on the board.

To make sure that the device is recognized by the system run the following command to display the I2C devices:

.. code-block:: bash

    pi@MH5-3B76:~ $ i2cdetect -y 1
        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- 1a -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- --
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- --
    70: -- -- -- -- -- -- -- --

The device ``1a`` is the audio chip control interface. ``48`` is the ADC chip (TLA2024) that is used to monitor the voltages on the various buses on the board and ``6a`` is the accelerometer / gyroscope device.

To be able to use the device first clone the repository:

.. code-block:: bash

    $ git clone https://github.com/HinTak/seeed-voicecard.git
    $ cd seed-voicecard
    $ git checkout v5.9
    $ sudo ./install.sh

After the installation is complete you should reboot the Raspberry Pi. When logging in back with ``ssh`` you should now see if running the I2C tool:

.. code-block:: bash

    pi@MH5-3B76:~ $ i2cdetect -y 1
        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- UU -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- --
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- --
    70: -- -- -- -- -- -- -- --

The fact that the device at ``1a`` is now marked as ``UU`` is indicating that the device is now managed by a dedicated driver instead of being a generic I2C device.

Let's confirm that the "card" is seen:

.. code-block:: bash

    pi@MH5-3B76:~ $ aplay -l
    **** List of PLAYBACK Hardware Devices ****
    card 0: Headphones [bcm2835 Headphones], device 0: bcm2835 Headphones [bcm2835 Headphones]
        Subdevices: 8/8
        Subdevice #0: subdevice #0
        Subdevice #1: subdevice #1
        Subdevice #2: subdevice #2
        Subdevice #3: subdevice #3
        Subdevice #4: subdevice #4
        Subdevice #5: subdevice #5
        Subdevice #6: subdevice #6
        Subdevice #7: subdevice #7
    card 1: seeed2micvoicec [seeed-2mic-voicecard], device 0: bcm2835-i2s-wm8960-hifi wm8960-hifi-0 [bcm2835-i2s-wm8960-hifi wm8960-hifi-0]
        Subdevices: 1/1
        Subdevice #0: subdevice #0

The card is shown as ``seeed2micvoicec`` which is correct. You can configure the output of the card by running ``alsamixer``. Select the card with **F6** key:

.. code-block::

    ┌────────────────────────────────────────────────────── AlsaMixer v1.1.8 ───────────────────────────────────────────────────────┐
    │ Card: seeed-2mic-voicecard                                                                            F1:  Help               │
    │ Chip:                                                                                                 F2:  System information │
    │ View: F3:[Playback] F4: Capture  F5: All                                                              F6:  Select sound card  │
    │ Item: Headphone [dB gain: 6.00, 6.00]                                                                 Esc: Exit               │
    │                                                                                                                               │
    │                                                                                                                               │
    │                                                                                                                               │
    │                                                                                                                               │
    │                                                                                                                               │
    │                                                                                                                               │
    │                                                                                                                               │
    │   ┌──┐              ┌──┐     ┌──┐     ┌──┐                                         ┌──┐     ┌──┐                              │
    │   │▒▒│              │▒▒│     │▒▒│     │  │                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │  │                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │  │                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │  │                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │  │                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              →
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              │
    │   │▒▒│              │▒▒│     │▒▒│     │▒▒│                                         │▒▒│     │  │                              │
    │   └──┘     ┌──┐     └──┘     └──┘     └──┘     ┌──┐     ┌──┐     ┌──┐     ┌──┐     └──┘     ├──┤     Low      High   Left Dat │
    │            │MM│                                │MM│     │MM│     │MM│     │MM│              │MM│                              │
    │            └──┘                                └──┘     └──┘     └──┘     └──┘              └──┘                              │
    │ 100<>100          100<>100   100       80                                        100<>100    0                                │
    │<Headphon>Headphon Speaker  Speaker  Speaker  Speaker  PCM Play Mono Out Mono Out Playback    3D    3D Filte 3D Filte ADC Data │

Changing default ``python`` to ``python3``
------------------------------------------

The fresh Reasppberry Pi installation will use ``python2`` as the default python interpreter. We will change that to ``python3``.

.. code-block:: bash

    $ sudo rm /usr/bin/python
    $ sudo ln -s /usr/bin/python3 /usr/bin/python
    $ sudo rm /usr/bin/python-config
    $ sudo ln -s /usr/bin/python3-config /usr/bin/python-config

Verify that the links are to the version 3 of the interpreter:

.. code-block:: bash

    $ ls -al /usr/bin/python*
    lrwxrwxrwx 1 root root      16 May 10 13:15 /usr/bin/python -> /usr/bin/python3
    lrwxrwxrwx 1 root root       9 Mar  4  2019 /usr/bin/python2 -> python2.7
    -rwxr-xr-x 1 root root 2984816 Oct 10  2019 /usr/bin/python2.7
    lrwxrwxrwx 1 root root      36 Oct 10  2019 /usr/bin/python2.7-config -> arm-linux-gnueabihf-python2.7-config
    lrwxrwxrwx 1 root root      16 Mar  4  2019 /usr/bin/python2-config -> python2.7-config
    lrwxrwxrwx 1 root root       9 Mar 26  2019 /usr/bin/python3 -> python3.7
    -rwxr-xr-x 2 root root 4275580 Jan 22 20:04 /usr/bin/python3.7
    lrwxrwxrwx 1 root root      36 Jan 22 20:04 /usr/bin/python3.7-config -> arm-linux-gnueabihf-python3.7-config
    -rwxr-xr-x 1 root root     407 Jan 25  2019 /usr/bin/python3.7-coverage
    -rwxr-xr-x 2 root root 4275580 Jan 22 20:04 /usr/bin/python3.7m
    lrwxrwxrwx 1 root root      37 Jan 22 20:04 /usr/bin/python3.7m-config -> arm-linux-gnueabihf-python3.7m-config
    lrwxrwxrwx 1 root root      16 Mar 26  2019 /usr/bin/python3-config -> python3.7-config
    -rwxr-xr-x 1 root root     403 Jan 25  2019 /usr/bin/python3-coverage
    lrwxrwxrwx 1 root root      10 Mar 26  2019 /usr/bin/python3m -> python3.7m
    lrwxrwxrwx 1 root root      17 Mar 26  2019 /usr/bin/python3m-config -> python3.7m-config
    -rwxr-xr-x 1 root root     152 Dec 30  2018 /usr/bin/python3-pbr
    lrwxrwxrwx 1 root root      23 May 10 13:16 /usr/bin/python-config -> /usr/bin/python3-config

Setting-up the WiFi and router
------------------------------

Previously we have activated the WiFi interfaces. Now we will configure the system so that: the internal WiFI card will be used to setup an Access Point (AP) that can be used to connect directly to the robot, while the WiFi dongle would be used to connect to any exiting infrastructure network avaialble. We will setup packet routing between the built-in WiFi and the dongle WiFi as well as the Ethernet port. This way, if the robot is connected to an WiFi or a cabled network, if you connected with a desktop to the AP you still have internet access via this routing.

Activating the AP
~~~~~~~~~~~~~~~~~

We will setup the AP to use the ``wlan0`` and bridge the second ``wlan1`` wifi and ``eth0``.

We aim to use 5GHz frequency to provide very low latency. It would be recommended that when running ROS applications the remote computers are connected directly to the Access Point instead of using another network to route the traffic. We will install the needed software as follows (details from the [original Raspberry Pi documentation](https://www.raspberrypi.org/documentation/configuration/wireless/access-point-routed.md))

.. code-block:: bash

    $ sudo apt install hostapd
    $ sudo systemctl unmask hostapd
    $ sudo systemctl enable hostapd
    $ sudo apt install dnsmasq
    $ sudo DEBIAN_FRONTEND=noninteractive apt install -y netfilter-persistent iptables-persistent

Configure the static address for the ``wlan0``:

.. code-block:: bash

    $ sudo nano /etc/dhcpcd.conf

Add at the end of the file:

.. code-block::

    interface wlan0
        static ip_address=192.168.XX.1/24
        nohook wpa_supplicant

Where XX is the last hex of the MAC address as explained in the :ref:`running_raspi_config`.

Configure routing
~~~~~~~~~~~~~~~~~

We will first create create a routing configuration file:

.. code-block:: bash

    $ sudo nano /etc/sysctl.d/routed-ap.conf

And add these lines into it:

.. code-block::

    # https://www.raspberrypi.org/documentation/configuration/wireless/access-point-routed.md
    # Enable IPv4 routing
    net.ipv4.ip_forward=1

We will add the following routing rules and save them to be loaded by the ``netfilter-persistent`` service:

.. code-block:: bash

    $ sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
    $ sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE
    $ sudo netfilter-persistent save

Filtering rules are saved to the directory ``/etc/iptables/``.

Configure the DHCP and DNS services:

.. code-block:: bash

    $ sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
    $ sudo nano /etc/dnsmasq.conf

And enter in the file:

.. code-block::

    # Listening interface
    interface=wlan0 
    # Pool of IP addresses served via DHCP
    dhcp-range=192.168.XX.2,192.168.XX.100,255.255.255.0,24h
    # Local wireless DNS domain
    domain=wlan
    # Alias for this router
    address=/gw.wlan/192.168.XX.1

Where you have to replace ``XX`` with the same number as above. The Raspberry Pi will deliver IP addresses between ``192.168.XX.2`` and ``192.168.XX.100``, with a lease time of 24 hours, to wireless DHCP clients. You should be able to reach the Raspberry Pi under the name ``gw.wlan`` from wireless clients.

To ensure that WiFi is not blocked run the following command:

.. code-block::

    $ sudo rfkill unblock wlan

Configure the AP
~~~~~~~~~~~~~~~~

Run the following to setup a configuration file for AP daemon:

.. code-block:: bash

    $ sudo nano /etc/hostapd/hostapd.conf

And enter the following:

.. code-block::

    country_code=GB
    interface=wlan0
    ssid=MH5-3B76
    hw_mode=a
    channel=40
    macaddr_acl=0
    auth_algs=1
    ignore_broadcast_ssid=0
    wpa=2
    wpa_passphrase=Pass4MH5
    wpa_key_mgmt=WPA-PSK
    wpa_pairwise=TKIP
    rsn_pairwise=CCMP

The ``ssid`` should be "MH5" plus the last 4 codes of the MAC address of the ``wlan0`` device in order to avoid conflicts if multiple robots are in the same room.

``wpa_passphrase`` should be always "Pass4MH5" before sending the device to the user. They can change it with a custom password to enhance security. If the robot is to be used in a different location, the country must be set for that location and the channel should be revisited to make sure it is allowed in that country. Make sure though that it is a 5GHz channel to take advantage of the low latency that it offers. Reboot.

.. code-block:: bash

    $ sudo reboot now

You should now be able to see the network in the list of available networks. Connect to the network from the remote desktop. You should still be able to access the internet as the RPi is connected to the Ethernet and the routing demon will effectively convert the Pi in a router.

Check the latency of the connection from your connected laptop:

.. code-block:: bash

    $ ping 192.168.XX.1
    PING 192.168.4.1 (192.168.4.1): 56 data bytes
    64 bytes from 192.168.4.1: icmp_seq=0 ttl=64 time=2.181 ms
    64 bytes from 192.168.4.1: icmp_seq=1 ttl=64 time=2.084 ms
    64 bytes from 192.168.4.1: icmp_seq=2 ttl=64 time=1.975 ms
    64 bytes from 192.168.4.1: icmp_seq=3 ttl=64 time=1.601 ms
    64 bytes from 192.168.4.1: icmp_seq=4 ttl=64 time=1.088 ms
    64 bytes from 192.168.4.1: icmp_seq=5 ttl=64 time=2.001 ms
    64 bytes from 192.168.4.1: icmp_seq=6 ttl=64 time=1.024 ms
    64 bytes from 192.168.4.1: icmp_seq=7 ttl=64 time=1.503 ms
    64 bytes from 192.168.4.1: icmp_seq=8 ttl=64 time=2.127 ms
    64 bytes from 192.168.4.1: icmp_seq=9 ttl=64 time=2.028 ms
    64 bytes from 192.168.4.1: icmp_seq=10 ttl=64 time=1.101 ms
    64 bytes from 192.168.4.1: icmp_seq=11 ttl=64 time=2.159 ms
    64 bytes from 192.168.4.1: icmp_seq=12 ttl=64 time=2.025 ms
    64 bytes from 192.168.4.1: icmp_seq=13 ttl=64 time=2.184 ms

You should see something less that 2ms for the packets.

Setting up a secure WiFi connection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to use the dongle to connect to a WiFi netwoork, but you don't want the passphrase for that network to be shown in clear in the ``wpa_supplicant.conf`` follow the following steps:

First connect to the nework by entering the passphrase in clear:

.. code-block:: bash

    $ sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

And enter the acces information as:

.. code-block::

    network={
        ssid="testing"
        psk="testingPassword"
    }

Change the SSID and and the PSK to the ones specific for your network and make sure that the connection works by triggering the reconfiguration of the the interface:

.. code-block:: bash

    wpa_cli -i wlan0 reconfigure

To encode the password you can use ``wpa_passphrase``:

.. code-block:: bash

    $ wpa_passphrase "testing" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf > /dev/null

The program will wait fro you to enter the password (it will be displayed while you enter it, so please be careful) and it will update the ``wpa_supplicant.conf`` file with the encripted value, so you will now have two sets of details for the same SSID:

.. code-block::

    network={
      ssid="testing"
      psk="testingPassword"
    }
    # reading passphrase from stdin
    network={
      ssid="testing"
      #psk="testingPassword"
      psk=131e1e221f6e06e3911a2d11ff2fac9182665c004de85300f9cac208a6a80531
    }

And now you can delete the line with the password in clear and the previous section with the details in clear.

Setting-up the Bluetooth Keyboard
---------------------------------

Run ``bluetoothctl`` to pair the Keyboard.

.. code-block:: bash

    $ bluetoothctl
    Agent registered

At the changed prompt (that indicates we are in the tool) scan the available devices and make sure that the keyboard is in pair mode by pressing the bluetooth key:

.. code-block:: bash

    [bluetooth]# scan on
    Discovery started
    [CHG] Controller DC:A6:32:49:3B:78 Discovering: yes
    [NEW] Device 8C:85:90:D3:B8:8B 8C-85-90-D3-B8-8B
    [NEW] Device 64:DF:3D:3E:56:6E 64-DF-3D-3E-56-6E
    [NEW] Device 53:1D:2E:57:C3:4D 53-1D-2E-57-C3-4D
    [CHG] Device 64:DF:3D:3E:56:6E RSSI: -53
    [CHG] Device 53:1D:2E:57:C3:4D ManufacturerData Key: 0x004c
    [CHG] Device 53:1D:2E:57:C3:4D ManufacturerData Value:
    0c 0e 00 5e bd a0 e3 8c 88 26 80 fc ae 05 00 72  ...^.....&.....r
    [NEW] Device CC:C5:0A:23:20:4E CC-C5-0A-23-20-4E
    [CHG] Device CC:C5:0A:23:20:4E LegacyPairing: no
    [CHG] Device CC:C5:0A:23:20:4E Name: Bluetooth 3.0 Macro Keyboard
    [CHG] Device CC:C5:0A:23:20:4E Alias: Bluetooth 3.0 Macro Keyboard
    [CHG] Device 53:1D:2E:57:C3:4D ManufacturerData Key: 0x004c
    [CHG] Device 53:1D:2E:57:C3:4D ManufacturerData Value:
    0c 0e 00 5f bd 19 f2 f8 d5 8b 42 cd 0f 66 d9 ea  ..._......B..f..
    [CHG] Device 64:DF:3D:3E:56:6E RSSI: -64
    [CHG] Device CC:C5:0A:23:20:4E LegacyPairing: yes

The keyboard should be listed along with its addess. We will use this address to initiate pairing:

.. code-block:: bash

    [bluetooth]# pair CC:C5:0A:23:20:4E
    Attempting to pair with CC:C5:0A:23:20:4E
    [CHG] Device CC:C5:0A:23:20:4E Connected: yes
    [CHG] Device CC:C5:0A:23:20:4E Modalias: usb:v05ACp8502d011B
    [CHG] Device CC:C5:0A:23:20:4E UUIDs: 00001000-0000-1000-8000-00805f9b34fb
    [CHG] Device CC:C5:0A:23:20:4E UUIDs: 00001124-0000-1000-8000-00805f9b34fb
    [CHG] Device CC:C5:0A:23:20:4E UUIDs: 00001200-0000-1000-8000-00805f9b34fb
    [CHG] Device CC:C5:0A:23:20:4E ServicesResolved: yes
    [CHG] Device CC:C5:0A:23:20:4E Paired: yes
    Pairing successful
    [CHG] Device CC:C5:0A:23:20:4E ServicesResolved: no
    [CHG] Device CC:C5:0A:23:20:4E Connected: no

Once paired we will connect to it:

.. code-block:: bash

    [bluetooth]# connect CC:C5:0A:23:20:4E
    Attempting to connect to CC:C5:0A:23:20:4E
    [CHG] Device CC:C5:0A:23:20:4E Connected: yes
    Connection successful
    [CHG] Device CC:C5:0A:23:20:4E ServicesResolved: yes
    [CHG] Device 53:1D:2E:57:C3:4D ManufacturerData Key: 0x004c
    [CHG] Device 53:1D:2E:57:C3:4D ManufacturerData Value:
    0c 0e 08 60 bd 99 ab 8f 8c 7a ac a1 36 f9 2c 32  .........z..6.,2
    [Bluetooth 3.0 Macro Keyboard]# trust CC:C5:0A:23:20:4E
    [CHG] Device CC:C5:0A:23:20:4E Trusted: yes
    Changing CC:C5:0A:23:20:4E trust succeeded

We can now leave the tool:

.. code-block:: bash

    [Bluetooth 3.0 Macro Keyboard]# quit

Installing ROS Noetic
---------------------

We will now install ROS Noetic from sources.

Setup the repo
~~~~~~~~~~~~~~

.. code-block:: bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
    $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    $ sudo apt update

You should see additional sources listed:

.. code-block::

    Get:3 http://packages.ros.org/ros/ubuntu buster InRelease [4,671 B]
    Get:4 http://packages.ros.org/ros/ubuntu buster/main armhf Packages [28.6 kB]

Install dependencies
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    $ sudo apt-get install -y python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall build-essential cmake

We now need to initialize ``rosdep``:

.. code-block:: bash

    $ sudo rosdep init
    $ rosdep update

Setup a catkin space
~~~~~~~~~~~~~~~~~~~~

Because we install from sources we need to download all packages to a catkin workspace.

.. code-block:: bash

    $ mkdir ~/ros_catkin_ws
    $ cd ~/ros_catkin_ws

We now create a generator with a given variant of ROS (collection of packages). In out case we will use the robot and perception (that does not include GUI components like ``rviz`` and ``rqt`` as we will not run those on the robot).

.. code-block:: bash

    $ rosinstall_generator robot perception --rosdistro noetic --deps --wet-only --tar > noetic-ros_MH5-wet.rosinstall

This will create a file ``noetic-ros_MH5-wet.rosinstall`` that contains the details for the packages that need to be installed. It is used in by ``wstool`` in the next step.


Fetching the packages
~~~~~~~~~~~~~~~~~~~~~

We now use the ``wstool`` to download the packages that were specificated by the ``rosinstall_generator`` in the ``noetic-ros_MH5-wet.rosinstall`` file.

.. code-block:: bash

    $ wstool init src noetic-ros_MH5-wet.rosinstall

This will take some minutes to download all these packages and place them in the ``src/`` directory in the workspace. Then before compiling the packages in the ``src`` folder, we install all system dependencies using ``rosdep install``:

.. code-block:: bash

    $ rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster


Building the packages
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    $ sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3


Switching to ``catkin_tools``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To build the ROS packages in the future we will use ``catkin_tools`` that we need first to install:

.. code-block:: bash

    $ sudo apt-get install python-catkin-tools

Then we can run:

.. code-block:: bash

    $ catkin build

Updating the installation
~~~~~~~~~~~~~~~~~~~~~~~~~

If you need to add / update packages then the following activities must be performed:

.. code-block:: bash

    $ mv -i noetic-ros_MH5-wet.rosinstall noetic-ros_MH5-wet.rosinstall.old
    $ rosinstall_generator <packages> --rosdistro noetic --deps --tar > noetic-ros_MH5-wet.rosinstall

You can use ``diff`` to see what will be updated:

.. code-block:: bash

    $ diff -u noetic-desktop.rosinstall noetic-desktop.rosinstall.old

Incorporate the changes:

.. code-block:: bash

    $ vcs import --input noetic-desktop.rosinstall ./src

