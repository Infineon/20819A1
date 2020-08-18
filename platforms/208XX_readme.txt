------------------------------------------------------------------------------------
BT 208XX SDK
------------------------------------------------------------------------------------

Overview
--------
The Cypress CYW20819, CYW20820, and CYW89820 are ultra-low-power dual-mode
Bluetooth 5.0 wireless MCU devices. They have a stand-alone baseband processor
with an integrated 2.4 GHz transceiver supporting BR/EDR/BLE.  They are
identical except that the CYW20820 and CYW89820 have an internal Power
Amplifier (iPA) with programmable BDR transmit power up to 10.5 dBm.  CYW208XX
is used to refer to all those parts.  Similarly, CYW9208XXEVB is used to refer
to the CYW920819EVB-02, CYW920820EVB-02, and CYW989820EVB-01 boards.

SDK Software Features
----------------------
- Dual mode Bluetooth stack included in the ROM (BR/EDR and BLE).
- BT stack and profile level APIs for embedded BT application development.
- WICED HCI protocol to simplify host/MCU application development.
- APIs and drivers to access on board peripherals
- Bluetooth protocols include GAP, GATT, SMP, RFCOMM, SDP, AVDT/AVCT, BLE Mesh
- BLE and BR/EDR profile APIs, libraries and sample apps
- Support for Over-The-Air (OTA) upgrade.
- Device Configurator for creating custom pin mapping.
- Bluetooth Configurator for creating BLE GATT Database.
- Documentation for APIs, datasheet, profiles and features.

Kits
----
CYW920819EVB-02:
    62-FBGA package, Arduino compatible headers, 9-axis motion sensor and
    thermistor, user switches and LEDs, USB connector for power, programming
    and USB-UART bridge.
    Note: Max UART baud rate is 3M
    For more information, see - http://www.cypress.com/CYW920819EVB-02
CYBT-213043-MESH:
    35-SMT package, PIR sensor (motion detection), Ambient Light Sensor and
    thermistor, user switches and RGB LEDs, with additional 1MB External Serial
    Flash.
    CYW20819-based dual-mode (BLE/BR/EDR) Bluetooth 5.0 with SIG MESH Qualified
    Module, FCC, ISED, MIC, and CE Certified Module.
    USB connector for power, programming and USB-UART bridge.
    Note: Max UART baud rate is 1M. Use baud rate of 115200 for Client Control.
    For more information, see - http://www.cypress.com/CYBT-213043-MESH
CYW920820EVB-02:
    62-FBGA package, Arduino compatible headers, 9-axis motion sensor and
    thermistor, user switches and LEDs, USB connector for power, programming
    and USB-UART bridge, 10.5 dBm internal power amplifier.
    Note: Max UART baud rate is 3M
    For more information, see - http://www.cypress.com/CYW920820EVB-02
CYW989820EVB-01:
    48-WQFN package, Arduino compatible headers, 9-axis motion sensor and
    thermistor, user switches and LEDs, USB connector for power, programming
    and USB-UART bridge, 10.5 dBm internal power amplifier.
    Note: Max UART baud rate is 3M
    For more information, see - https://www.cypress.com/products/automotive-wireless
CYBT-213043-EVAL:
    35-SMT package, Arduino compatible headers. CYW20819-based dual-mode
    (BLE/BR/EDR) Bluetooth 5.0-compliant fully certified module
    (CYBT-213043-02).
    Note: Max UART baud rate is 1M. Use baud rate of 115200 for Client Control.
    For more information, see - http://www.cypress.com/CYBT-213043-EVAL

Software Tools
--------------
Following applications are installed with ModusToolbox on your computer.

BT Spy :
    BTSpy is a trace viewer utility that can be used with WICED BT platforms to
    view protocol and application trace messages from the embedded device. The
    utility is located in the folder below. For more information, see readme.txt
    in the same folder.
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox IDE QuickPanel, or by right-clicking
    a project in the IDE Project Explorer pane and selecting the ModusToolbox
    context menu.
    It is supported on Windows, Linux and macOS.
    <Workspace Dir>\wiced_btsdk\tools\btsdk-utils\BTSpy

BT/BLE Profile Client Control:
    This application emulates host MCU applications for BLE and BR/EDR profiles.
    It demonstrates WICED BT APIs. The application communicates with embedded
    apps over the WICED HCI interface. The application is located in the folder
    below. For more information, see readme.txt in the same folder.
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox IDE QuickPanel, or by right-clicking
    a project in the IDE Project Explorer pane and selecting the ModusToolbox
    context menu.
    It is supported on Windows, Linux and macOS.
    <Workspace Dir>\wiced_btsdk\tools\btsdk-host-apps-bt-ble\client_control

BLE Mesh Client Control:
    Similar to the above app, this application emulates host MCU applications
    for BLE Mesh models. It can configure and provision mesh devices and create
    mesh networks. The application is located in the folder below. For more
    information, see readme.txt in the same folder.
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox IDE QuickPanel (if a mesh-capable
    project is selected in the IDE Project Explorer pane), or by right-clicking
    a mesh-capable project in the IDE Project Explorer pane and selecting the
    ModusToolbox context menu.
    The full version is provided for Windows (VS_ClientControl) supporting all
    Mesh models.
    A limited version supporting only the Lighting model (QT_ClientControl) is
    provided for Windows, Linux, and macOS.
    <Workspace Dir>\wiced_btsdk\tools\btsdk-host-apps-mesh

Peer apps:
    Applications that run on Windows, iOS or Android and act as peer BT apps to
    demonstrate specific profiles or features, communicating with embedded apps
    over the air.
    BLE apps location:
    <Workspace Dir>\wiced_btsdk\tools\btsdk-peer-apps-ble
    BLE Mesh apps location:
    <Workspace Dir>\wiced_btsdk\tools\btsdk-peer-apps-mesh
    OTA apps location:
    <Workspace Dir>\wiced_btsdk\tools\btsdk-peer-apps-ota

Device Configurator:
    Use this tool to create a custom pin mapping for your device. Run this tool
    from the Tools section of the ModusToolbox IDE QuickPanel, or by
    right-clicking a project in the IDE Project Explorer pane and selecting the
    ModusToolbox context menu.
    It is supported on Windows, Linux and macOS.
    Note: The pin mapping is based on wiced_platform.h for your board.
    Location:
    <Install Dir>\tools_2.0\device-configurator

Bluetooth Configurator:
    Use this tool to create and configure the BLE GATT Database for your
    application.
    Run this tool from the Tools section of the ModusToolbox IDE QuickPanel, or
    by right-clicking a project in the IDE Project Explorer pane and selecting
    the ModusToolbox context menu.
    It is supported on Windows, Linux and macOS.
    Location:
    <Install Dir>\tools_2.0\bt-configurator

Power Estimator:
    Use this application to get an estimate of power consumed by your
    application, running on CYW920819EVB-02 kit.
    Run this tool from the Tools section of the ModusToolbox IDE QuickPanel, or
    by right-clicking a project in the IDE Project Explorer pane and selecting
    the ModusToolbox context menu.
    It is supported on Windows, Linux and macOS.
    Location:
    <Install Dir>\tools_2.0\cype-tool

Tracing
-------
To view application traces, there are 2 methods available. Note that the
application needs to configure the tracing options.
1. WICED Peripheral UART - Open this port on your computer using a serial port
utility such as Tera Term or PuTTY (usually using 115200 baud rate).
2. WICED HCI UART - Open this port on your computer using the Client Control
application mentioned above (usually using 3M baud rate). Then run the BT Spy
utility mentioned above.

Application Settings
--------------------
Application settings can be changed in the application makefile. Options listed
below are available for all applications. Other application specific options may
also be available and are documented in the readme.txt for those applications.

BT_DEVICE_ADDRESS
    Set BT device address for your BT device. The BT address is 6 bytes,
    for example 20819A10FFEE. By default, the SDK will set a BDA for your device
    by combining the 7 hex digit device ID with the last 5 hex digits of the
    host PC MAC address.
UART
    Configure the UART port you want the application to be downloaded. For
    example 'COM6' on Windows or '/dev/ttyWICED_HCI_UART0' on Linux or
    '/dev/tty.usbserial-000154' on macOS.
    By default, the SDK will auto detect the port.
ENABLE_DEBUG
    For HW debugging, see the document WICED-Hardware-Debugging.pdf for more
    information. Configuring this setting with value =1 configures GPIO for SWD.
    CYW920819EVB-02/CYW920820EVB-02: SWD signals are shared with D4 and D5, see
    SW9 in schematics.
    CYBT-213043-MESH/CYBT-213043-EVAL: SWD signals are routed to P12=SWDCK and
    P13=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK and SWDIO
    to your SWD Debugger probe.
    CYW989820EVB-01: SWDCK (P02) is routed to the J13 DEBUG connector, but not
    SWDIO. Add a wire from J10 pin 3 (PUART CTS) to J13 pin 2 to connect GPIO
    P10 to SWDIO.
POWER_ESTIMATOR
    For power estimation, configure this setting with value =1. See the
    cype-tool help for more information.
    This setting enables power estimation feature of your app on the kit.

Downloading application to kit
------------------------------
If you have issues downloading to the kit, follow the steps below:
- Press and hold the 'Recover' button on the kit.
- Press and hold the 'Reset' button on the kit.
- Release the 'Reset' button.
- After one second, release the 'Recover' button.

Over The Air (OTA) Firmware Upgrade
-----------------------------------
Applications that support OTA upgrade can be updated via the peer OTA app in:
<Workspace Dir>\wiced_btsdk\tools\btsdk-peer-apps-ota
See the readme.txt file located in the above folder for instructions.
To generate OTA image for the app, configure OTA_FW_UPGRADE=1 in the app
makefile, or append OTA_FW_UPGRADE=1 to a build command line, for example:
> make PLATFORM=CYW920819EVB-02 OTA_FW_UPGRADE=1 build
This will generate <app>.bin file in the 'build' folder.


------------------------------------------------------------------------------------
