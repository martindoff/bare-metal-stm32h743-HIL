<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="https://github.com/martindoff/bare-metal-stm32h743-HIL/blob/main/img/cover.png" alt="Logo" width="900" height="400">

  <h3 align="center">Bare metal stm32h7</h3>

  <p align="center">
    A Hardware-In-the-Loop (HIL) simulation example for the stm32h743 microcontroller. 
    <br />
    
  </p>
  
  
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
        <li><a href="#detailed-description">Detailed description</a></li> 
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#acknowledgment">Acknowledgment</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This example shows how to establish a serial communication between the stm32h743 microcontroller and Simulink running on a development computer in order to perform Hardware-In-the-Loop (HIL) simulations. The serial connection between the board and the development computer is enabled using the UART protocole so that data are exchanged serially between the host and the target. 
A case study is considered where a simplified point-mass model of an aircraft in Simulink is controlled by the microcontroller with a PID.

This project does not rely on HAL libraries and the code can be built and flashed using GNU make (so that you do not need any IDE such as STM32CubeIDE) and the GNU ARM Embedded Toolchain. The code was tested with the stm32h743vit6 development board from <a href="https://github.com/mcauser/MCUDEV_DEVEBOX_H7XX_M">DevEBox</a> but could be easily adapted for any configuration. The board can be purchased on  <a href="https://www.banggood.com/STM32H750VBT6-or-STM32H743VIT6-STM32H7-Development-Board-STM32-System-Board-M7-Core-Board-TFT-Interface-with-USB-Cable-p-1661383.html?cur_warehouse=CN&ID=6288383">Banggood</a>. 

### Built With

* [GNU make](https://www.gnu.org/software/make/)
* [GNU ARM Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* C
* [Matlab / Simulink](https://uk.mathworks.com/products/matlab.html)

### Detailed description
The software consists of two parts: the program running on the microcontroller (stm32h743) and the Simulink model for the simulator. 

#### 1) stm32h743
The program on the stm32h743 configures the PB12, PB13 pins for UART5. The UART protocole is kept in the default configuration (`8N1`) and the baud rate is set to `38400`.

The main loop sequentially receives feedback data from the aircraft model in Simulink, computes a control law using a PID controller, and sends the control command back to the simulator. 

The receive and send functions are implemented as follows, relying on the UART peripheral (`UART5`): 
```C
/**
  * Send in blocking mode
  */
void UART_send_blocking(uint8_t* byte)
{
    while(!(UART5->ISR & USART_ISR_TXE_TXFNF)){}; // wait for empty transmit register
    UART5->TDR = *byte;
}

/**
  * Receiving in blockin mode 
  */
void UART_rcv_blocking(uint8_t* byte)
{
    while(!(UART5->ISR & USART_ISR_RXNE_RXFNE)){}; // wait for non empty read register
    *byte = UART5->RDR;

}
```

The blocking mode allows us to make sure that the transmit register is empty before writing into it (for sending to Simulink) and that the read register is non empty before reading it (to get data from Simulink).

Since the data have to be sent/received byte per byte as per the UART protocole (8 bits at a time), and since we want to manipulate objects with type `float` (4 bytes), we define the following union type: 
```C
typedef union {  // allows us to store different data types in the same memory location
  float single;
  uint8_t bytes[4];
} custom_float_t;
```

That way we can store / access the data byte per byte when we receive / send, or 4 bytes at a time when we use the object inside the code. For example, when receiving data, we use: 
```C
custom_float_t rcv;
// Reception from Simulink
for (int i=0; i<4; i++)  // 1 float = 4 bytes
{
    UART_rcv_blocking(&rcv.bytes[i]);  // receive in blocking mode, 1 byte at a time (= uint8_t)
}
```
which allows us to store the data byte per byte as they are received into the union type variable `rcv`. Then, we can store the 4 bytes of data at a time into a variable with `float` type as follows
```C
float TAS = rcv.single;  // store the 4 bytes as a float
```

Once the true airspeed of the aircraft is received and stored in a variable, a PID controller is implemented, computing a control law $u_k$ of the form: 

$$ u_k = k_p (v_k^* - v_k) + k_i \sum_{i=0}^k (v_i^* - v_i) d + k_d (v_{k-1} - v_{k})/d $$

where $v_k$, $v_k^*$ are the TAS and reference TAS at step $k$, $k_p$, $k_i$, $k_d$ are PID gains, and $d$ is the time step. 

When sending data back to Simulink, note that a header (`'A'`) and a terminator (`'\0'`) character should be prepended and appended to the data sent in order to improve the robustness of the data exchange, allowing Simulink to synchronise with the data sent by the microcontroller. 



Note: the system clock frequency is set to 480MHz, assuming the presence of a 25MHz high speed external (HSE) crystal. If you do not use a HSE or if you have an older version of the chip you might have to modify the clock configuration function or rely on the default internal oscillator (64MHz). In that case you will have to change the baud rate register (e.g. with the internal oscillator at 64MHz and baud rate of 38400, the BRR should be modified with: uint16_t uartdiv = 64000000 / 38400;)

#### 2) Simulink

<img src="https://github.com/martindoff/bare-metal-stm32h743-HIL/blob/main/img/sim.png" alt="Logo" width="600" height="300">


The Simulink model can be separated into two main subsystems: the "COM to microcontroller" block responsible for data exchange and the "Aircraft model" block. The latter implements a simple point-mass model of an aircraft in level flight and take the commanded thrust (from the microcontroller) as input and outputs the updated true airspeed (TAS). The TAS is then fed back to the "COM to microcontroller" block so that a control law is implemented on the hardware. 

We will mainly discuss the "COM to microcontroller" block here as the physics of the aircraft is irrelevant and just serves as an example for this case study. The content of the "COM to microcontroller" subsystem is as follows:

<img src="https://github.com/martindoff/bare-metal-stm32h743-HIL/blob/main/img/COM_module.png" alt="Logo" width="900" height="400">

The reception chain (upper part of the figure) consists of the "Receive from stm32h743" and "Cast to double" blocks. They are responsible for acquiring the control command computed on the stm32h7 chip and converting the data for use in the simulation.
The transmission chain (lower part of the figure) consists of the "Zero Order Hold", "Cast to single", "Byte pack" and "Send to stm32h743" blocks. These blocks are responsible for respectively sampling, converting, packing and sending the data from the simulation to the chip. 
Finally, the "UART Configuration" block is responsible for configuring the serial communication.

We now detail the configuration of all blocks

#### "Receive from stm32h743":
Acquire data from the chip. Configuration: 
* **COM port name:** specify the name of the COM port associated with the device (in my setup it was `/dev/cu.usbserial-14201` but yours will certainly have a different name).  
* **Header and terminator:** add header (`'A'`) and terminator (`'\0'`) characters. This allows Simulink to know when a message starts and ends and prevent synchronisation issues. 
*  **Data type:** set to `single` (4 bytes) as we receive a `float` from the microcontroller (both types are equivalent).  
* **Data size:** the data size is set to `[1 1]` as we send 1  `single` / `float` (change it to `[1 N]` if N `single` / `float` are sent). 
* **Enable blocking mode:** tick the box to receive in blocking mode.
* **Block sample time:** set to 0.1s.

#### "Cast to double":
Convert input data to double.

#### "Zero Order Hold":
Hold the input for the specified sample period. Configuration: 

* **sample time:** set to 0.1s. 

#### "Cast to single":
Convert input data to single.

#### "Byte pack":
Pack input data into a single output vector of required type. In our case, this has the effect of converting a `single` into 4 bytes. Configuration:

* **input type:** set to `{'single'}`.
* **output type:** set to `uint32_t` since 1 `single` is 4 bytes, i.e 1 `uint32_t`.
* **byte alignment:** set to 4 as we want the 4 bytes in the input `single` to be sent as 1 `uint32_t`.

#### "Send to stm32h743":
Send data to the chip. Configuration:

* **COM port name:** specify the name of the COM port associated with the device (same as previously)  
* **Enable blocking mode:** tick the box to send in blocking mode.

#### "UART Configuration":
Configure the serial port. Configuration:

* **COM port name:** specify the name of the COM port associated with the device (same as previously)  
* **Baud rate:** set to `38400`. Make sure to have the same baud rate as programmed on the microcontroller! 
* **Data bits:** set to `8` bits.
* **Parity:** none. 
* **Stop bits:** set to `1` stop bit. 
* **Byte order:** little endian. 
* **Flow control:** none.
* **Timeout:** `10`.  


The blocks configuration is summarised below: 

<img src="https://github.com/martindoff/bare-metal-stm32h743-HIL/blob/main/img/block_parameters.png" alt="Logo" width="1200" height="500">

Credit: this part of the project was inspired from the tutorial [here](https://github.com/leomariga/Simulink-Arduino-Serial). 

<!-- GETTING STARTED -->
## Getting Started


### Prerequisites

You need to install the following:
* GNU make
* git 
* st-link 
* GNU ARM toolchain 
* Matlab / Simulink, especially the Simulink Support Package for Arduino Hardware (sic) for the byte pack block and the Instrument Control Toolbox for the serial blocks

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/martindoff/bare-metal-stm32h743-HIL.git
   ```
2. Go to directory 
   ```sh
   cd bare-metal-stm32h743-HIL
   ```
3. Build
   ```sh
   make
   ```
3. Flash the microcontroller (connect via st-link V2 debugger) 
   ```sh
   make flash
   ```
4. Disconnect the st-link V2 debugger and connect the board to a development computer with a USB TTL serial adapter according to the following schematics:
<img src="https://github.com/martindoff/bare-metal-stm32h743-HIL/blob/main/img/stm32-uart.jpg" alt="Logo" width="300" height="300">

5. Find the name of the COM port. In macOS, this can be found in the /dev directory and starts by the prefix 
`/dev/cu`. 
6. Copy the 'Simulink' folder to your matlab path and run the initialisation script in the Matlab command window:
 ```matlab
   param_init
```
7. Open the simulink model 'uart.slx' and modify the name of the COM port in the serial configuration, send and receive blocks. If you do not find the name of your COM port in the drop down list, you might have to unplug/replug the board and shut down / restart Matlab / Simulink. 
8. Run the simulation and observe the true airspeed reach the 80m/s setpoint and the commanded thrust sent by the microcontroller to control the model.
<img src="https://github.com/martindoff/bare-metal-stm32h743-HIL/blob/main/img/results.png" alt="Logo" width="900" height="400">

9. Note that if you restart the simulation without unplugging or reseting the board, you will obtain a different result. This is because the integral term of the PID will have accumulated error values from the previous simulation and will thus have a different value than at initialisation. It is best to reset the chip before running any new instance of the simulation in order to avoid any surprises.

<!-- ROADMAP -->
## Roadmap

Starting from this HIL simulation example, more sophisticated controllers can be developed and tested on any Simulink model. Direct extensions of this project are: 

* sending / receiving arrays of `float` (exchanging several variables per time step).
* make sure that the control loop runs at the same frequency as the simulation. This involves for example implementing a precise delay function. 

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.


## Acknowledgment
Thanks to Leonardo Mariga for his detailed [tutorial](https://github.com/leomariga/Simulink-Arduino-Serial) on "How to connect Arduino and Simulink" that helped debugging the Simulink side of the present project  

<!-- CONTACT -->
## Contact

Martin Doff-Sotta - martin.doff-sotta@eng.ox.ac.uk

Linkedin: https://www.linkedin.com/in/mdoffsotta/





<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/github_username
