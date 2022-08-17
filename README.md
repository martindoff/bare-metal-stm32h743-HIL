<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="https://github.com/mcauser/MCUDEV_DEVEBOX_H7XX_M/blob/master/docs/STM32H7XX_M.jpg" alt="Logo" width="300" height="300">

  <h3 align="center">Bare metal stm32h7</h3>

  <p align="center">
    A bare metal implementation of a blinky example for the stm32h743 with a simple makefile and without using HAL libraries. 
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
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Here is a simple blinky example to get started with the stm32 H7 family. This project does not rely on HAL libraries and the code can be built and flashed using GNU make (so that you do not need any IDE such as STM32CubeIDE) and the GNU ARM Embedded Toolchain. The code was tested with the stm32h743vit6 development board from <a href="https://github.com/mcauser/MCUDEV_DEVEBOX_H7XX_M">DevEBox</a> but could be easily adapted for any configuration. The board can be purchased on  <a href="https://www.banggood.com/STM32H750VBT6-or-STM32H743VIT6-STM32H7-Development-Board-STM32-System-Board-M7-Core-Board-TFT-Interface-with-USB-Cable-p-1661383.html?cur_warehouse=CN&ID=6288383">Banggood</a>. 

The system clock frequency is set at 480MHz, assuming the presence of a 25MHz high speed external (HSE) crystal. If you do not use a HSE or if you have an older version of the chip* you might have to modify the clock configuration function or rely on the default internal oscillator (64MHz).  

*Note that stmicroelectronics recently introduced a new version of their chip (version V) able to operate at up to 480MHz.


### Built With

* [GNU make](https://www.gnu.org/software/make/)
* [GNU ARM Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* C



<!-- GETTING STARTED -->
## Getting Started


### Prerequisites

You need to install the following:
* GNU make
* git 
* st-link 
* GNU ARM toolchain 

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/martindoff/bare-metal-stm32h7.git
   ```
2. Go to directory 
   ```sh
   cd bare-metal-stm32h7
   ```
3. Build
   ```sh
   make
   ```
3. Flash the board (connect via st-link V2 debugger) 
   ```sh
   make flash
   ```

<!-- ROADMAP -->
## Roadmap

Starting from a simple blinky example, other features can be implemented to explore the capabilities of the board (e.g. test peripherals such as UART, SPI, I2C, ..., test SD card port, LCD screen, etc.) . 
See the [open issues](https://github.com/martindoff/bare-metal-stm32h7/issues) for a list of proposed features (and known issues).



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



<!-- CONTACT -->
## Contact

Martin Doff-Sotta - martin.doff-sotta@eng.ox.ac.uk

Linkedin: https://www.linkedin.com/in/mdoffsotta/

Project Link: [https://github.com/martindoff/bare-metal-stm32h7](https://github.com/martindoff/bare-metal-stm32h7)





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
