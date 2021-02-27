<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

[comment]: <> ([![Contributors][contributors-shield]][contributors-url])

[comment]: <> ([![Forks][forks-shield]][forks-url])

[comment]: <> ([![Stargazers][stars-shield]][stars-url])

[comment]: <> ([![Issues][issues-shield]][issues-url])

[comment]: <> ([![MIT License][license-shield]][license-url])

[comment]: <> ([![LinkedIn][linkedin-shield]][linkedin-url])



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Sensor Based Object Compliance Test</h3>

  <p align="center">
    This project initiates a novel approach towards the compliance test of the object!
    <br />

[comment]: <> (    <a href="https://github.com/othneildrew/Best-README-Template"><strong>Explore the docs »</strong></a>)

[comment]: <> (    <br />)

[comment]: <> (    <br />)

[comment]: <> (    <a href="https://github.com/othneildrew/Best-README-Template">View Demo</a>)

[comment]: <> (    ·)

[comment]: <> (    <a href="https://github.com/othneildrew/Best-README-Template/issues">Report Bug</a>)

[comment]: <> (    ·)

[comment]: <> (    <a href="https://github.com/othneildrew/Best-README-Template/issues">Request Feature</a>)
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
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
    <li><a href="#build">Build</a></li>
    <li><a href="#Run">Run</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

There are several projects and online research papers, journals and conference papers that 
deal with the compliance test of the object. However, some uses image processing, or machine
learning or deep learning.
In our project, we introduce a novel approach of using Point Cloud to determine the compliance
as well how to pack the object within a package which is somewhat smaller than the object,
given that it is satisfying the compliance test.

### Built With

The project was (still in process) using the following framework and hardware:
* [ROS](https://www.ros.org/)
* [C++](https://www.cplusplus.com/)
* [Intel Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)
* [KUKA LBW iiwa R820 Robot](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa)




<!-- GETTING STARTED -->
## Getting Started

To get started with the project, you need to have the following prerequisites installed
in your system.


### Prerequisites

* ROS

  The ROS (Robot Operating System) is a free software, sourced and maintained by 
  [Open Robotics](https://www.openrobotics.org/). You must have ROS in order to run this project on your system
  
  
### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   https://github.com/sshaizkhan/amazon_ws.git
   ```
3. Install [Realsense ROS](https://github.com/IntelRealSense/realsense-ros) library by following the installation process explained on the website

#### Note: 
It's important to install the Realsense in the /src folder of the project

<!-- USAGE EXAMPLES -->
## Build

Once you have all the necessary files and hardware arranged, you can build your project.

```sh
cd your_project_directory
catkin_make
```
In order to save processing power, you can also use:

```sh
catkin_make -j7
```
Here, 7 denotes the number of cores you want to assign during make process, since it's high
power consuming process. You can assign however amount of core you want.
<!-- ROADMAP -->

## Run

After building the project, you are ready to run the project and various nodes in order
to acomplish the task.

```sh
roslaunch your_package_name your_launch_file.launch
```


## Usage
Check out this amazing video, where a 7 dof KUKA LBR iiwa robot is picking the package, 
placing it over the poke bed checking for compliance using side mounted Realsense depth 
camera and then packaging it into a packet smaller than the package.

![Packaging video](https://drive.google.com/file/d/1ZqCYGY2boIU0i4A6bUEENG2Xz630SbSV/view?usp=sharing)


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

Your Name - [Shahwaz Khan](sshaizkhan@gmail.com)

Project Link:  [Amazon_ws](https://github.com/sshaizkhan/amazon_ws.git)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

[comment]: <> (* [GitHub Emoji Cheat Sheet]&#40;https://www.webpagefx.com/tools/emoji-cheat-sheet&#41;)

[comment]: <> (* [Img Shields]&#40;https://shields.io&#41;)

[comment]: <> (* [Choose an Open Source License]&#40;https://choosealicense.com&#41;)

[comment]: <> (* [GitHub Pages]&#40;https://pages.github.com&#41;)

[comment]: <> (* [Animate.css]&#40;https://daneden.github.io/animate.css&#41;)

[comment]: <> (* [Loaders.css]&#40;https://connoratherton.com/loaders&#41;)

[comment]: <> (* [Slick Carousel]&#40;https://kenwheeler.github.io/slick&#41;)

[comment]: <> (* [Smooth Scroll]&#40;https://github.com/cferdinandi/smooth-scroll&#41;)

[comment]: <> (* [Sticky Kit]&#40;http://leafo.net/sticky-kit&#41;)

[comment]: <> (* [JVectorMap]&#40;http://jvectormap.com&#41;)

[comment]: <> (* [Font Awesome]&#40;https://fontawesome.com&#41;)





[comment]: <> (<!-- MARKDOWN LINKS & IMAGES -->)

[comment]: <> (<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->)

[comment]: <> ([contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge)

[comment]: <> ([contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors)

[comment]: <> ([forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge)

[comment]: <> ([forks-url]: https://github.com/othneildrew/Best-README-Template/network/members)

[comment]: <> ([stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge)

[comment]: <> ([stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers)

[comment]: <> ([issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge)

[comment]: <> ([issues-url]: https://github.com/othneildrew/Best-README-Template/issues)

[comment]: <> ([license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge)

[comment]: <> ([license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt)

[comment]: <> ([linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555)

[comment]: <> ([linkedin-url]: https://linkedin.com/in/othneildrew)

[comment]: <> ([product-screenshot]: images/screenshot.png)
