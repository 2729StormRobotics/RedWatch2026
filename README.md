# RedWatch 2026 FRC Robot Code

[![WPILib](https://img.shields.io/badge/WPILib-2026.1.1-blue.svg)](https://github.com/wpilibsuite/allwpilib)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This repository contains the source code for Team 2729's 2026 FRC robot, codenamed "RedWatch". The robot is built using WPILib and features advanced vision processing with Limelight 4 cameras and MegaTag 2 localization.

## Features

- **Advanced Vision System**: Utilizes dual Limelight 4 cameras with MegaTag 2 for precise field localization
- **Modular Design**: Clean separation of concerns with dedicated subsystems and commands
- **Advanced Logging**: Comprehensive data logging using AdvantageKit
- **Simulation Support**: Full support for simulation and unit testing
- **Path Planning**: Advanced path following and autonomous routines

## Prerequisites

- WPILib 2026.1.1 or later
- Java Development Kit (JDK) 17
- FRC Game Tools
- Git

## Getting Started

1. **Clone the repository**:
   ```bash
   git clone https://github.com/2729StormRobotics/RedWatch2026.git
   cd RedWatch2026
   ```

2. **Build the project**:
   ```bash
   ./gradlew build
   ```

3. **Deploy to robot**:
   ```bash
   ./gradlew deploy
   ```

## Project Structure

- `src/main/java/frc/robot/` - Main robot code
  - `commands/` - Command implementations
  - `subsystems/` - Subsystem implementations
  - `util/` - Utility classes and helpers
- `src/main/deploy/` - Deployment files and configuration
- `custom_assets/` - 3D models and custom assets
- `vendordeps/` - Vendor dependencies

## Documentation

- [Vision Subsystem](VISION_SUBSYSTEM.md) - Documentation for the vision system
- [AdvantageKit Logging](ADVANTAGEKIT_LOGGING.md) - Logging and data collection
- [WPILib Documentation](https://docs.wpilib.org/) - Complete WPILib documentation

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
