#!/bin/bash

# from https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/wpilibNewCommands/WPILibNewCommands.json
echo =================== updating WPILib ===================
gradle vendordep --url=https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/wpilibNewCommands/WPILibNewCommands.json

# from https://store.ctr-electronics.com/software/
echo =================== updating Phoenix libs ===================
gradle vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenixpro/PhoenixProAnd5-frc2023-latest.json

# from https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html
echo =================== updating PhotonVision libs ===================
gradle vendordep --url=https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

# from https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#online-installation
echo =================== updating Rev libs ===================
gradle vendordep --url=https://software-metadata.revrobotics.com/REVLib-2024.json

# from https://www.playingwithfusion.com/docview.php?docid=1205
echo =================== updating PlayingWithFusion libs ===================
gradle vendordep --url=https://www.playingwithfusion.com/frc/playingwithfusion2024.json
