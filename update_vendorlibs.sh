#!/bin/bash
echo =================== Resetting vendorlibs ===================
rm -rf vendordeps/*

echo =================== cleaning project ===================
gradle clean

# from https://github.com/wpilibsuite/2025Beta
echo =================== updating WPILib ===================
gradle vendordep --url=https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/wpilibNewCommands/WPILibNewCommands.json

# from https://store.ctr-electronics.com/software/
# or https://github.com/CrossTheRoadElec/Phoenix-Releases/releases
echo =================== updating Phoenix libs ===================
gradle vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
gradle vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2025-latest.json

# from https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html
#echo =================== updating PhotonVision libs ===================
#gradle vendordep --url=https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

# from https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#online-installation
echo =================== updating Rev libs ===================
gradle vendordep --url=https://software-metadata.revrobotics.com/REVLib-2025.json

# from https://www.playingwithfusion.com/docview.php?docid=1205
#echo =================== updating PlayingWithFusion libs ===================
#gradle vendordep --url=https://www.playingwithfusion.com/frc/playingwithfusion2024.json
#grep -q frcYear vendordeps/playingwithfusion2024.json || echo Patching PlayingWithFusion to set year as 2025
#sed -i '/"version": "2024.03.09",/a "frcYear": "2025",' vendordeps/playingwithfusion2023.json
