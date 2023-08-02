#!/bin/bash
echo updating Phoenix libs
gradle vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenixpro/PhoenixProAnd5-frc2023-latest.json
echo updating Photonvision libs
gradle vendordep --url=https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
echo updating Rev libs
gradle vendordep --url=https://software-metadata.revrobotics.com/REVLib-2023.json
