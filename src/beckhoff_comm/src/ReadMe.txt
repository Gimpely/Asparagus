Communication between PC (ros) and PLC (Beckhoff) is established via ADS library which was developed by Beckhoff

We are sending arrays in both directions -> From PLC to PC and from PC to PLC


DELTA ROBOT -> PLC to PC

    dataArray[0] -> Angle of first servo motor
    dataArray[1] -> Angle of second servo motor
    dataArray[2] -> Angle of third servo motor
    dataArray[3] -> Angle of first DC motor
    dataArray[4] -> Angle of second DC motor

DELTA ROBOT -> PC to PLC

    dataArray[0] -> Velocity of first servo motor
    dataArray[1] -> Velocity of second servo motor
    dataArray[2] -> Velocity of third servo motor
    dataArray[3] -> Velocity of first DC motor
    dataArray[4] -> Velocity of second DC motor
    dataArray[5] -> Timestamp when new velocity has been calculated (to prevent crashes of delta robot if communication is lost)