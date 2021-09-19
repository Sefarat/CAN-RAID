Readme file to show an example of how RAID protects against Persistent Bus Off Attacks.
This setup requires a minimum of three ECU: A sender, a receiver, and an attacker.
***************************************************************

Without RAID:
PersistentTime: The attacker's code. Attacks the ID defined as VICID
sendMult: ECU sending two IDs. VICID, and OTHID 
receive_check: listens to OTHID 

Instructions:
1-Upload receive_check to ECU1
2-Upload sendMult to ECU2
3-Watch the serial monitor of ECU1 print the messages it receives
4-Upload PersistentTime to ECU3
5-Watch serial monitor of ECU1 stop printing messages

***************************************************************

With RAID:

sendMultRAID: ECU sending two IDs. VICID, and OTHID encoded using RAID
receive_checkRAID: listens to OTHID encoded using RAID

Instructions:
1-Upload receive_checkRAID to ECU1
2-Upload sendMultRAID to ECU2
3-Watch the serial monitor of ECU1 print the messages it receives
4-Upload PersistentTime to ECU3
5-Watch serial monitor of ECU1 keep printing

