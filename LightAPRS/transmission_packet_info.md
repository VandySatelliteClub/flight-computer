## Example transmission from LightAPRS (what shows up on the APRS.fi website)

**2023-04-22 14:20:39** = timestamp never part of telemetry_buff - starts at element 26 of packet ptr in APRS_sendLocWtTmStmp\
**1 MPH 0° alt 709 ft**  = part of telemetry_buff comes after timestamp, long, and lat within raw string - parsed to this format by APRS.fi\
**005TxC 23.30C 995.22hPa 6.34V 05S "http://www.lightaprs.com"** = comment variable within main ino file - becomes part of telemetry_buff\ 
**LightAPRS-W by TA2NHP & TA2MUN** = StatusMessage variable within main ino file


*005TxC 23.30C 995.22hPa 6.34V 05S* refers to elements 16 through 53 of the telemetry buff


## Example of raw packet sent on APRS.fi website - will be different from our string but structure is same
2023-10-08 14:34:38 CDT: N0AWA>APDR16,TCPIP*,qAC,T2ROMANIA:=3618.04N/08648.73Wu170/058/A=000537 Have a wonderful day
[timestamp date/time] [symbolTable and symbol]            [latitude / longitude heading/ speed / A= Altitude] [telemetry_buff and additional messages]


## Additional information on the parser for APRS
- https://aprs.fi/doc/guide/aprsfi-raw-packets.html\
- https://metacpan.org/pod/Ham::APRS::FAP
