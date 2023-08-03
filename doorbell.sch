v 20130925 2
C 40000 40000 0 0 0 title-B.sym
C 51000 47100 1 0 0 opamp-1.sym
{
T 51700 47900 5 10 0 0 0 0 1
device=OPAMP
T 51700 47700 5 10 1 1 0 0 1
refdes=U1
T 51700 48500 5 10 0 0 0 0 1
symversion=0.1
T 51000 47100 5 10 0 0 0 0 1
slotdef=1:3,2,8,4,1
T 51000 47100 5 10 0 0 0 0 1
numslots=2
T 51000 47100 5 10 0 0 0 0 1
slot=2
T 51000 47100 5 10 0 0 0 0 1
slotdef=2:5,6,8,4,7
T 51000 47100 5 10 0 0 0 0 1
footprint=DIP8
}
C 52500 48000 1 0 0 npn-2.sym
{
T 53100 48500 5 10 0 0 0 0 1
device=NPN_TRANSISTOR
T 52800 48500 5 10 1 1 0 0 1
refdes=Q1
T 52500 48000 5 10 0 0 0 0 1
footprint=TO92b
}
C 52500 47000 1 180 1 pnp-2.sym
{
T 53100 46600 5 10 0 0 180 6 1
device=PNP_TRANSISTOR
T 52800 46500 5 10 1 1 180 6 1
refdes=Q2
T 52500 47000 5 10 0 0 180 6 1
footprint=TO92b
}
N 52500 48500 52000 48500 4
N 52000 46500 52000 48500 4
N 52000 46500 52500 46500 4
N 53000 47000 53000 48000 4
C 48300 47100 1 0 0 opamp-1.sym
{
T 49000 47900 5 10 0 0 0 0 1
device=OPAMP
T 49000 47700 5 10 1 1 0 0 1
refdes=U1
T 49000 48500 5 10 0 0 0 0 1
symversion=0.1
T 48300 47100 5 10 0 0 0 0 1
slotdef=1:3,2,8,4,1
T 48300 47100 5 10 0 0 0 0 1
numslots=2
T 48300 47100 5 10 0 0 0 0 1
slot=1
T 48300 47100 5 10 0 0 0 0 1
slotdef=2:5,6,8,4,7
T 48300 47100 5 10 0 0 0 0 1
footprint=DIP8
}
C 47800 47300 1 270 0 capacitor-1.sym
{
T 48500 47100 5 10 0 0 270 0 1
device=CAPACITOR
T 48100 47300 5 10 1 1 270 0 1
refdes=C2
T 48700 47100 5 10 0 0 270 0 1
symversion=0.1
T 48100 46800 5 10 1 1 270 0 1
value=100n
T 47800 47300 5 10 0 0 0 0 1
footprint=RECT 200
}
N 51500 47100 51500 46000 4
N 46800 46000 54500 46000 4
N 51500 47900 51500 49000 4
N 41500 49000 53400 49000 4
N 48800 47900 48800 49000 4
N 48800 47100 48800 46000 4
N 47200 47300 48300 47300 4
N 48000 46400 49300 46400 4
N 49300 46400 49300 47500 4
N 51000 47300 51000 47000 4
N 51000 47000 53000 47000 4
C 47300 49000 1 270 0 resistor-1.sym
{
T 47700 48700 5 10 0 0 270 0 1
device=RESISTOR
T 47500 48900 5 10 1 1 270 0 1
refdes=R1
T 47500 48500 5 10 1 1 270 0 1
value=10k
T 47300 49000 5 10 0 0 0 0 1
footprint=AXIAL_LAY 300
}
C 46300 47200 1 0 0 resistor-1.sym
{
T 46600 47600 5 10 0 0 0 0 1
device=RESISTOR
T 46300 47400 5 10 1 1 0 0 1
refdes=R2
T 46300 47200 5 10 0 0 0 0 1
footprint=AXIAL_LAY 300
T 46800 47400 5 10 1 1 0 0 1
value=2150
}
C 46600 46900 1 270 0 capacitor-2.sym
{
T 47300 46700 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 47000 46800 5 10 1 1 270 0 1
refdes=C3
T 47500 46700 5 10 0 0 270 0 1
symversion=0.1
T 47000 46400 5 10 1 1 270 0 1
value=10u
T 46600 46900 5 10 0 0 0 0 1
footprint=RCY100P
}
C 53600 47000 1 0 0 capacitor-2.sym
{
T 53800 47700 5 10 0 0 0 0 1
device=POLARIZED_CAPACITOR
T 53600 47500 5 10 1 1 0 0 1
refdes=C1
T 53800 47900 5 10 0 0 0 0 1
symversion=0.1
T 53600 47000 5 10 0 0 0 0 1
footprint=RCY100P
}
C 53200 46900 1 270 0 capacitor-2.sym
{
T 53900 46700 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 53700 46900 5 10 1 1 270 0 1
refdes=C4
T 54100 46700 5 10 0 0 270 0 1
symversion=0.1
T 53700 46400 5 10 1 1 270 0 1
value=100u
T 53200 46900 5 10 0 0 0 0 1
footprint=RCY200P
}
N 46800 46900 47400 46900 4
N 47400 46900 47400 48100 4
N 48300 47700 47400 47700 4
N 53400 46900 53400 49000 4
C 54500 45300 1 0 0 speaker-1.sym
{
T 56500 47800 5 10 0 0 0 0 1
device=SPEAKER
T 55100 47300 5 10 1 1 0 0 1
refdes=SPK1
}
N 53000 47200 53600 47200 4
C 41500 45500 1 0 0 ATtiny25_45_85.sym
{
T 45400 48100 5 10 1 1 0 6 1
refdes=U2
T 41800 48550 5 10 0 0 0 0 1
device=ATtiny25_45_85
T 41800 48750 5 10 0 0 0 0 1
footprint=DIP8
T 41500 45500 5 10 0 1 0 0 1
net=+5V:8
}
N 46300 47300 45900 47300 4
C 41500 44500 1 0 0 switch-pushbutton-no-1.sym
{
T 41900 44800 5 10 1 1 0 0 1
refdes=S1
T 41900 45100 5 10 0 0 0 0 1
device=SWITCH_PUSHBUTTON_NO
T 41500 44500 5 10 0 0 0 0 1
footprint=RECT 300
}
C 42400 44200 1 0 0 gnd-1.sym
C 47300 45700 1 0 0 gnd-1.sym
C 41300 49000 1 0 0 5V-plus-1.sym
N 41500 44500 41500 46100 4
N 46100 45600 46100 46900 4
N 46100 45600 50000 45600 4
C 47300 46900 1 270 0 resistor-1.sym
{
T 47700 46600 5 10 0 0 270 0 1
device=RESISTOR
T 47500 46900 5 10 1 1 270 0 1
refdes=R3
T 47500 46400 5 10 1 1 270 0 1
value=10k
T 47300 46900 5 10 0 0 0 0 1
footprint=AXIAL_LAY 300
}
N 41500 49000 41500 47000 4
C 41600 46100 1 90 0 resistor-1.sym
{
T 41200 46400 5 10 0 0 90 0 1
device=RESISTOR
T 41400 46100 5 10 1 1 90 0 1
refdes=R4
T 41400 46600 5 10 1 1 90 0 1
value=10k
T 41600 46100 5 10 0 0 180 0 1
footprint=AXIAL_LAY 300
}
C 50400 47400 1 0 1 resistor-1.sym
{
T 50100 47800 5 10 0 0 0 6 1
device=RESISTOR
T 49500 47600 5 10 1 1 0 0 1
refdes=R5
T 50000 47600 5 10 1 1 0 0 1
value=10k
T 50400 47400 5 10 0 0 0 6 1
footprint=AXIAL_LAY 300
}
C 50200 46900 1 270 0 capacitor-1.sym
{
T 50900 46700 5 10 0 0 270 0 1
device=CAPACITOR
T 50500 46900 5 10 1 1 270 0 1
refdes=C5
T 51100 46700 5 10 0 0 270 0 1
symversion=0.1
T 50500 46400 5 10 1 1 270 0 1
value=22n
T 50200 46900 5 10 0 0 0 0 1
footprint=RECT 200
}
N 50400 46900 50400 47700 4
N 50400 47700 51000 47700 4
N 49500 47500 49300 47500 4
N 46100 46900 45900 46900 4
N 50000 45600 50000 46900 4
N 50000 46900 50400 46900 4
